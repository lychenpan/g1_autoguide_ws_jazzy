from __future__ import annotations

import asyncio
import json
import logging
import os
import queue
import threading
import time
import uuid
import wave
from dataclasses import dataclass
from typing import Optional
import sys
sys.path.append("/home/unitree/workspace/unitree_sdk2_python")

_PING_DISABLED = frozenset({"none", "null", "", "off", "disable", "disabled"})

# Diagnostics (env): TTS_RECV_GAP_WARN_MS, TTS_RECV_WAIT_WARN_MS, TTS_PLAY_STALL_WARN_MS,
# TTS_RECV_TRACE=1 (log every PCM recv), TTS_PUT_WARN_MS (queue put slow).


def _ws_ping_from_env(name: str) -> Optional[float]:
    raw = os.environ.get(name, "none").strip().lower()
    if raw in _PING_DISABLED:
        return None
    return float(raw)


@dataclass
class UnitreePlayer:
    """
    Thin wrapper around Unitree G1 audio PlayStream API.

    Input PCM must be: PCM16LE, 16kHz, mono.
    """

    net_interface: str
    timeout_s: float = 10.0
    volume: int = 100

    def __post_init__(self) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

        ChannelFactoryInitialize(0, self.net_interface)
        self._client = AudioClient()
        self._client.SetTimeout(self.timeout_s)
        self._client.Init()
        self.set_volume(self.volume)

    def set_volume(self, volume: int) -> None:
        code = self._client.SetVolume(int(volume))
        code_i = int(code[0] if isinstance(code, tuple) else code)
        if code_i != 0:
            raise RuntimeError(f"Unitree SetVolume failed: {code_i}")

    def play_stream(self, app_name: str, stream_id: str, pcm: bytes) -> int:
        code, _ = self._client.PlayStream(app_name, stream_id, pcm)
        return int(code)

    def stop(self, app_name: str) -> int:
        return int(self._client.PlayStop(app_name))


class RemoteTTSPlayer:
    """
    Stream TTS PCM from remote `/ws/tts` and play on G1 speaker.

    - playtext(text): starts streaming+playback in background
    - pause(): immediately stops speaker output, but keeps buffering unplayed audio
    - reuse(): resumes speaker output from buffered audio (if not finished)
    """

    def __init__(
        self,
        *,
        ws_tts_url: str = "ws://112.95.75.67:10010/ws/tts",
        unitree_net_iface: Optional[str] = None,
        app_name: str = "ttsplayer",
        volume: int = 100,
        max_buffer_frames: int = 5000,  # about a few seconds depending on server chunking
        save_wav_dir: Optional[str] = None,
        play_tail_s: float = 0.25,
        play_buffer_s: Optional[float] = None,
        ws_ping_interval: Optional[float] = None,
        ws_ping_timeout: Optional[float] = None,
        ws_open_timeout: Optional[float] = None,
    ) -> None:
        self._ws_tts_url = ws_tts_url
        # Ping disabled by default (matches server); set WS_PING_INTERVAL=60 to re-enable.
        self._ws_ping_interval = (
            ws_ping_interval
            if ws_ping_interval is not None
            else _ws_ping_from_env("WS_PING_INTERVAL")
        )
        self._ws_ping_timeout = (
            ws_ping_timeout
            if ws_ping_timeout is not None
            else _ws_ping_from_env("WS_PING_TIMEOUT")
        )
        self._ws_open_timeout = (
            float(ws_open_timeout)
            if ws_open_timeout is not None
            else float(os.environ.get("WS_OPEN_TIMEOUT", "30"))
        )
        if unitree_net_iface is None:
            unitree_net_iface = os.environ.get("UNITREE_NET_IFACE", "eth0")
        self._player = UnitreePlayer(unitree_net_iface, volume=volume)
        self._app_name = app_name
        self._log = logging.getLogger(self.__class__.__name__)
        self._save_wav_dir = save_wav_dir
        self._play_tail_s = float(play_tail_s)
        buf_s = (
            float(play_buffer_s)
            if play_buffer_s is not None
            else float(os.environ.get("PLAY_BUFFER_S", "0.75"))
        )
        self._play_buffer_bytes = max(0, int(buf_s * (16000 * 2)))

        self._q: "queue.Queue[Optional[bytes]]" = queue.Queue(maxsize=max_buffer_frames)
        self._play_allowed = threading.Event()
        self._play_allowed.set()

        self._stop_all = threading.Event()
        self._session_lock = threading.Lock()
        self._active = False
        self._done = threading.Event()
        self._end_received = threading.Event()
        self._recv_finished = threading.Event()

        self._count_lock = threading.Lock()
        self._frames_in = 0
        self._bytes_in = 0
        self._audio_bps = 16000 * 2 * 1  # PCM16LE, 16kHz, mono => 32000 bytes/sec

        self._recv_thread: Optional[threading.Thread] = None
        self._play_thread: Optional[threading.Thread] = None
        self._current_stream_id = uuid.uuid4().hex
        self._session_id = uuid.uuid4().hex[:8]
        self._wav_path: Optional[str] = None

        self._diag_lock = threading.Lock()
        self._last_recv_mono: Optional[float] = None
        self._last_play_mono: Optional[float] = None
        self._recv_gap_warn_s = float(os.environ.get("TTS_RECV_GAP_WARN_MS", "500")) / 1000.0
        self._recv_wait_warn_s = float(os.environ.get("TTS_RECV_WAIT_WARN_MS", "500")) / 1000.0
        self._play_stall_warn_s = float(os.environ.get("TTS_PLAY_STALL_WARN_MS", "300")) / 1000.0
        self._put_warn_s = float(os.environ.get("TTS_PUT_WARN_MS", "200")) / 1000.0
        self._recv_trace = os.environ.get("TTS_RECV_TRACE", "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )

    def _reset_diag(self) -> None:
        with self._diag_lock:
            self._last_recv_mono = None
            self._last_play_mono = None

    def _diag_snapshot(self, bytes_out: int = 0) -> dict[str, int | str]:
        with self._count_lock:
            bytes_in = self._bytes_in
            frames_in = self._frames_in
        return {
            "qsize": self._safe_qsize(),
            "bytes_in": bytes_in,
            "frames_in": frames_in,
            "pending_play": max(0, bytes_in - bytes_out),
        }

    def playtext(self, text: str) -> None:
        text = str(text).strip()
        if not text:
            return

        with self._session_lock:
            if self._active:
                self.stop()
            self._stop_all.clear()
            self._done.clear()
            self._active = True
            self._current_stream_id = uuid.uuid4().hex
            self._session_id = uuid.uuid4().hex[:8]
            self._end_received.clear()
            self._recv_finished.clear()
            with self._count_lock:
                self._frames_in = 0
                self._bytes_in = 0
            self._reset_diag()
            self._wav_path = self._build_wav_path()

            self._log.info(
                "playtext start session=%s chars=%d url=%s app=%s stream_id=%s play_buffer_bytes=%d",
                self._session_id,
                len(text),
                self._ws_tts_url,
                self._app_name,
                self._current_stream_id[:8],
                self._play_buffer_bytes,
            )
            self._recv_thread = threading.Thread(
                target=self._recv_loop_thread, args=(text,), daemon=True
            )
            self._play_thread = threading.Thread(target=self._play_loop_thread, daemon=True)
            self._recv_thread.start()
            self._play_thread.start()

    def pause(self) -> None:
        """
        Suspend speaker output immediately.
        Incoming audio is still received and buffered.
        """
        self._log.info("pause session=%s", self._session_id)
        self._play_allowed.clear()
        try:
            self._player.stop(self._app_name)
        except Exception:
            # Best-effort; pause must be robust.
            pass

    def reuse(self) -> None:
        """Resume speaker output if there is buffered/unplayed audio."""
        # New stream id after stop makes resume more reliable on some firmware.
        self._current_stream_id = uuid.uuid4().hex
        self._log.info(
            "reuse session=%s new_stream_id=%s qsize=%s",
            self._session_id,
            self._current_stream_id[:8],
            self._safe_qsize(),
        )
        self._play_allowed.set()

    def is_playing(self) -> bool:
        return self._active and (not self._done.is_set())

    def wait_done(self, timeout_s: Optional[float] = None) -> bool:
        """Block until the current playtext finishes (or timeout)."""
        return bool(self._done.wait(timeout=timeout_s))

    def stop(self) -> None:
        """
        Stop everything and clear buffered audio.
        After stop(), reuse() will do nothing until playtext() is called again.
        """
        with self._session_lock:
            if not self._active:
                return
            self._log.info(
                "stop requested session=%s qsize=%s stream_id=%s",
                self._session_id,
                self._safe_qsize(),
                self._current_stream_id[:8],
            )
            self._stop_all.set()
            self._play_allowed.set()
            self._end_received.clear()
            self._recv_finished.set()
            self._drain_queue()
            try:
                self._player.stop(self._app_name)
            except Exception:
                pass

        # Allow threads to exit quickly.
        try:
            self._q.put_nowait(None)
        except queue.Full:
            pass

        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)
        if self._play_thread and self._play_thread.is_alive():
            self._play_thread.join(timeout=1.0)

        with self._session_lock:
            self._active = False
            self._done.set()
        self._log.info("stop finished session=%s", self._session_id)

    def _drain_queue(self) -> None:
        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            return

    def _safe_qsize(self) -> str:
        try:
            return str(self._q.qsize())
        except Exception:
            return "?"

    def _pending_playback_bytes(self, bytes_out: int) -> int:
        with self._count_lock:
            return max(0, self._bytes_in - bytes_out)

    def _stream_draining(self) -> bool:
        return self._end_received.is_set() and self._recv_finished.is_set()

    def _build_wav_path(self) -> Optional[str]:
        if not self._save_wav_dir:
            return None
        try:
            os.makedirs(self._save_wav_dir, exist_ok=True)
        except Exception as e:
            self._log.warning("wav dir create failed dir=%s err=%r", self._save_wav_dir, e)
            return None
        ts = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        fn = f"tts_{ts}_session-{self._session_id}.wav"
        return os.path.join(self._save_wav_dir, fn)

    def _recv_loop_thread(self, text: str) -> None:
        asyncio.run(self._recv_loop_async(text))

    async def _recv_loop_async(self, text: str) -> None:
        import websockets

        request_id = uuid.uuid4().hex[:8]
        bytes_in = 0
        frames_in = 0
        last_progress_log = time.monotonic()
        start_t = time.monotonic()
        wav_writer: Optional[wave.Wave_write] = None
        try:
            self._log.info(
                "ws connect session=%s request_id=%s url=%s",
                self._session_id,
                request_id,
                self._ws_tts_url,
            )
            async with websockets.connect(
                self._ws_tts_url,
                max_size=None,
                ping_interval=self._ws_ping_interval,  # None = disabled
                ping_timeout=self._ws_ping_timeout,
                open_timeout=self._ws_open_timeout,
                close_timeout=10,
            ) as ws:
                payload = {"text": text, "request_id": request_id}
                await ws.send(json.dumps(payload, ensure_ascii=False))
                self._log.info(
                    "ws sent session=%s request_id=%s chars=%d",
                    self._session_id,
                    request_id,
                    len(text),
                )
                if self._wav_path:
                    try:
                        wav_writer = wave.open(self._wav_path, "wb")
                        wav_writer.setnchannels(1)
                        wav_writer.setsampwidth(2)  # PCM16LE
                        wav_writer.setframerate(16000)
                        self._log.info(
                            "wav saving enabled session=%s path=%s",
                            self._session_id,
                            self._wav_path,
                        )
                    except Exception as e:
                        self._log.warning(
                            "wav open failed session=%s path=%s err=%r",
                            self._session_id,
                            self._wav_path,
                            e,
                        )
                        wav_writer = None

                # Expect a JSON "start" then binary frames then JSON "end"
                while not self._stop_all.is_set():
                    recv_wait_t0 = time.monotonic()
                    msg = await ws.recv()
                    recv_wait_ms = (time.monotonic() - recv_wait_t0) * 1000.0
                    if isinstance(msg, bytes):
                        now = time.monotonic()
                        recv_t_rel = now - start_t
                        gap_ms: Optional[float] = None
                        with self._diag_lock:
                            if self._last_recv_mono is not None:
                                gap_ms = (now - self._last_recv_mono) * 1000.0
                            self._last_recv_mono = now

                        if recv_wait_ms >= self._recv_wait_warn_s * 1000.0:
                            snap = self._diag_snapshot()
                            self._log.warning(
                                "RECV_WAIT_SLOW session=%s request_id=%s frame=%d "
                                "recv_wait_ms=%.0f gap_ms=%s nbytes=%d t_rel=%.3fs "
                                "qsize=%s pending_play=%s "
                                "(blocked in ws.recv — network or server not sending)",
                                self._session_id,
                                request_id,
                                frames_in + 1,
                                recv_wait_ms,
                                f"{gap_ms:.0f}" if gap_ms is not None else "n/a",
                                len(msg),
                                recv_t_rel,
                                snap["qsize"],
                                snap["pending_play"],
                            )

                        if gap_ms is not None and gap_ms >= self._recv_gap_warn_s * 1000.0:
                            snap = self._diag_snapshot()
                            self._log.warning(
                                "RECV_GAP session=%s request_id=%s frame=%d "
                                "gap_ms=%.0f nbytes=%d t_rel=%.3fs qsize=%s pending_play=%s "
                                "(inter-arrival gap — likely network jitter or server synthesis/send stall)",
                                self._session_id,
                                request_id,
                                frames_in + 1,
                                gap_ms,
                                len(msg),
                                recv_t_rel,
                                snap["qsize"],
                                snap["pending_play"],
                            )

                        if self._recv_trace or frames_in < 3:
                            self._log.info(
                                "RECV_PCM session=%s request_id=%s frame=%d t_rel=%.3fs "
                                "recv_wait_ms=%.0f gap_ms=%s nbytes=%d qsize=%s",
                                self._session_id,
                                request_id,
                                frames_in + 1,
                                recv_t_rel,
                                recv_wait_ms,
                                f"{gap_ms:.0f}" if gap_ms is not None else "n/a",
                                len(msg),
                                self._safe_qsize(),
                            )

                        # Backpressure is OK: block until playback consumes.
                        bytes_in += len(msg)
                        frames_in += 1
                        with self._count_lock:
                            self._bytes_in = bytes_in
                            self._frames_in = frames_in
                        put_t0 = time.monotonic()
                        self._q.put(msg)
                        put_ms = (time.monotonic() - put_t0) * 1000.0
                        if put_ms >= self._put_warn_s * 1000.0:
                            snap = self._diag_snapshot()
                            self._log.warning(
                                "RECV_QUEUE_BLOCK session=%s request_id=%s frame=%d "
                                "put_wait_ms=%.0f qsize=%s pending_play=%s "
                                "(play thread slow — local backlog, not network)",
                                self._session_id,
                                request_id,
                                frames_in,
                                put_ms,
                                snap["qsize"],
                                snap["pending_play"],
                            )
                        if wav_writer is not None:
                            try:
                                wav_writer.writeframes(msg)
                            except Exception as e:
                                self._log.warning(
                                    "wav write failed session=%s err=%r (disabling wav write)",
                                    self._session_id,
                                    e,
                                )
                                try:
                                    wav_writer.close()
                                except Exception:
                                    pass
                                wav_writer = None
                        now = time.monotonic()
                        if now - last_progress_log >= 1.0:
                            self._log.info(
                                "ws recv session=%s request_id=%s frames=%d bytes=%d qsize=%s",
                                self._session_id,
                                request_id,
                                frames_in,
                                bytes_in,
                                self._safe_qsize(),
                            )
                            last_progress_log = now
                        continue

                    if recv_wait_ms >= self._recv_wait_warn_s * 1000.0:
                        self._log.warning(
                            "RECV_WAIT_SLOW session=%s request_id=%s kind=json "
                            "recv_wait_ms=%.0f t_rel=%.3fs (blocked in ws.recv)",
                            self._session_id,
                            request_id,
                            recv_wait_ms,
                            time.monotonic() - start_t,
                        )

                    j = json.loads(msg)
                    t = j.get("type")
                    if t == "error":
                        raise RuntimeError(j.get("message") or "remote tts error")
                    if t == "start":
                        self._log.info(
                            "ws start session=%s request_id=%s meta=%s",
                            self._session_id,
                            request_id,
                            {k: v for k, v in j.items() if k != "type"},
                        )
                    if t == "end":
                        self._end_received.set()
                        self._log.info(
                            "ws end session=%s request_id=%s frames=%d bytes=%d elapsed=%.2fs",
                            self._session_id,
                            request_id,
                            frames_in,
                            bytes_in,
                            time.monotonic() - start_t,
                        )
                        break

        except Exception as e:
            # IMPORTANT: don't swallow exceptions; they are the #1 reason playback "mysteriously stops".
            self._log.exception(
                "ws failed session=%s request_id=%s frames=%d bytes=%d elapsed=%.2fs err=%r",
                self._session_id,
                request_id,
                frames_in,
                bytes_in,
                time.monotonic() - start_t,
                e,
            )
        finally:
            if wav_writer is not None:
                try:
                    wav_writer.close()
                    self._log.info(
                        "wav saved session=%s path=%s frames=%d bytes=%d",
                        self._session_id,
                        self._wav_path,
                        frames_in,
                        bytes_in,
                    )
                except Exception as e:
                    self._log.warning(
                        "wav close failed session=%s path=%s err=%r",
                        self._session_id,
                        self._wav_path,
                        e,
                    )
            self._recv_finished.set()
            # Wake playback thread (it decides when it's truly "done").
            try:
                self._q.put_nowait(None)
            except queue.Full:
                # If full, wait a bit then try again.
                try:
                    self._q.put(None, timeout=0.5)
                except Exception:
                    pass

    def _play_loop_thread(self) -> None:
        frames_out = 0
        bytes_out = 0
        consecutive_rc_fail = 0
        last_progress_log = time.monotonic()
        start_t = time.monotonic()
        first_play_t: Optional[float] = None
        finished_naturally = False
        buffer_wait_logged = False
        queue_stall_logged = False
        try:
            while True:
                if self._stop_all.is_set():
                    self._log.info("play loop stop_all session=%s", self._session_id)
                    break

                # Let recv run ahead of PlayStream to absorb network jitter (PLAY_BUFFER_S).
                buffer_wait_t0: Optional[float] = None
                while (
                    (not self._stop_all.is_set())
                    and (not self._stream_draining())
                    and self._pending_playback_bytes(bytes_out) < self._play_buffer_bytes
                ):
                    if buffer_wait_t0 is None:
                        buffer_wait_t0 = time.monotonic()
                    elif (
                        not buffer_wait_logged
                        and (time.monotonic() - buffer_wait_t0) >= self._play_stall_warn_s
                    ):
                        snap = self._diag_snapshot(bytes_out)
                        self._log.warning(
                            "PLAY_STALL_BUFFER session=%s waited_ms=%.0f "
                            "pending_play=%d target=%d bytes_out=%d qsize=%s frames_in=%s "
                            "(intentional pre-roll — waiting for PLAY_BUFFER before first/next play)",
                            self._session_id,
                            (time.monotonic() - buffer_wait_t0) * 1000.0,
                            snap["pending_play"],
                            self._play_buffer_bytes,
                            bytes_out,
                            snap["qsize"],
                            snap["frames_in"],
                        )
                        buffer_wait_logged = True
                    time.sleep(0.01)
                buffer_wait_logged = False

                try:
                    frame = self._q.get(timeout=0.1)
                except queue.Empty:
                    frame = None

                if frame is None:
                    if (
                        first_play_t is not None
                        and not self._stream_draining()
                        and not queue_stall_logged
                    ):
                        with self._diag_lock:
                            since_play_ms = (
                                (time.monotonic() - self._last_play_mono) * 1000.0
                                if self._last_play_mono is not None
                                else 0.0
                            )
                        if since_play_ms >= self._play_stall_warn_s * 1000.0:
                            snap = self._diag_snapshot(bytes_out)
                            self._log.warning(
                                "PLAY_STALL_QUEUE_EMPTY session=%s since_last_play_ms=%.0f "
                                "bytes_out=%d/%s qsize=%s pending_play=%s recv_done=%s end=%s "
                                "(no PCM in queue — likely waiting on network/recv; check RECV_GAP/RECV_WAIT_SLOW)",
                                self._session_id,
                                since_play_ms,
                                bytes_out,
                                snap["bytes_in"],
                                snap["qsize"],
                                snap["pending_play"],
                                self._recv_finished.is_set(),
                                self._end_received.is_set(),
                            )
                            queue_stall_logged = True
                    # `None` is a wake-up marker OR a timeout. Only finish when:
                    # - server has sent `end`
                    # - recv thread is finished
                    # - everything received has been played
                    if self._end_received.is_set() and self._recv_finished.is_set():
                        with self._count_lock:
                            frames_in = self._frames_in
                            bytes_in = self._bytes_in
                        if frames_out >= frames_in and self._q.empty():
                            # We have submitted all received PCM to the device.
                            # The device may still be playing buffered audio; estimate remaining time
                            # from received PCM duration vs elapsed since first successful PlayStream.
                            if first_play_t is not None and bytes_in > 0:
                                audio_len_s = bytes_in / float(self._audio_bps)
                                elapsed_s = time.monotonic() - first_play_t
                                remaining_s = audio_len_s - elapsed_s
                                if remaining_s > 0:
                                    self._log.info(
                                        "play tail sleep session=%s audio_len=%.3fs elapsed=%.3fs sleep=%.3fs",
                                        self._session_id,
                                        audio_len_s,
                                        elapsed_s,
                                        remaining_s,
                                    )
                                    time.sleep(remaining_s)
                            self._log.info(
                                "play loop done session=%s frames_in=%d frames_out=%d bytes_in=%d bytes_out=%d elapsed=%.2fs",
                                self._session_id,
                                frames_in,
                                frames_out,
                                bytes_in,
                                bytes_out,
                                time.monotonic() - start_t,
                            )
                            finished_naturally = True
                            break
                    continue

                queue_stall_logged = False

                # Pause gates speaker output without losing buffered frames.
                while (not self._play_allowed.is_set()) and (not self._stop_all.is_set()):
                    time.sleep(0.01)
                if self._stop_all.is_set():
                    self._log.info("play loop stop during pause session=%s", self._session_id)
                    break

                rc = self._player.play_stream(self._app_name, self._current_stream_id, frame)
                if rc != 0:
                    # Non-fatal; keep trying next frames.
                    consecutive_rc_fail += 1
                    if consecutive_rc_fail in (1, 5, 20) or (consecutive_rc_fail % 50 == 0):
                        self._log.warning(
                            "PlayStream rc=%d session=%s stream_id=%s fails=%d qsize=%s frame_bytes=%d",
                            rc,
                            self._session_id,
                            self._current_stream_id[:8],
                            consecutive_rc_fail,
                            self._safe_qsize(),
                            len(frame),
                        )
                    time.sleep(0.01)
                    continue

                consecutive_rc_fail = 0
                now = time.monotonic()
                play_gap_ms: Optional[float] = None
                with self._diag_lock:
                    if self._last_play_mono is not None:
                        play_gap_ms = (now - self._last_play_mono) * 1000.0
                    self._last_play_mono = now

                if (
                    first_play_t is not None
                    and play_gap_ms is not None
                    and play_gap_ms >= self._play_stall_warn_s * 1000.0
                ):
                    snap = self._diag_snapshot(bytes_out)
                    self._log.warning(
                        "PLAY_GAP session=%s gap_ms=%.0f frame_bytes=%d "
                        "bytes_out=%d/%s qsize=%s pending_play=%s "
                        "(long gap between PlayStream calls — queue was empty or buffer wait)",
                        self._session_id,
                        play_gap_ms,
                        len(frame),
                        bytes_out,
                        snap["bytes_in"],
                        snap["qsize"],
                        snap["pending_play"],
                    )

                if first_play_t is None:
                    first_play_t = now
                    self._log.info(
                        "first play accepted session=%s stream_id=%s pending_bytes=%d buffer_target=%d",
                        self._session_id,
                        self._current_stream_id[:8],
                        self._pending_playback_bytes(bytes_out),
                        self._play_buffer_bytes,
                    )
                frames_out += 1
                bytes_out += len(frame)
                if now - last_progress_log >= 1.0:
                    with self._count_lock:
                        frames_in = self._frames_in
                        bytes_in = self._bytes_in
                    self._log.info(
                        "play ok session=%s frames_out=%d/%d bytes_out=%d/%d qsize=%s stream_id=%s",
                        self._session_id,
                        frames_out,
                        frames_in,
                        bytes_out,
                        bytes_in,
                        self._safe_qsize(),
                        self._current_stream_id[:8],
                    )
                    last_progress_log = now
        finally:
            # If we stop the device immediately on natural completion, it can cut off
            # the last buffered audio on the speaker side. Only force-stop on explicit stop().
            if finished_naturally:
                if self._play_tail_s > 0 and first_play_t is not None:
                    # Extra safety margin (optional) for device buffering jitter.
                    time.sleep(self._play_tail_s)
            else:
                try:
                    self._player.stop(self._app_name)
                except Exception:
                    pass
            with self._session_lock:
                self._active = False
                self._done.set()
            self._log.info(
                "play loop finished session=%s natural=%s frames_out=%d bytes_out=%d elapsed=%.2fs",
                self._session_id,
                finished_naturally,
                frames_out,
                bytes_out,
                time.monotonic() - start_t,
            )


if __name__ == "__main__":
    # Minimal manual test:
    #   python3 tts_player.py "hello world"
    import sys

    logging.basicConfig(
        level=getattr(logging, os.environ.get("TTS_LOG_LEVEL", "INFO").upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    text = open("../test/1.txt").read().splitlines()
    test = text
    test = "\n".join(test)
    print(test)
    tts = RemoteTTSPlayer()
    tts.playtext(test)
    tts.wait_done()
    print("wait done")
