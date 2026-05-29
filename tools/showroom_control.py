import json
import urllib.request

BASE = "http://3fde93b9.r11.cpolar.top"
TIMEOUT = 30.0


def http_get(url: str) -> tuple[int, str]:
    req = urllib.request.Request(url, method="GET")
    with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
        code = getattr(resp, "status", 200)
        body = resp.read().decode(errors="replace").strip()
    return code, body


# 下一页（n=2）
code, text = http_get(f"{BASE}/api/agent/next?n=2")
print(code, text)
try:
    print(json.loads(text))
except json.JSONDecodeError:
    pass

# UDP PlayVideo
code, text = http_get(f"{BASE}/api/send_udp?command=PlayVideo")
print(code, text)