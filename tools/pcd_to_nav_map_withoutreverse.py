#!/usr/bin/env python3
"""
Convert 3D PCD map from FAST_LIO to 2D occupancy grid for Nav2 navigation

Optimized for Unitree G1 robot with Livox Mid360 LiDAR (Z-DOWN orientation)

Usage:
    python3 pcd_to_nav_map.py <input.pcd> <output_prefix>
    
Example:
    python3 pcd_to_nav_map.py ../maps/test.pcd ../maps/nav_map
    
This will create:
    - nav_map.png (occupancy grid image)
    - nav_map.yaml (map metadata for Nav2)

Coordinate System (G1 Robot):
    - Mid360 mounted at 1.23m height, facing DOWN
    - Z=0 at sensor, Z increases downward
    - Floor is at Z ≈ 1.23m
    - Obstacles are BELOW floor level (Z < 1.23m)
"""

import sys
import struct
import numpy as np
import yaml
from PIL import Image


def read_pcd_file(filename):
    """
    Read PCD file in either ASCII or binary format
    
    Returns: numpy array of points (N x 3: x, y, z)
    """
    with open(filename, 'rb') as f:
        # Read header
        header = {}
        data_format = None
        num_points = 0
        fields = []
        field_sizes = []
        field_types = []
        field_counts = []
        
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            
            if not line:
                continue
            
            if line.startswith('VERSION'):
                header['version'] = line.split()[1]
            elif line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                field_sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('TYPE'):
                field_types = line.split()[1:]
            elif line.startswith('COUNT'):
                field_counts = [int(x) for x in line.split()[1:]]
            elif line.startswith('WIDTH'):
                header['width'] = int(line.split()[1])
            elif line.startswith('HEIGHT'):
                header['height'] = int(line.split()[1])
            elif line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                data_format = line.split()[1]
                break
        
        # Find x, y, z field indices
        try:
            x_idx = fields.index('x')
            y_idx = fields.index('y')
            z_idx = fields.index('z')
        except ValueError:
            print("❌ PCD file doesn't have x, y, z fields!")
            return None
        
        # Calculate point size
        point_size = sum(field_sizes[i] * field_counts[i] for i in range(len(fields)))
        
        print(f"   Format: {data_format}")
        print(f"   Fields: {', '.join(fields)}")
        print(f"   Points: {num_points:,}")
        print(f"   Point size: {point_size} bytes")
        
        # Read data
        points = []
        
        if data_format.lower() == 'binary':
            # Read binary data
            data = f.read()
            
            # Unpack binary data
            for i in range(num_points):
                offset = i * point_size
                point_data = data[offset:offset + point_size]
                
                if len(point_data) < point_size:
                    break
                
                # Extract x, y, z values
                values = []
                byte_offset = 0
                
                for field_idx in range(len(fields)):
                    size = field_sizes[field_idx]
                    ftype = field_types[field_idx]
                    count = field_counts[field_idx]
                    
                    if ftype == 'F':  # Float
                        if size == 4:
                            fmt = 'f'
                        elif size == 8:
                            fmt = 'd'
                        else:
                            fmt = 'f'
                    elif ftype == 'U':  # Unsigned int
                        if size == 1:
                            fmt = 'B'
                        elif size == 2:
                            fmt = 'H'
                        elif size == 4:
                            fmt = 'I'
                        elif size == 8:
                            fmt = 'Q'
                        else:
                            fmt = 'I'
                    elif ftype == 'I':  # Signed int
                        if size == 1:
                            fmt = 'b'
                        elif size == 2:
                            fmt = 'h'
                        elif size == 4:
                            fmt = 'i'
                        elif size == 8:
                            fmt = 'q'
                        else:
                            fmt = 'i'
                    else:
                        fmt = 'f'
                    
                    for _ in range(count):
                        value = struct.unpack('<' + fmt, point_data[byte_offset:byte_offset + size])[0]
                        values.append(value)
                        byte_offset += size
                
                # Extract x, y, z
                x = values[x_idx]
                y = values[y_idx]
                z = values[z_idx]
                
                points.append([x, y, z])
            
        else:  # ASCII format
            for line in f:
                try:
                    line_str = line.decode('ascii').strip()
                    if not line_str:
                        continue
                    parts = line_str.split()
                    if len(parts) >= max(x_idx, y_idx, z_idx) + 1:
                        x = float(parts[x_idx])
                        y = float(parts[y_idx])
                        z = float(parts[z_idx])
                        points.append([x, y, z])
                except:
                    continue
    
    return np.array(points)

def save_points_to_pcd(points, output_file, points_format='binary'):
    """
    Save 3D points to a PCD file in binary format
    
    Args:
        points: Nx3 numpy array of points (x, y, z coordinates)
        output_file: Output PCD file path
        points_format: 'binary' or 'ascii' format (default: 'binary')
    
    Returns:
        True if successful, False otherwise
    """
    if points is None or len(points) == 0:
        print("❌ No points to save!")
        return False
    
    try:
        # Create directory if it doesn't exist
        import os
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
            print(f"📁 Created directory: {output_dir}")
        
        num_points = len(points)
        
        with open(output_file, 'wb') as f:
            # Write PCD header
            f.write(b'# .PCD v.7 - Point Cloud Data file format\n')
            f.write(f'VERSION .7\n'.encode())
            f.write(b'FIELDS x y z\n')
            f.write(b'SIZE 4 4 4\n')
            f.write(b'TYPE F F F\n')
            f.write(b'COUNT 1 1 1\n')
            f.write(f'WIDTH {num_points}\n'.encode())
            f.write(b'HEIGHT 1\n')
            f.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {num_points}\n'.encode())
            f.write(f'DATA {points_format}\n'.encode())
            
            if points_format.lower() == 'binary':
                # Write points in binary format (little-endian floats)
                for point in points:
                    x, y, z = point[0], point[1], point[2]
                    f.write(struct.pack('<fff', float(x), float(y), float(z)))
            else:
                # Write points in ASCII format
                for point in points:
                    x, y, z = point[0], point[1], point[2]
                    f.write(f'{x} {y} {z}\n'.encode())
        
        print(f"✅ Saved {num_points:,} points to PCD file: {output_file}")
        return True
        
    except Exception as e:
        print(f"❌ Error saving PCD file: {e}")
        return False


def statistical_outlier_filter(points, k=50, std_ratio=1.0):
    """
    Remove outlier points using statistical analysis
    
    Args:
        points: Nx3 numpy array
        k: Number of neighbors to analyze (default 50)
        std_ratio: Standard deviation multiplier for outlier threshold (default 2.0)
    
    Returns:
        filtered_points: Points with outliers removed
        num_removed: Number of points removed
    """
    from scipy.spatial import cKDTree
    
    if len(points) < k:
        return points, 0
    
    print(f"🧹 Applying statistical outlier filter (k={k}, std_ratio={std_ratio})...")
    
    # Build KD-tree for efficient nearest neighbor search
    tree = cKDTree(points)
    
    # Mean distance to k nearest neighbors per point. Query in row batches so we
    # never allocate (N, k+1) distances at once — that exhausts RAM on large clouds
    # (e.g. 6.7M × 501 × 8 B ≈ 25 GiB).
    n = len(points)
    max_chunk_bytes = 128 * 1024 * 1024  # cap temporary distance buffer ~128 MiB
    query_batch = max(1, int(max_chunk_bytes / ((k + 1) * np.dtype(np.float64).itemsize)))
    mean_distances = np.empty(n, dtype=np.float64)
    for start in range(0, n, query_batch):
        end = min(start + query_batch, n)
        distances, _ = tree.query(points[start:end], k=k + 1)
        mean_distances[start:end] = distances[:, 1:].mean(axis=1)  # exclude self (col 0)
    
    # Compute global statistics
    global_mean = mean_distances.mean()
    global_std = mean_distances.std()
    
    # Filter: keep points within threshold
    threshold = global_mean + std_ratio * global_std
    mask = mean_distances < threshold
    
    filtered_points = points[mask]
    num_removed = len(points) - len(filtered_points)
    
    print(f"   Removed {num_removed:,} outlier points ({num_removed/len(points)*100:.1f}%)")
    print(f"   Kept {len(filtered_points):,} points")
    
    return filtered_points, num_removed


def voxel_downsample(points, voxel_size=0.05):
    """
    Downsample point cloud using voxel grid filter
    Reduces density and removes duplicate/overlapping points
    
    Args:
        points: Nx3 numpy array
        voxel_size: Size of voxel grid (meters)
    
    Returns:
        downsampled_points: Filtered points
    """
    if len(points) == 0:
        return points
    
    print(f"📦 Applying voxel downsampling (voxel_size={voxel_size}m)...")
    
    # Compute voxel indices
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)
    
    # Use unique to keep only one point per voxel
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    
    downsampled_points = points[unique_indices]
    num_removed = len(points) - len(downsampled_points)
    
    print(f"   Removed {num_removed:,} duplicate points ({num_removed/len(points)*100:.1f}%)")
    print(f"   Kept {len(downsampled_points):,} points")
    
    return downsampled_points


def pcd_to_occupancy_grid(pcd_file, output_prefix, resolution=0.05, 
                          height_min=-1.8, height_max=0.2,
                          filter_outliers=True, voxel_filter=True):
    """
    Convert 3D PCD to 2D occupancy grid
    
    Args:
        pcd_file: Input PCD file path
        output_prefix: Output file prefix (will create .png and .yaml)
        resolution: Grid resolution in meters (default 5cm)
        height_min: Minimum Z height for obstacles (default -0.8m for G1 robot)
        height_max: Maximum Z height for obstacles (default 1.13m for G1 robot)
        filter_outliers: Apply statistical outlier removal (default True)
        voxel_filter: Apply voxel downsampling to remove duplicates (default True)
    
    Note: For G1 robot with Mid360 at 1.23m height (Z-DOWN):
        - Floor is at Z ≈ 1.23m
        - Obstacles from 0.1m to 2.0m above floor → Z from 1.13m to -0.77m
        - Default captures obstacles from 0.1m to 2.0m above ground
    """
    
    print(f"📂 Loading PCD file: {pcd_file}")
    
    # Read PCD file (supports both ASCII and binary formats)
    points = read_pcd_file(pcd_file)

    #### x axis rotation
    rotation_matrix = np.array([
        [1,  0,  0],
        [0, 1,  0],
        [0,  0, 1]
    ])
    
    points =  np.dot(points, rotation_matrix.T)
    
    # save_points_to_pcd(points, pcd_file.replace(".pcd","-f2.pcd"))


    if points is None or len(points) == 0:
        print("❌ Failed to read PCD file or no points found!")
        return
    
    print(f"✅ Loaded {len(points)} points")
    
    # Apply voxel downsampling first (removes duplicates and overlaps)
    if voxel_filter:
        points = voxel_downsample(points, voxel_size=0.02)  # 2cm voxel
    
    # Apply statistical outlier removal (removes ghost points in empty space)
    if filter_outliers:
        # k≈20–50 matches typical PCL/Open3D SOR; very large k blows up RAM/time even with batched query
        points, _ = statistical_outlier_filter(points, k=50, std_ratio=1.0)
    
    # Analyze Z-axis range to understand coordinate system
    z_min_actual = points[:, 2].min()
    z_max_actual = points[:, 2].max()
    z_mean = points[:, 2].mean()
    z_std = points[:, 2].std()
    
    print(f"\n📊 Z-axis Analysis:")
    print(f"   Z range: {z_min_actual:.3f}m to {z_max_actual:.3f}m")
    print(f"   Z mean:  {z_mean:.3f}m (±{z_std:.3f}m)")
    
    # Detect coordinate system orientation
    if z_mean < 0:
        print(f"   ℹ️  Z-axis: Points UP (negative Z = down, ground likely near {z_min_actual:.2f}m)")
        coord_info = "up"
    else:
        print(f"   ℹ️  Z-axis: Points DOWN (positive Z = down, ground likely near {z_max_actual:.2f}m)")
        coord_info = "down"
    
    # Filter by height (only obstacles at robot's height)
    print(f"\n🔍 Filtering points between {height_min}m and {height_max}m height...")
    obstacle_points = points[(points[:, 2] > height_min) & (points[:, 2] < height_max)]
    print(f"✅ Found {len(obstacle_points)} obstacle points")
    
    if len(obstacle_points) == 0:
        print("❌ No obstacle points found! Adjust height_min/height_max parameters")
        exit()
        print(f"\n💡 Suggestions based on your Z-axis range [{z_min_actual:.2f}, {z_max_actual:.2f}]:")
        if coord_info == "down":
            # Z-axis points down, so ground is at maximum Z
            suggested_min = z_max_actual - 2.5  # 2.5m below ground
            suggested_max = z_max_actual - 0.1  # 10cm below ground (obstacles)
            print(f"   Try: --height-min {suggested_min:.2f} --height-max {suggested_max:.2f}")
            print(f"   (For Z-DOWN: obstacles are BELOW ground level)")
        else:
            # Z-axis points up, ground is at minimum Z
            suggested_min = z_min_actual + 0.1  # 10cm above ground
            suggested_max = z_min_actual + 2.5  # 2.5m above ground
            print(f"   Try: --height-min {suggested_min:.2f} --height-max {suggested_max:.2f}")
            print(f"   (For Z-UP: obstacles are ABOVE ground level)")
        return
    
    # Get 2D projection (X, Y only)
    xy_points = obstacle_points[:, :2]
    
    # Calculate grid bounds
    x_min, y_min = xy_points.min(axis=0)
    x_max, y_max = xy_points.max(axis=0)
    
    # Add margin around map
    margin = 1.0  # meters
    x_min -= margin
    y_min -= margin
    x_max += margin
    y_max += margin
    
    # Calculate grid size
    width = int((x_max - x_min) / resolution)
    height = int((y_max - y_min) / resolution)
    
    print(f"📐 Grid size: {width} x {height} pixels")
    print(f"📏 Coverage: {x_max-x_min:.2f}m x {y_max-y_min:.2f}m")
    print(f"📍 Origin: ({x_min:.2f}, {y_min:.2f})")
    
    # Initialize grid (-1 = unknown)
    grid = np.full((height, width), -1, dtype=np.int8)
    
    # Mark occupied cells (100 = occupied)
    print("🖌️  Marking occupied cells...")
    for point in xy_points:
        x_idx = int((point[0] - x_min) / resolution)
        y_idx = int((y_max - point[1]) / resolution)
        if 0 <= x_idx < width and 0 <= y_idx < height:
            grid[y_idx, x_idx] = 100
    
    # Simple inflation: mark cells around obstacles as occupied too
    # TODO: do this later
    # print("💨 Inflating obstacles...")
    # occupied_indices = np.argwhere(grid == 100)
    # inflation_radius = int(0.3 / resolution)  # 30cm robot radius
    
    # for idx in occupied_indices:
    #     y, x = idx
    #     for dy in range(-inflation_radius, inflation_radius + 1):
    #         for dx in range(-inflation_radius, inflation_radius + 1):
    #             ny, nx = y + dy, x + dx
    #             if 0 <= ny < height and 0 <= nx < width:
    #                 if dx*dx + dy*dy <= inflation_radius*inflation_radius:
    #                     grid[ny, nx] = 100
    
    # Mark remaining unknown cells as free (0 = free)
    print("🆓 Marking free space...")
    grid[grid == -1] = 0
    
    # Convert to image format
    # Nav2 expects: 0-254 (0=free/white, 100=occupied/black, 255=unknown/gray)
    img = np.zeros((height, width), dtype=np.uint8)
    img[grid == 0] = 254    # Free = white
    img[grid == 100] = 0    # Occupied = black
    img[grid == -1] = 205   # Unknown = gray (unused now)
    
    # Flip for image convention
    # IMPORTANT: ONLY flip vertically (flipud), NOT horizontally!
    # 
    # Why flipud? 
    #   - Grid is in map convention: Y-axis points UP (y_min at bottom)
    #   - PNG is in image convention: Y-axis points DOWN (0,0 at top-left)
    #   - flipud converts from map (Y-up) to image (Y-down)
    #
    # Why NOT fliplr?
    #   - FAST-LIO already outputs points in corrected 'odom' frame
    #   - The static TF (base_link→livox_frame) handles the inverted LiDAR
    #   - X and Y axes are already correct in the PCD data
    #   - No additional horizontal flip needed!
    # #
    # img = np.flipud(img)  # Map Y-up → Image Y-down
    # ## 
    # img = np.fliplr(img)   # Horizontal flip ← WRONG!
    
    # Save PNG
    output_png = f"{output_prefix}.png"
    
    # Create directory if it doesn't exist
    import os
    output_dir = os.path.dirname(output_png)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
        print(f"📁 Created directory: {output_dir}")
    
    Image.fromarray(img).save(output_png)
    print(f"✅ Saved occupancy grid image: {output_png}")
    
    origin_x = float(x_min)
    origin_y = float(y_max) - height * resolution  # 修正原点计算

    # Create YAML metadata for Nav2
    output_yaml = f"{output_prefix}.yaml"
    yaml_data = {
        'image': output_png.split('/')[-1],  # Relative filename
        'resolution': float(resolution),
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'mode': 'trinary'
    }
    
    with open(output_yaml, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    print(f"✅ Saved map metadata: {output_yaml}")
    
    print("\n✨ Conversion complete!")
    print(f"\n📋 To use with Nav2:")
    print(f"   ros2 launch slash_g1 g1_navigation.launch.py map_file:={output_yaml}")


def print_usage():
    print("Usage: python3 pcd_to_nav_map.py <input.pcd> <output_prefix> [options]")
    print()
    print("Arguments:")
    print("  input.pcd       : Input PCD file from FAST_LIO")
    print("  output_prefix   : Output file prefix (creates .png and .yaml)")
    print()
    print("Options:")
    print("  --resolution <m>    : Grid resolution in meters (default: 0.05)")
    print("  --height-min <m>    : Min Z height for obstacles (default: -0.8)")
    print("  --height-max <m>    : Max Z height for obstacles (default: 1.13)")
    print("  --no-filter         : Disable outlier and voxel filtering")
    print("  --no-outlier-filter : Disable only outlier removal")
    print("  --no-voxel-filter   : Disable only voxel downsampling")
    print()
    print("Note: Defaults are optimized for G1 robot (Mid360 at 1.23m height, Z-DOWN):")
    print("  • Default captures obstacles from 0.1m to 2.0m above ground")
    print("  • For Z-DOWN: Floor at Z≈1.23m, obstacles BELOW this value")
    print("  • Filtering enabled by default to remove ghost points and duplicates")
    print("  • Run script first to see Z-axis analysis, then adjust if needed")
    print()
    print("Examples:")
    print("  # Default (G1 robot with Z-DOWN, filtering enabled)")
    print("  python3 pcd_to_nav_map.py test.pcd nav_map")
    print()
    print("  # Disable all filtering (keep ghost points)")
    print("  python3 pcd_to_nav_map.py test.pcd nav_map --no-filter")
    print()
    print("  # Custom height range (capture 0.5m to 1.5m above floor)")
    print("  python3 pcd_to_nav_map.py test.pcd nav_map --height-min -0.27 --height-max 0.73")
    print()
    print("  # Higher resolution with stronger outlier filtering")
    print("  python3 pcd_to_nav_map.py test.pcd nav_map --resolution 0.01")


if __name__ == '__main__':
    import os
    if len(sys.argv) > 2:
        input_pcd = sys.argv[1]
        output_prefix = sys.argv[2]
    
    
    # Parse optional arguments
    # Defaults optimized for G1 robot: Mid360 at 1.23m height, Z-DOWN orientation
    # Captures obstacles from 0.1m to 2.0m above floor
    resolution = 0.01
    height_min = -1.23+0.1      # 2.0m above floor (1.23 - 2.0 = -0.77m)
    height_max = 0.1   # 0.1m above floor (1.23 - 0.1 = 1.13m)
    filter_outliers = True
    voxel_filter = True
    
    # t1 = "../maps/bks/"
    # for f in os.listdir(t1): guiderobot/slash_ws/maps/test_20251031_160047_597.pcd/home/unitree/workspace/guiderobot/slash_ws/maps/test_20251106_155113_441.pcd
    #  /home/unitree/workspace/guiderobot/slash_ws/m
    # slash_ws/maps/test_20251203_211149_851.pcd
    # input_pcd = "/home/unitree/workspace/guiderobot/slash_ws/maps/test_20251203_211149_851.pcd"
    input_pcd = "/home/unitree/workspace/global_map.pcd"

    output_prefix = "../maps/nav_map/" + input_pcd.split("/")[-1].replace(".pcd", "")
    try:
        pcd_to_occupancy_grid(input_pcd, output_prefix, resolution, height_min, height_max,
                            filter_outliers, voxel_filter)
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    # try:
    #     pcd_to_occupancy_grid(input_pcd, output_prefix, resolution, height_min, height_max,
    #                         filter_outliers, voxel_filter)
    # except Exception as e:
    #     print(f"❌ Error: {e}")
    #     import traceback
    #     traceback.print_exc()
    #     sys.exit(1)

