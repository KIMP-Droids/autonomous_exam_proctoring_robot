#!/usr/bin/env python3
"""
Simple script to estimate wheel radius from STL mesh bounding box.
This reads the wheel STL file and computes approximate radius.
"""

import sys
import os

def estimate_wheel_radius_from_stl(stl_path):
    """
    Estimate wheel radius by reading STL and computing bounding box.
    For a wheel, radius is approximately max(width, depth) / 2.
    """
    try:
        # Try using numpy-stl if available
        try:
            from stl import mesh
            stl_mesh = mesh.Mesh.from_file(stl_path)
            
            # Get bounding box
            min_x, max_x = stl_mesh.x.min(), stl_mesh.x.max()
            min_y, max_y = stl_mesh.y.min(), stl_mesh.y.max()
            min_z, max_z = stl_mesh.z.min(), stl_mesh.z.max()
            
            width = max_x - min_x
            depth = max_y - min_y
            height = max_z - min_z
            
            # For a wheel, radius is typically the larger of width/2 or depth/2
            # Assuming wheel is oriented with rotation axis along Z
            radius = max(width, depth) / 2.0
            
            print(f"STL bounding box:")
            print(f"  Width (X): {width:.4f} m")
            print(f"  Depth (Y): {depth:.4f} m")
            print(f"  Height (Z): {height:.4f} m")
            print(f"\nEstimated wheel radius: {radius:.4f} m")
            print(f"Estimated wheel diameter: {radius * 2:.4f} m")
            
            return radius
            
        except ImportError:
            print("numpy-stl not available. Install with: pip install numpy-stl")
            print("Using fallback estimation...")
            
            # Fallback: read STL as text and find vertex coordinates
            with open(stl_path, 'rb') as f:
                content = f.read()
                
            # STL binary format: 80 byte header, then triangles
            # For text STL, we'd parse differently
            # This is a simplified approach
            print("Note: Binary STL detected. For accurate measurement, install numpy-stl:")
            print("  pip install numpy-stl")
            print("\nUsing default estimate: 0.1 m (10 cm radius)")
            return 0.1
            
    except Exception as e:
        print(f"Error reading STL: {e}")
        print("Using default estimate: 0.1 m (10 cm radius)")
        return 0.1


if __name__ == "__main__":
    if len(sys.argv) < 2:
        # Default to left wheel
        ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        stl_path = os.path.join(
            ws_root, 
            'src', 
            'robot_description', 
            'meshes', 
            'left_wheel_link.STL'
        )
        print(f"No STL path provided, using default: {stl_path}")
    else:
        stl_path = sys.argv[1]
    
    if not os.path.exists(stl_path):
        print(f"Error: STL file not found: {stl_path}")
        print("\nUsage: python3 measure_wheel_radius.py [path_to_wheel.STL]")
        sys.exit(1)
    
    radius = estimate_wheel_radius_from_stl(stl_path)
    print(f"\nRecommended URDF wheel_radius value: {radius:.4f}")

