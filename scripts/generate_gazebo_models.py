#!/usr/bin/env python3
"""
Generate Gazebo SDF model files for ArUco markers.
Creates ceiling-mounted ArUco tag models for fiducial infrastructure simulation.
"""

import os
import argparse
from pathlib import Path

def create_model_config(marker_id, tag_size=0.14):
    """Create model.config file for an ArUco marker model."""
    return f"""<?xml version="1.0"?>
<model>
  <name>aruco-{marker_id}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>

  <author>
    <name>ArUco Models Package</name>
    <email>your.email@example.com</email>
  </author>

  <description>
    ArUco marker {marker_id} for fiducial navigation and SLAM.
    Tag size: {tag_size}m x {tag_size}m
  </description>
</model>"""

def create_model_sdf(marker_id, tag_size=0.14, thickness=0.001):
    """Create SDF file for an ArUco marker model."""
    return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="aruco-{marker_id}">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      
      <collision name="collision">
        <geometry>
          <box>
            <size>{tag_size} {tag_size} {thickness}</size>
          </box>
        </geometry>
      </collision>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>{tag_size} {tag_size} {thickness}</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>0 0 0 1</emissive>
          <pbr>
            <metal>
              <albedo_map>model://aruco-{marker_id}/materials/textures/aruco_{marker_id:03d}.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-6</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1e-6</iyy>
          <iyz>0</iyz>
          <izz>1e-6</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>"""


def create_aruco_model(marker_id, models_dir, textures_dir, tag_size=0.14):
    """Create a complete ArUco marker model."""
    model_name = f"aruco-{marker_id}"
    model_dir = os.path.join(models_dir, model_name)
    
    # Create model directory structure
    os.makedirs(model_dir, exist_ok=True)
    os.makedirs(os.path.join(model_dir, "materials", "textures"), exist_ok=True)
    
    # Create model.config
    config_path = os.path.join(model_dir, "model.config")
    with open(config_path, "w") as f:
        f.write(create_model_config(marker_id, tag_size))
    
    # Create model.sdf
    sdf_path = os.path.join(model_dir, "model.sdf")
    with open(sdf_path, "w") as f:
        f.write(create_model_sdf(marker_id, tag_size))
    
    # Copy texture file
    texture_src = os.path.join(textures_dir, f"aruco_{marker_id:03d}.png")
    texture_dst = os.path.join(model_dir, "materials", "textures", f"aruco_{marker_id:03d}.png")
    
    if os.path.exists(texture_src):
        import shutil
        shutil.copy2(texture_src, texture_dst)
        print(f"Created model: {model_name}")
        return True
    else:
        print(f"Warning: Texture file not found: {texture_src}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Generate Gazebo ArUco marker models')
    parser.add_argument('--models-dir', default='models', 
                       help='Output directory for model files')
    parser.add_argument('--textures-dir', default='textures',
                       help='Directory containing marker texture images')
    parser.add_argument('--tag-size', type=float, default=0.14,
                       help='Physical size of ArUco tags in meters (default: 0.14)')
    parser.add_argument('--ids', nargs='+', type=int,
                       help='Specific marker IDs to generate models for')
    parser.add_argument('--all', action='store_true',
                       help='Generate models for all available textures')
    
    args = parser.parse_args()
    
    if args.all:
        # Find all texture files
        texture_files = list(Path(args.textures_dir).glob("aruco_*.png"))
        marker_ids = []
        for texture_file in texture_files:
            try:
                # Extract ID from filename like "aruco_000.png"
                id_str = texture_file.stem.split('_')[1]
                marker_ids.append(int(id_str))
            except (IndexError, ValueError):
                print(f"Warning: Could not parse marker ID from {texture_file}")
        marker_ids.sort()
    elif args.ids:
        marker_ids = args.ids
    else:
        # Default to first 5 markers
        marker_ids = [0, 1, 2, 3, 4]
    
    print(f"Generating Gazebo models for ArUco markers: {marker_ids}")
    print(f"Models directory: {args.models_dir}")
    print(f"Textures directory: {args.textures_dir}")
    print(f"Tag size: {args.tag_size}m")
    
    os.makedirs(args.models_dir, exist_ok=True)
    
    success_count = 0
    for marker_id in marker_ids:
        if create_aruco_model(marker_id, args.models_dir, args.textures_dir, args.tag_size):
            success_count += 1
    
    print(f"Successfully created {success_count} models out of {len(marker_ids)} requested.")

if __name__ == '__main__':
    main()
