#!/usr/bin/env python3
"""
Enhanced Gazebo model generator for ArUco markers.
Supports multiple ArUco dictionaries and large-scale marker generation.
"""

import os
import sys
import argparse
from pathlib import Path

# Add current directory to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from generate_aruco_markers_advanced import ArUcoGenerator
except ImportError:
    print("Error: Could not import ArUcoGenerator from generate_aruco_markers_advanced.py")
    print("Make sure the advanced marker generator is in the same directory.")
    sys.exit(1)

def create_gazebo_model(dict_name, marker_id, model_dir, texture_filename):
    """
    Create a Gazebo model directory with SDF file for an ArUco marker.
    
    Args:
        dict_name: ArUco dictionary name
        marker_id: Marker ID
        model_dir: Directory for the model
        texture_filename: Name of the texture file
    """
    os.makedirs(model_dir, exist_ok=True)
    
    # Model name
    model_name = f"aruco_{dict_name.lower()}_{marker_id:04d}"
    
    # Create model.config
    config_content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>ArUco Models Package</name>
    <email>example@example.com</email>
  </author>
  <description>
    ArUco marker {marker_id} from {dict_name} dictionary
  </description>
</model>
"""
    
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(config_content)
    
    # Create model.sdf with PBR texture
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>0.2 0.2</size>
          </plane>
        </geometry>
        <material>
          <pbr>
            <metal>
              <albedo_map>model://{model_name}/materials/textures/{texture_filename}</albedo_map>
              <metalness>0.0</metalness>
              <roughness>0.8</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
    
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(sdf_content)
    
    print(f"Created Gazebo model: {model_name}")

def generate_gazebo_models(dict_name, marker_ids, base_output_dir, texture_dir):
    """
    Generate Gazebo models for a set of ArUco markers.
    
    Args:
        dict_name: ArUco dictionary name
        marker_ids: List of marker IDs
        base_output_dir: Base directory for models
        texture_dir: Directory containing texture files
    """
    # Create models directory
    models_dir = os.path.join(base_output_dir, "models")
    os.makedirs(models_dir, exist_ok=True)
    
    # Copy textures to each model directory
    for marker_id in marker_ids:
        model_name = f"aruco_{dict_name.lower()}_{marker_id:04d}"
        model_dir = os.path.join(models_dir, model_name)
        
        # Create materials/textures directory
        materials_dir = os.path.join(model_dir, "materials", "textures")
        os.makedirs(materials_dir, exist_ok=True)
        
        # Copy texture file
        texture_filename = f"aruco_{dict_name.lower()}_{marker_id:04d}.png"
        src_texture = os.path.join(texture_dir, texture_filename)
        dst_texture = os.path.join(materials_dir, texture_filename)
        
        if os.path.exists(src_texture):
            import shutil
            shutil.copy2(src_texture, dst_texture)
            
            # Create the Gazebo model
            create_gazebo_model(dict_name, marker_id, model_dir, texture_filename)
        else:
            print(f"Warning: Texture file not found: {src_texture}")

def create_world_file(dict_name, marker_ids, output_dir, world_name="aruco_test_comprehensive"):
    """
    Create a comprehensive Gazebo world file with multiple ArUco markers.
    
    Args:
        dict_name: ArUco dictionary name  
        marker_ids: List of marker IDs to include
        output_dir: Output directory
        world_name: Name of the world file
    """
    world_file = os.path.join(output_dir, "worlds", f"{world_name}.world")
    os.makedirs(os.path.dirname(world_file), exist_ok=True)
    
    # Start world content
    world_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="aruco_comprehensive_world">
    
    <!-- Physics -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>1000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <!-- Scene -->
    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Enhanced lighting system -->
    <light name="directional_light" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>5 5 10 0 -0.5 -0.5</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 -0.5 -1.0</direction>
    </light>

    <!-- Point lights for ceiling illumination -->
    <light name="point_light_1" type="point">
      <pose>-2 -2 2.5 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

    <light name="point_light_2" type="point">
      <pose>2 -2 2.5 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

    <light name="point_light_3" type="point">
      <pose>0 2 2.5 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

"""
    
    # Add ArUco markers in a grid pattern on the ceiling
    ceiling_height = 2.99
    grid_size = int((len(marker_ids)) ** 0.5) + 1  # Grid dimensions
    spacing = 1.0  # Spacing between markers
    
    start_x = -(grid_size - 1) * spacing / 2
    start_y = -(grid_size - 1) * spacing / 2
    
    for i, marker_id in enumerate(marker_ids):
        row = i // grid_size
        col = i % grid_size
        
        x = start_x + col * spacing
        y = start_y + row * spacing
        
        model_name = f"aruco_{dict_name.lower()}_{marker_id:04d}"
        
        world_content += f"""
    <!-- ArUco marker {marker_id} -->
    <include>
      <uri>model://{model_name}</uri>
      <name>{model_name}_instance</name>
      <pose>{x:.2f} {y:.2f} {ceiling_height:.2f} 3.14159 0 0</pose>
    </include>
"""
    
    # Add some navigation obstacles for testing
    world_content += """
    <!-- Navigation obstacles -->
    <model name="box_obstacle_1">
      <static>true</static>
      <pose>-3 -3 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="box_obstacle_2">
      <static>true</static>
      <pose>3 3 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

  </world>
</sdf>
"""
    
    with open(world_file, "w") as f:
        f.write(world_content)
    
    print(f"Created world file: {world_file}")
    return world_file

def main():
    """Main function for generating Gazebo models and worlds"""
    
    parser = argparse.ArgumentParser(
        description='Generate Gazebo models and worlds for ArUco markers',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate models for first 10 markers from DICT_4X4_50
  python generate_gazebo_models_advanced.py --dict DICT_4X4_50 --count 10
  
  # Generate models for specific marker IDs
  python generate_gazebo_models_advanced.py --dict DICT_5X5_50 --ids 0 5 10 15 20
  
  # Generate large-scale fiducial infrastructure with 100 markers
  python generate_gazebo_models_advanced.py --dict DICT_4X4_250 --range 0 99 --world
  
  # Generate comprehensive test environment
  python generate_gazebo_models_advanced.py --dict DICT_6X6_50 --count 25 --world --textures --models
        """
    )
    
    parser.add_argument('--dict', '-d', default='DICT_4X4_50',
                       help='ArUco dictionary to use (default: DICT_4X4_50)')
    parser.add_argument('--output', '-o', default='.', 
                       help='Output directory (default: current directory)')
    parser.add_argument('--ids', nargs='+', type=int,
                       help='Specific marker IDs to generate models for')
    parser.add_argument('--range', nargs=2, type=int, metavar=('START', 'END'),
                       help='Generate models for markers in range START to END')
    parser.add_argument('--count', '-c', type=int, default=5,
                       help='Generate models for first COUNT markers (default: 5)')
    parser.add_argument('--size', '-s', type=int, default=512,
                       help='Texture size in pixels (default: 512)')
    parser.add_argument('--textures', action='store_true',
                       help='Generate texture files')
    parser.add_argument('--models', action='store_true',
                       help='Generate Gazebo model files')
    parser.add_argument('--world', action='store_true',
                       help='Generate Gazebo world file')
    parser.add_argument('--all', action='store_true',
                       help='Generate textures, models, and world (same as --textures --models --world)')
    
    args = parser.parse_args()
    
    # Handle --all flag
    if args.all:
        args.textures = True
        args.models = True
        args.world = True
    
    # Default behavior if no specific flags
    if not (args.textures or args.models or args.world):
        args.textures = True
        args.models = True
        args.world = True
    
    # Initialize generator
    generator = ArUcoGenerator()
    
    # Validate dictionary
    if args.dict not in generator.get_available_dictionaries():
        print(f"Error: Dictionary '{args.dict}' not available")
        print(f"Available dictionaries: {', '.join(generator.get_available_dictionaries())}")
        return 1
    
    # Determine marker IDs
    if args.ids:
        marker_ids = args.ids
    elif args.range:
        marker_ids = list(range(args.range[0], args.range[1] + 1))
    else:
        marker_ids = list(range(args.count))
    
    # Validate marker IDs exist in dictionary
    dict_info = generator.get_dictionary_info(args.dict)
    max_available = dict_info['max_id']
    marker_ids = [mid for mid in marker_ids if mid <= max_available]
    
    if not marker_ids:
        print(f"Error: No valid marker IDs. Dictionary {args.dict} has markers 0-{max_available}")
        return 1
    
    print(f"Generating Gazebo assets for {len(marker_ids)} markers from {args.dict}")
    print(f"Marker IDs: {marker_ids[:10]}{'...' if len(marker_ids) > 10 else ''}")
    
    # Paths
    texture_dir = os.path.join(args.output, "textures")
    
    try:
        # Generate textures
        if args.textures:
            print("\n--- Generating ArUco marker textures ---")
            generator.generate_markers_batch(args.dict, texture_dir, marker_ids, args.size)
        
        # Generate Gazebo models
        if args.models:
            print("\n--- Generating Gazebo models ---")
            generate_gazebo_models(args.dict, marker_ids, args.output, texture_dir)
        
        # Generate world file
        if args.world:
            print("\n--- Generating Gazebo world ---")
            world_name = f"aruco_{args.dict.lower()}_{len(marker_ids)}_markers"
            create_world_file(args.dict, marker_ids, args.output, world_name)
        
        print(f"\n✅ Successfully generated Gazebo assets for {len(marker_ids)} ArUco markers!")
        print(f"📁 Output directory: {os.path.abspath(args.output)}")
        
        if args.world:
            print(f"\nTo test the world:")
            print(f"  gazebo {os.path.join(args.output, 'worlds', world_name + '.world')}")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    sys.exit(main() or 0)
