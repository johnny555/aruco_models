# ArUco Models Package

This package provides Gazebo models for ArUco markers designed for ceiling-mounted fiducial infrastructure simulation.

## Features

- **ArUco Marker Generation**: Script to generate ArUco markers with proper patterns
- **Gazebo Models**: Ready-to-use Gazebo models with textures
- **Test Worlds**: Example worlds with ceiling-mounted markers and proper lighting
- **Launch Files**: Easy-to-use launch configurations

## Generated Markers

The package includes ArUco markers 0-4 from the DICT_4X4_50 dictionary:
- Size: 0.14m x 0.14m (configurable)
- Thickness: 0.001m (very thin for ceiling mounting)
- Format: PNG textures at 512x512 resolution

## Usage

### 1. Generate Additional Markers

To generate more ArUco markers:

```bash
cd /workspace/src/aruco_models
python3 scripts/generate_aruco_markers.py --ids 5 6 7 8 9 --output textures
python3 scripts/generate_gazebo_models.py --ids 5 6 7 8 9
```

### 2. Launch Test World

To test the ArUco markers in Gazebo:

```bash
cd /workspace
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspace/install/share/aruco_models
gazebo /workspace/install/share/aruco_models/aruco_test_improved.world
```

Or using the launch file:

```bash
ros2 launch aruco_models aruco_test_world.launch.py world_file:=aruco_test_improved.world
```

### 3. Integration with ArUco Detection

The models are compatible with the existing `aruco_detect` and `aruco_gazebo` packages:

- **aruco_detect**: For real camera-based detection
- **aruco_gazebo**: For simulation-based detection using model poses

Model names follow the pattern `aruco-{id}` which is recognized by `aruco_gazebo`.

### 4. Customization

#### Tag Size
To change the physical size of the markers:

```bash
python3 scripts/generate_gazebo_models.py --tag-size 0.2 --ids 0 1 2 3 4
```

#### Positioning
Markers are positioned at ceiling height (Z=2.99m) with proper orientation:
- Pose format: `X Y Z Roll Pitch Yaw`
- Ceiling mounting: `Roll=3.14159` (180°) to face downward

#### Lighting
The improved world includes:
- **Directional light**: Primary upward illumination
- **Point lights**: Additional coverage for better visibility
- **No shadows**: Disabled for cleaner marker detection

## File Structure

```
aruco_models/
├── models/           # Generated Gazebo models
│   ├── aruco-0/
│   ├── aruco-1/
│   └── ...
├── textures/         # ArUco marker images
├── worlds/           # Test world files
├── launch/           # Launch configurations
└── scripts/          # Generation utilities
```

## Integration with Existing Systems

### With Krytn Robot
```bash
ros2 launch aruco_models aruco_robot_test.launch.py robot_name:=krytn
```

### With ArUco Detection
```bash
# Terminal 1: Launch world
gazebo aruco_test_improved.world

# Terminal 2: Run ArUco detection
ros2 run aruco_detect aruco_detect --ros-args -p fiducial_len:=0.14 -p dictionary:=0
```

### With Navigation
The ceiling-mounted markers provide excellent reference points for:
- SLAM initialization
- Localization
- Navigation waypoint definition

## Troubleshooting

### Textures Not Showing
- Ensure `GAZEBO_MODEL_PATH` includes the models directory
- Verify texture files exist in `model/materials/textures/`
- Check that PNG files are valid

### Models Not Found
- Rebuild the package: `colcon build --packages-select aruco_models`
- Source the workspace: `source install/setup.bash`
- Update model path: `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspace/install/share/aruco_models`

### Poor Lighting
- Use the improved world file with enhanced lighting
- Adjust light intensity in the world file
- Disable shadows for better marker visibility

## Advanced Features

### Batch Generation
Generate multiple markers at once:

```bash
python3 scripts/generate_aruco_markers.py --all
python3 scripts/generate_gazebo_models.py --all
```

### Custom Patterns
Extend the `ARUCO_4X4_50_PATTERNS` dictionary in `generate_aruco_markers.py` to add more marker patterns.

### Different Dictionaries
Modify the pattern generation to support other ArUco dictionaries (5x5, 6x6, etc.).
