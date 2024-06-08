# Welding Path Planning and Execution with RoboDK and OCC

This project aims to perform welding path planning and execution using RoboDK and the OpenCASCADE (OCC) libraries. It involves the following key steps:
1. **Reading and Processing CAD Data**: Parsing and analyzing STEP files to identify and extract relevant geometric information.
2. **Path Planning with A star Algorithm**: Using the A* algorithm for 3D path planning to determine the shortest collision-free path.
3. **Collision Detection and Angle Adjustment**: Adjusting robot angles to avoid collisions during welding operations.
4. **Executing Welding Paths**: Using RoboDK to move the robot along the planned paths for welding.

## Requirements

- Python 3.x
- RoboDK
- OpenCASCADE (OCC) libraries (PythonOCC)
- NumPy
- Pandas
- Tkinter

## Installation

1. Install Python 3.x from [Python.org](https://www.python.org/).
2. Install RoboDK and ensure it's properly set up on your system.
3. Install the required Python libraries:
    ```bash
    pip install numpy pandas matplotlib openpyxl
    ```
4. Install PythonOCC:
    ```bash
    pip install pythonocc-core
    ```

## File Structure

- `etape1.py`: Main script for reading and processing the CAD data, setting up curves, and detecting collisions.
- `etape3.py`: Script for executing welding paths using the A* algorithm and RoboDK.
- `functions.py`: Contains helper functions for processing CAD data, detecting collisions, and calculating angles.
- `interface.py`: Tkinter-based GUI for selecting and testing curves for welding.
- `astar.py` class (in `etape1.py` and `etape3.py`): Implements the A* algorithm for 3D path planning.
- `README.md`: This file, providing an overview of the project.

## Usage

### Running the Interface

To run the interface for selecting and testing curves:
```bash
python interface.py
```

## Function Descriptions
1. etape1.py
- Reads and processes the CAD data.
- Sets up curves and detects intersections.
- Uses functions from `functions.py` to determine paths and angles for welding.

2. etape3.py
- Executes the welding paths using the A* algorithm and RoboDK.
- Uses data from processed curves and performs welding operations.

3. functions.py
- `set_curves()`: Reads and processes the CAD data to extract curves and intersections.
- `create_curves(curveList)`: Adds the detected curves to RoboDK.
- `split_solid0_into_parts(mainfaces)`: Divides a solid0 into parts and returns the points for each part.
- `interpolate_points(start, end, step_size=1.0)`: Generates interpolated points between two points.
- `points_of_each_solid(solids)`: Returns the points for each solid object.
- `points_of_each_face(mainfaces)`: Returns the points for each face.
- `find_way(curveList, curve)`: Determines the axis on which the curve is located.
- `rotation_angles(intersectSolid1, intersectSolid2, curve, curveList, grupFace, solid0Liste)`: Calculates the rotation angles for approaching the curve.
- `testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry, rz, step)`: Checks for collisions and returns the coordinates and angles that avoid collisions.

4. functions.py
- Provides a GUI for selecting and testing curves.
- Uses Tkinter for the interface and subprocess for running scripts.

5. astar.py
- `__init__(self, start, goal, obstacles, resolution=0.5)`: Initializes the A* algorithm with start and goal points, obstacles, and resolution.
- `heuristic_fun(self, node)`: Heuristic function to estimate the cost between the current node and the goal.
- `get_neighbors(self, node)`: Retrieves the neighboring nodes of the current node.
- `is_collide(self, point)`: Checks if a point collides with any of the obstacles.
- `run(self, N=None)`: Runs the A* algorithm to find the shortest path.
- `path(self)`: Constructs the path from the start point to the goal.
- `visualization(self)`: Visualizes the path and obstacles (optional).

## References
This code is adapted from the A* 3D path planning implementation by zhm-real. The original implementation can be found at:
https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_3D/Astar3D.py

