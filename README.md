# Pick and Place for Panda Robot Arms

Pick and place is a fundamental robotic manipulation task where a robot arm identifies, grasps, and relocates objects from one position to another. As the task remains a key area of research—due to challenges in scene understanding, grasp reliability, and motion planning—this project aims to explore the core principles of robotic picking and placing.

In this work, we implement solutions using the **C++-based MoveIt library** within a simulated environment for the Panda robot arm. The Panda’s 7-DoF structure and ROS compatibility make it ideal for testing motion planning and manipulation strategies.

The repository presents solutions for **two independent but thematically related projects**, each focusing on different levels of perception, control, and environment complexity:
- **Project A** introduces basic object picking and colour-based placement.
- **Project B** extends capabilities to shape recognition, arbitrary orientations, size variation, and obstacle-aware manipulation.

---

## Repository Layout
```
comp0250_s25_labs/
├── src/
│   ├── cw1_team_09/     # Project A codes
│   └── cw2_team_09/     # Project B codes
└── ...
```

---

## Environment Setup
1. Clone the repository
2. Build the workspace and source it  
   ```bash
   catkin build
   source devel/setup.bash
   ```

3. Install OpenCV following the official [documentation](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)

---

## Project A — **Cubic Manipulation and Colour Sorting**  

| Task | Goal | How to Start | What Your Node Must Do |
|------|------|--------------|------------------------|
| **1 – Basic Pick & Place** | Grasp a single 40 mm cube and drop it into a 10 cm basket without collisions. | `rosservice call /task 1` | Read the object and basket poses from the service request, plan with MoveIt!, perform the pick-and-place, respond with an empty service reply. |
| **2 – Basket Colour Identification** | Report the colour (red, blue, purple) or absence of a basket at each provided location. | `rosservice call /task 2` | For every candidate point, capture the point cloud on `/r200/...`, segment objects, classify HSV colour, return an ordered string array such as `["blue","none","purple"]`. |
| **3 – Colour-Matched Sorting** | Place each coloured cube into the matching coloured basket. | `rosservice call /task 3` | Detect all cubes and baskets, pair by colour, execute repeated pick-and-place operations until no sortable items remain. |

---

## Project 2 — **Shape Recognition and Advanced Manipulation**  
*(formerly “Coursework 2” – worth 60 % of the module mark)*

| Task | Goal | How to Start | What Your Node Must Do |
|------|------|--------------|------------------------|
| **1 – Orientation-Aware Pick & Place** | Pick up either a “nought” or “cross” (size 40 mm) at an arbitrary yaw and place it in a large brown basket. | `rosservice call /task 1` | Compute object orientation from its point cloud, align the gripper, grasp, lift and release inside the basket; reply with an empty response. |
| **2 – Mystery Shape Matching** | Decide which of two reference shapes matches a third “mystery” shape. | `rosservice call /task 2` | For each of the three centroids supplied, capture the surrounding cloud, classify the shape via centre-density metric, return `1` or `2` indicating the matching reference. |
| **3 – Scene Survey, Counting and Obstacle Avoidance** | Count all shapes (sizes 20–40 mm), determine which type is most common, then place one instance of that type into the basket while avoiding black obstacles. | `rosservice call /task 3` | Perform a panoramic scan, merge and filter the cloud, segment clusters, classify each, compute totals, publish the counts in the service response, pick the chosen shape, raise it above the tallest obstacle and drop it into the basket. |

> **Assessment variables**  
> The marking scripts may vary object orientations, ground noise, number of obstacles, and whether multiple sizes appear. Code should handle every documented variation.

---

## Running a Project

### Launch
```bash
# Project 1
roslaunch cw1_team_09 run_solution.launch

# Project 2
roslaunch cw2_team_09 run_solution.launch
```

### Trigger specific tasks  
Replace `N` with **1**, **2** or **3**:  
```bash
rosservice call /task N
```

---

## Development Notes
* Use only the service request data or the RGB-D topics; direct model inspection is forbidden.  
* All custom nodes must be launched via `run_solution.launch` in each package.  
* Additional dependencies beyond standard ROS, MoveIt!, OpenCV and PCL must be declared in `package.xml` and `CMakeLists.txt`.

---

## Contributors
| Member | Focus Areas | Approx. Hours |
|--------|-------------|---------------|
| Sanan Garayev | Planning, perception, documentation | 80 h |
| Ceren Doner   | Vision pipelines, testing           | 80 h |
| Enis Yalcin   | Grasp control, PCL algorithms       | 80 h |

---

## Licence
Distributed under the MIT Licence – see `LICENSE` for full text.



















# Pick and Place Simulation for Panda Robot Arm

This project involves the application of pick and place logic for Panda manipulators in three different test scenarios. The scenarios and developed algorithms are shown below. Notably, the scenario generator is written as a Python script, while all the manipulator codes are written in C++ using MoveIt library.

## Running the Code

To execute the code, follow these steps:

### Terminal 1

```bash
catkin clean
catkin build
source devel/setup.bash
roslaunch cw1_team_09 run_solution.launch
```

### Terminal 2
Change the task number as needed (e.g., task 1, task 2, task 3):
```bash
rosservice call /task 1
```

## Algorithmic Structure

### Scenario 1: Pick-and-Place Strategy
This scenario aims to apply a pick-and-place algorithm when only one object and one basket with known shapes are given. The shape information is read listening to the world generator.

1. Configure MoveIt! and set a fixed end-effector orientation.
2. Open the gripper and retrieve the object pose with a height offset.
3. Lower the gripper to a pickup pose (12 cm above the object) and close the gripper.
4. Define the basket pose and approach it with an upward offset of 30 cm.
5. Move to the basket and release the object by opening the gripper.

### Scenario 2: Basket Color Detection
This scenario aims to use PIL library functions to detect the color of the baskets and return a list of strings.

1. Set up MoveIt! with conservative settings and define a fixed orientation for the initial movement.
2. Collect and prepare basket location data by converting from the camera frame to the world frame.
3. Move the robot to the basket locations one by one while maintaining a height margin.
4. Scan the point clouds in the area and perform color filtering, including removal of the green background, clustering, and color detection by converting RGB values to HSV.
5. For movement-based error handling, after scanning each subarea, move the robot back to its predefined home position before continuing to check the remaining subareas.
6. Return a list of color names in the order of the subarea locations visited by the robot.

### Scenario 3: Enhanced Area Scanning & Pick-and-Place
This scenario aims to pick the objects and put them into the baskets with same color. The main limitation is that the exact locations of the objects and baskets are unknown. 

1. Clear the transform buffer, retrieve the robot's home pose, and set the planning parameters.
2. Define a central scanning position, as well as positions 20 cm to the left and 20 cm to the right, to cover the entire area.
3. Move to the central position and scan the area using the same color filtering and clustering methods as in Task 2.
4. Retrieve the locations of the clusters using bounding boxes to find their central points.
5. Repeat the process for the left and right positions.
6. Filter baskets and boxes using a threshold: if a cluster has 1,000–2,000 points, label it as a box; if it has more than 3,000 points, label it as a basket. If multiple boxes have the same color, choose the one with the most points.
7. Define approach margins for both baskets and boxes, and verify color matches. If there is no color match between a box and a basket, ignore them, as mentioned in the Task 2 description.
8. Perform the pick-and-place algorithm developed for Task 1, returning to the predefined home pose after both picking and placing.
9. Return to the home position after completing all operations.

## Known Issues & Troubleshooting

Due to inverse kinematics and the RRT Connect algorithm in the MoveIt! library, timing and planning errors may occur. However, the Panda robot arm typically finds a valid plan on subsequent trials.

### Scenario 1 Issues
- Only planning issues may occur. If the robot fails to move to the desired location (although it generally moves without any problems), restart the code and try again.

### Scenario 2 Issues
- Similar planning issues as mentioned above.
- On stable machines (e.g., dual-boot or virtual machines), Task 2 works almost perfectly. On PCs with an external SSD, lower FPS may affect performance, sometimes resulting in incomplete point cloud data on the first attempt.

### Scenario 3 Issues
- Using three scanning coordinates may sometimes cause the path planning to fail after several attempts, which stops the code. Restarting the code usually resolves this issue.
