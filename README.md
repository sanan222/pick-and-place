Pick and Place Simulation for Franka Manipulator

This project involves the application of pick and place logic for Franka manipulators in three different test scenarios. The scenarios and developed algorithms are shown below. Notably, the scenario generator is written as a Python script, while all the manipulator codes are written in C++ using MoveIt library.

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

### Task 2: Basket Color Detection

1. Set up MoveIt! with conservative settings and define a fixed orientation for the initial movement.
2. Collect and prepare basket location data by converting from the camera frame to the world frame.
3. Move the robot to the basket locations one by one while maintaining a height margin.
4. Scan the point clouds in the area and perform color filtering, including removal of the green background, clustering, and color detection by converting RGB values to HSV.
5. For movement-based error handling, after scanning each subarea, move the robot back to its predefined home position before continuing to check the remaining subareas.
6. Return a list of color names in the order of the subarea locations visited by the robot.

### Task 3: Enhanced Area Scanning & Pick-and-Place

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

### Task 1 Issues
- Only planning issues may occur. If the robot fails to move to the desired location (although it generally moves without any problems), restart the code and try again.

### Task 2 Issues
- Similar planning issues as mentioned above.
- On stable machines (e.g., dual-boot or virtual machines), Task 2 works almost perfectly. On PCs with an external SSD, lower FPS may affect performance, sometimes resulting in incomplete point cloud data on the first attempt.

### Task 3 Issues
- Using three scanning coordinates may sometimes cause the path planning to fail after several attempts, which stops the code. Restarting the code usually resolves this issue.
