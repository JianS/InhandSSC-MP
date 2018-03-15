This is a simulation for in-hand sliding on a customized object w/ spring-sliding compliant fingers. In this program we are solving the motion planning using the inverse contact mechanics. The input is the initial and goal grasp, and outcome is the anchor motions. 

Run main_MPI.m to see a demo.

Initial and goal configurations are set at the beginning of 'main_MPI.m'.

There will be three output figures:

1. Shows initial configuration: Blue dots are initial fingertip locations; green cones are friction cones; blue, red and green arrows at fingertips are contact force, normal force and tangential forces respectively; black dots are goal fingertip locations.

2. Shows task space (y-pos of the two fingers): green region is the feasible region where the object won't be tipped over. Red curve is the most robust curve. Black curve is the planning result.

3. Animation showing the planned result. Press any key to start play back. 