### SLAM Sandbox
These files are needed for experimentation and visualization of functions intended for a pose-graph based SLAM system. 
Generally, they provide interfaces for quaternion conversion, frame visualization, etc. 
It is helpful in MATLAB to be able to generate Jacobians for any of these functions so they are created with the intent to be symbolically differentiable

### Heading SLAM
Environment for spoofing odometry and landmark measurements in 2D. Spline trajectories with waypoints for easy testing/observation of covariances if provided.

<img src="images/sim.gif" >
<img src="images/gt_vs_estimate.jpg" >




### Transform Sandbox
Quaternion state space and helper functions for transforms

<img src="images/transforms.png" >
