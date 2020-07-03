%generate spoofed odometry, landmark data
odom_covariance = [.05, 0; 
                    0, .05];
measurement_covariance = [10];

time_steps = 20;
landmark_location = [3, 5];
pause on;
start_x = [0,1,-5];
end_x = [0,1, 5];
gt_xq = [];
gt_yq = [];

for waypoint = 1:(length(start_x)-1)
    interp = linspace(start_x(waypoint), start_x(waypoint + 1), time_steps);
    gt_xq = [gt_xq, interp];
    gt_yq = [gt_yq, spline(start_x(waypoint:waypoint+1), [0 end_x(waypoint:waypoint+1) 0], interp)];
end

odom_noise = mvnrnd([0, 0], odom_covariance, length(gt_xq));
noisy_path = [gt_xq', gt_yq'] + odom_noise;
noisy_heading = zeros(length(gt_xq), 1);
plot(gt_xq, gt_yq, ':.g', start_x, end_x, 'or');

axis([-20, 20, -20, 20]);
for t = 1:length(gt_xq)
    robot_pose = [noisy_path(t, 1), noisy_path(t, 2)];
    rect = rectangle('Position', [robot_pose(1) - .25, robot_pose(2) - .25, .5, .5]); 
    x_ray = [robot_pose(1), landmark_location(1)];
    y_ray = [robot_pose(2), landmark_location(2)];
    l = line(x_ray, y_ray);
    heading_to_landmark = rad2deg(atan2(landmark_location(2) - robot_pose(2), landmark_location(1) - robot_pose(1)));
    noisy_heading(t) = heading_to_landmark + normrnd(0, measurement_covariance);
    pause(.1);
    delete(rect);
    delete(l);
end

