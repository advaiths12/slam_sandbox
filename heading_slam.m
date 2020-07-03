%generate spoofed odometry, landmark data
odom_covariance = [.009, 0; 
                    0, .011];
measurement_covariance = [10];

time_steps = 30;
landmark_location = [3, 5];
pause on;

%waypoints
start_x = [0,1,-5, 2, 3, 10, 5];
end_x = [0,1, 5, 10, 6, 3, -10];
gt_xq = [];
gt_yq = [];

%generate splines
for waypoint = 1:(length(start_x)-1)
    interp = linspace(start_x(waypoint), start_x(waypoint + 1), time_steps);
    gt_xq = [gt_xq, interp];
    gt_yq = [gt_yq, spline(start_x(waypoint:waypoint+1), [0 end_x(waypoint:waypoint+1) 0], interp)];
end

%generate position noise
odom_noise = mvnrnd([0, 0], odom_covariance, length(gt_xq));
noisy_path = [gt_xq', gt_yq'] + odom_noise;
noisy_odom = zeros(length(gt_xq), 2);
noisy_heading = zeros(length(gt_xq), 1);
plot(gt_xq, gt_yq, ':.b', start_x, end_x, 'or');

%render and get noisy heading
axis([-20, 20, -20, 20]);
prev_state = [0, 0];
for t = 1:length(gt_xq)
    robot_pose = [noisy_path(t, 1), noisy_path(t, 2)];
    rect = rectangle('Position', [robot_pose(1) - .25, robot_pose(2) - .25, .5, .5]); 
    x_ray = [robot_pose(1), landmark_location(1)];
    y_ray = [robot_pose(2), landmark_location(2)];
    l = line(x_ray, y_ray, 'Color','green','LineStyle','--');
    heading_to_landmark = rad2deg(atan2(landmark_location(2) - gt_xq(t), landmark_location(1) - gt_yq(t)));
    noisy_heading(t) = heading_to_landmark + normrnd(0, measurement_covariance);
    noisy_odom(t, :) = robot_pose - prev_state;
    prev_state = robot_pose;
    pause(.1);
    delete(rect);
    delete(l);
end

