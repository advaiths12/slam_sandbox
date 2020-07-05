%generate spoofed odometry, landmark data
odom_covariance = [.009, 0; 
                    0, .011];
measurement_covariance = [.0745, 0; 
                           0, .11];

factor_information = inv(diag([diag(odom_covariance); diag(measurement_covariance)]));

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
noisy_distance = zeros(length(gt_xq), 1);
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
    heading_to_landmark = atan2(landmark_location(2) - gt_xq(t), landmark_location(1) - gt_yq(t));
    noisy_heading(t) = heading_to_landmark + normrnd(0, measurement_covariance(1,1));
    noisy_distance(t) = norm(robot_pose - landmark_location) + normrnd(0, measurement_covariance(2,2));
    noisy_odom(t, :) = robot_pose - prev_state;
    prev_state = robot_pose;
    %pause(.1);
    delete(rect);
    delete(l);
end

measurement_dim = 4;
state_space_dim = 2;
landmark_state_dim = 2;

estimate = zeros(2*(length(gt_xq)+1) + landmark_state_dim, 1);
for t = 1:length(gt_xq)
    estimate(2*t+1:2*t+2) = estimate(2*t-1:2*t) + noisy_odom(t,:)';
    
end
estimate(end-1:end) = [3.5, 4.6];
d_x = ones(size(estimate));
while(norm(d_x) > 10)
    jacobian = zeros(measurement_dim*length(gt_xq) + 2, 2*(length(gt_xq)+1) + landmark_state_dim);
    jacobian(1:2,1:2) = eye(2);
    observations = reshape([noisy_odom';noisy_heading';noisy_distance'], measurement_dim*length(gt_xq), 1);
    observations = [0; 0; observations]; %add prior
    virtual_obs = zeros(measurement_dim*length(gt_xq)+2, 1);
    system_information = [[1/0.0001, 0; 
                          0, 1/0.0001], zeros(2, 4*length(gt_xq));
                          zeros(4*length(gt_xq), 2), diag(repmat(diag(factor_information), length(gt_xq), 1))];

    %populate jacobian
    for ts = 1:length(gt_xq)
        %insert motion jacobian second constraint
        jacobian(4*ts-1:4*ts, 2*ts-1:2*ts) = eye(2);
        jacobian(4*ts-1:4*ts, 2*ts+1:2*ts+2) = -eye(2);

        lx = estimate(end-1);
        ly = estimate(end);
        x_ts = estimate(2*ts+1);
        y_ts = estimate(2*ts+2);
        x_ts_next = estimate(2*ts+3);
        y_ts_next = estimate(2*ts+4);
        virtual_obs(4*ts-1:4*ts+2) = [x_ts_next - x_ts;
                                    y_ts_next - y_ts;
                                    atan2(ly - y_ts, lx - x_ts);
                                    sqrt((ly - y_ts)^2 + (lx - x_ts)^2)];
        %insert landmark jacobian
        jacobian(4*ts+1, 2*ts-1:2*ts) = [-(ly - y_ts)/((lx - x_ts)^2 + (ly - y_ts)^2), (lx - x_ts)/((lx - x_ts)^2 + (ly - y_ts)^2)];
        jacobian(4*ts+1, end-1:end) = [(ly - y_ts)/((lx - x_ts)^2 + (ly - y_ts)^2), -(lx - x_ts)/((lx - x_ts)^2 + (ly - y_ts)^2)];
        jacobian(4*ts+2, 2*ts-1:2*ts) = [(2*lx - 2*x_ts)/(2*((lx - x_ts)^2 + (ly - y_ts)^2)^(1/2)), (2*ly - 2*y_ts)/(2*((lx - x_ts)^2 + (ly - y_ts)^2)^(1/2))];
        jacobian(4*ts+2, end-1:end) = [-(2*lx - 2*x_ts)/(2*((lx - x_ts)^2 + (ly - y_ts)^2)^(1/2)), -(2*ly - 2*y_ts)/(2*((lx - x_ts)^2 + (ly - y_ts)^2)^(1/2))];
    end

    virtual_obs = [virtual_obs]; %add prior
    H = jacobian'*jacobian;
    H(1:2, 1:2) = eye(2);
    error_vector = -((observations - virtual_obs)'*jacobian)'; 
    [R,p] = chol(H'*H);

    %solve R'y = A'b forward substitution
    %solve Rx = y backward substitution
    y = forwardSubstitution(R', H'*error_vector, size(H', 1));
    d_x = backSubstitution(R, y, size(y, 1));
    estimate = estimate + d_x;
end



