%generate spoofed odometry, landmark data

% x, y
odom_covariance = [10000, 0; 
                    0, 10000];
                
%heading and distance
measurement_covariance = [25, 0; 
                           0,25];

factor_information = sqrt(inv(diag([diag(odom_covariance); diag(measurement_covariance)])));

time_steps = 30;
landmark_location = [3, 5];
pause on;

%waypoints
start_x = [0,1,-5, 2, 3, 10, 5, 0];
end_x = [0,1, 5, 10, 6, 3, -10, 0];
gt_xq = [];
gt_yq = [];

%generate splines
for waypoint = 1:(length(start_x)-1)
    interp = linspace(start_x(waypoint), start_x(waypoint + 1), time_steps);
    gt_xq = [gt_xq, interp];
    gt_yq = [gt_yq, spline(start_x(waypoint:waypoint+1), [0 end_x(waypoint:waypoint+1) 0], interp)];
end

%generate noises
odom_noise = mvnrnd([0, 0], sqrt(odom_covariance) ./ 100, length(gt_xq));
measurement_noise = mvnrnd([0, 0], sqrt(measurement_covariance), length(gt_xq));

gt_path = [reshape([gt_xq', gt_yq']', 2*length(gt_xq), 1); landmark_location'];
noisy_path = [gt_xq', gt_yq'];
noisy_odom = zeros(length(gt_xq), 2) + odom_noise;
noisy_heading = zeros(length(gt_xq), 1);
noisy_distance = zeros(length(gt_xq), 1);
noisy_x = zeros(length(gt_xq), 1);
noisy_y = zeros(length(gt_xq), 1);
figure();
hold on;
plot(gt_xq, gt_yq, ':.g', start_x, end_x, 'or');

%render and get noisy heading
axis([-20, 20, -20, 20]);
prev_state = [0, 0];
for t = 1:length(gt_xq)
    robot_pose = [noisy_path(t, 1), noisy_path(t, 2)];
    rect = rectangle('Position', [robot_pose(1) - .25, robot_pose(2) - .25, .5, .5]); 
    heading_to_landmark = atan2(landmark_location(2) - robot_pose(2), landmark_location(1) - robot_pose(1));
    noisy_heading(t) = heading_to_landmark + normrnd(0, measurement_covariance(1,1));
    noisy_x(t) = landmark_location(1) - robot_pose(1);
    noisy_y(t) = landmark_location(2) - robot_pose(2);
    noisy_distance(t) = norm(robot_pose - landmark_location) + normrnd(0, measurement_covariance(2,2));
    noisy_odom(t, :) = robot_pose - prev_state;
    prev_state = robot_pose;
    x_ray_obs = [robot_pose(1), robot_pose(1)+noisy_distance(t)*cos(noisy_heading(t))];
    y_ray_obs = [robot_pose(2), robot_pose(2)+noisy_distance(t)*sin(noisy_heading(t))];
    l1 = line(x_ray_obs, y_ray_obs, 'Color','red','LineStyle','--');
    x_ray_gt = [robot_pose(1), landmark_location(1)];
    y_ray_gt = [robot_pose(2), landmark_location(2)];
    l2 = line(x_ray_gt, y_ray_gt, 'Color','green','LineStyle','--');
    %pause(.1);
    delete(rect);
    delete(l1);
    delete(l2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
measurement_dim = 4;
state_space_dim = 2;
landmark_state_dim = 2;

estimate = zeros(2*(length(gt_xq)+1) + landmark_state_dim, 1);
for t = 1:length(gt_xq)
    estimate(2*t+1:2*t+2) = estimate(2*t-1:2*t) + noisy_odom(t,:)';
end
estimate(end-1:end) = [3.5, 4.6];
es = reshape(estimate, 2, length(estimate)/2)';
es_x = es(:,1);
es_y = es(:,2);
plot(es_x, es_y, ':.r');
d_x = ones(size(estimate));
prev_error = inf;
max_iters= 10;
iter = 0;
mu = 2;
tic
while(prev_error > 1E-5 && iter < max_iters)
    jacobian = zeros(measurement_dim*length(gt_xq) + 2, 2*(length(gt_xq)+1) + landmark_state_dim);
    jacobian(1:2,1:2) = eye(2);
    observations = reshape([noisy_odom';noisy_x';noisy_y'], measurement_dim*length(gt_xq), 1);
    observations = [0; 0; observations]; %add prior
    virtual_obs = zeros(measurement_dim*length(gt_xq)+2, 1);
    system_information = [[1/0.0001, 0; 
                          0, 1/0.0001], zeros(2, 4*length(gt_xq));
                          zeros(4*length(gt_xq), 2), diag(repmat(diag(factor_information), length(gt_xq), 1))];

    %populate jacobian
    for ts = 1:length(gt_xq)
        %insert motion jacobian second constraint
        jacobian(4*ts-1:4*ts, 2*ts-1:2*ts) = -eye(2);
        jacobian(4*ts-1:4*ts, 2*ts+1:2*ts+2) = eye(2);

        lx = estimate(end-1);
        ly = estimate(end);
        x_ts = estimate(2*ts-1);
        y_ts = estimate(2*ts);
        x_ts_next = estimate(2*ts+1);
        y_ts_next = estimate(2*ts+2);
        virtual_obs(4*ts-1:4*ts+2) = [x_ts_next - x_ts + noisy_odom(t, 1);
                                    y_ts_next - y_ts + noisy_odom(t, 2);
                                    lx - x_ts + measurement_noise(t, 1);
                                    ly - y_ts + measurement_noise(t, 2)];
        
        %insert landmark jacobian in robot pose node
        jacobian(4*ts+1, 2*ts-1:2*ts) = [-1, 0];
        jacobian(4*ts+1, end-1:end) = [1, 0];
        
        %insert landmark jacobian in landmark node
        jacobian(4*ts+2, 2*ts-1:2*ts) = [0, 1];
        jacobian(4*ts+2, end-1:end) = [0, -1];
    end

    H = (jacobian'*system_information)';
    H(1:2, 1:2) = eye(2);
    error_vector = (observations - virtual_obs);
    for deg = 5:4:length(error_vector)
        error_vector(deg) = wrapToPi(error_vector(deg));
    end
    error_vector = (error_vector'*system_information)';
    
    err_score = norm(error_vector);
    
    A = sparse(H'*H + eye(size(H',1))*mu);
    H(isinf(H)|isnan(H)) = 0;
    [R, flag, p] = chol(A);

    %solve R'y = A'b forward substitution
    %solve Rx = y backward substitution
    y = forwardSubstitution(R', p'*H'*error_vector, size(H', 1));
    d_x = p*backSubstitution(R, y, size(y, 1));
     
    
%     [C,R,P] = qr(H, error_vector, 0);
%     d_x(P,:) = back_sub(R, C);


    if(prev_error > err_score)
        estimate = estimate + d_x;
        mu = mu * 2;
    else
        mu = mu / 2;
    end
    prev_error = err_score;
    iter = iter + 1;
end
toc
res  = reshape(virtual_obs(3:end), 4, length(gt_xq))
nois_path = cumsum(res(1:2, :));
fprintf('RMSE: %f', sqrt(sum((estimate(3:end) - gt_path).^2)/length(gt_path)));
figure();
hold on; 
title("Comparison between Ground Truth and Estimate");
est_r = reshape(estimate(3:end), 2, length(estimate(3:end))/2);
est_pose = est_r(:,1:end-1);
est_landmark = est_r(:,end);
plot(est_pose(1,:), est_pose(2,:), ':.r'); 
plot(nois_path(1,:), nois_path(2,:), ':.b');
plot(est_landmark(1), est_landmark(2), 'or');
plot(gt_xq, gt_yq, ':.g'); 
plot(landmark_location(1), landmark_location(2), 'og');