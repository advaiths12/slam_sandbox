odom_state_space_dim = 7;
landmark_state_space_dim = 3;
odom_cov = diag([1E-4, 1E-4, 2E-4, .001, .001, .001, .001]);
landmark_measurement_variance = .1;
landmark_measurement_cov = sqrt(diag(repmat(landmark_measurement_variance, 1, landmark_state_space_dim)));

%define start, end waypoints
landmark_location = [0, 5, 0]';
start_X = [0, 0, 0, eul2quat([0, 0, 0])]';
end_X = [10, 10, -5, eul2quat([pi/2, pi/4, 0])]';
time_steps = 1000;                                                                                     
interp = linspace(0, 1, time_steps);
interp_states = mvnrnd(zeros(1, odom_state_space_dim), odom_cov, time_steps)' + start_X + interp.*(end_X - start_X);

%for time_steps number of poses, there should be time_steps - 1
%measurements
landmark_bearings = normrnd(0, landmark_measurement_cov(1,1), landmark_state_space_dim, time_steps) + reshape(repmat(landmark_location, time_steps, 1) - reshape(interp_states(1:3, :), landmark_state_space_dim*time_steps, 1), 3, time_steps);
landmark_bearings = landmark_bearings ./ vecnorm(landmark_bearings);


%prepare equations
b = reshape(interp_states(1:3, :), 3*time_steps, 1);
landmark_measurements = reshape(landmark_bearings, landmark_state_space_dim*time_steps, 1);
A = zeros(landmark_state_space_dim*time_steps, 3+time_steps);
A(:, 1:3) = repmat(eye(3), time_steps, 1);
for col = 4:size(A, 2)
    A(landmark_state_space_dim*(col-3)-2:landmark_state_space_dim*(col-3), col) = -landmark_measurements(3*(col-3)-2:3*(col-3), 1);
end

%solve for initial guess landmark positions
x = A\b;

figure();
hold on; 
axis equal;
plot_point(landmark_location, 'green');
plot_point(x(1:3), 'red');
for frame = 1:time_steps
    scale = x(frame+3);
    p = plot_point(interp_states(1:3, frame), 'blue');
    f = plot_frame(state_to_transform(interp_states(:,frame)), 'green');
    v = quiver3(interp_states(1, frame), interp_states(2, frame), interp_states(3, frame), landmark_bearings(1, frame), landmark_bearings(2, frame), landmark_bearings(3, frame), 'color', 'cyan');
    v2 = quiver3(interp_states(1, frame), interp_states(2, frame), interp_states(3, frame), scale*landmark_bearings(1, frame), scale*landmark_bearings(2, frame), scale*landmark_bearings(3, frame), 'color', 'red');
    pause(.01);  
    delete(v);
    delete(v2);
end
