syms x1 y1 z1 x2 y2 z2 real;
syms q1 q2 q3 q4 p1 p2 p3 p4 real;
state_0 = [x1, y1, z1, q1, q2, q3, q4]';
state_1 = [x2, y2, z2, p1, p2, p3, p4]';
tf = get_tf_between_states(state_0, state_1);
j1 = jacobian(tf, [x1, y1, z1, q1, q2, q3, q4]);
j2 = jacobian(tf, [x2, y2, z2, p1, p2, p3, p4]);
display("Transform function");
display(tf);
display("Jacobian of Transform wrt. State Space X0");
display(j1);
display("Jacobian of Transform wrt. State Space X1");
display(j2);

syms x y z qx qy qz qw;
syms lx ly lz;
lobs = get_landmark_obs_from_observer_and_landmark_state([lx; ly; lz], [x; y;z; qx; qy; qz; qw]);
j2 = jacobian(lobs, [x, y, z, qx, qy, qz, qw, lx, ly, lz]);
display("Landmark Observation Function");
display(lobs);
display("Jacobian wrt. State and Landmark");
display(j2);

syms x1 y1 x2 y2 lx1 ly1 real; %variables
syms theta1 theta2 odomx1 odomy1 dist1 real; %factors
state_1 = [x1, y1, lx1, ly1];
state_2 = [x2, y2, lx1, ly1];
virtual_measurements = [state_2(1:2)' - state_1(1:2)'; %odom 
        rad2deg(atan2(ly1 - y1, lx1 - x1));
        sqrt((state_1(1) - lx1)^2 + (state_1(2) - ly1)^2)];                    %heading
error_function =  [odomx1; odomy1; theta1; dist1] - virtual_measurements;
j_error = jacobian(error_function, [x1, y1, x2, y2, lx1, ly1])
