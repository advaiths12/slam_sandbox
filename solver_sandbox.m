syms x1 y1 z1 x2 y2 z2 real;
syms q1 q2 q3 q4 p1 p2 p3 p4 complex;
state_0 = [x1, y1, z1, q1, q2, q3, q4]';
state_1 = [x2, y2, z2, p1, p2, p3, p4]';
tf = get_tf_between_states(state_0, state_1);
j1 = jacobian(tf, [x1, y1, z1, q1, q2, q3, q4, x2, y2, z2, p1, p2, p3, p4]);
display("Transform function");
display(tf);
display("Jacobian of Transform wrt. State Space");
display(j1);


syms x y z qx qy qz qw;
syms lx ly lz;
lobs = get_landmark_obs_from_observer_and_landmark_state([lx; ly; lz], [x; y;z; qx; qy; qz; qw]);
j2 = jacobian(lobs, [x, y, z, qx, qy, qz, qw, lx, ly, lz]);
display("Landmark Observation Function");
display(lobs);
display("Jacobian wrt. State and Landmark");
display(j2);

syms x1 y1 x2 y2 lx1 ly1 lx2 ly2 theta1 theta2;
state_1 = [x1, y1, theta1];
state_2 = [x2, y2, theta2];
measurements = [state_2(1:2)' - state_1(1:2)';
        atan2(ly1 - y1, lx1 - x1)];
j_state = jacobian(measurements, [x1, y1, theta1, x2, y2, theta2, lx1, ly1, lx2, ly2])
