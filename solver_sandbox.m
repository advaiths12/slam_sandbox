syms x1 y1 z1 x2 y2 z2 real;
syms q1 q2 q3 q4 p1 p2 p3 p4 complex;
state_0 = [x1, y1, z1, q1, q2, q3, q4]';
state_1 = [x2, y2, z2, p1, p2, p3, p4]';
tf = get_tf_between_states(state_0, state_1);
j = jacobian(tf, [x1, y1, z1, q1, q2, q3, q4, x2, y2, z2, p1, p2, p3, p4]);
display("Transform function");
tf
display("Jacobian of Transform wrt. State Space");
j

