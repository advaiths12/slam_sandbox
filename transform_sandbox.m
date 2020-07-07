x0 = [0; 0; 0; eul2quat([0, 0, 0])'];
x1 = [1; 1; 0; eul2quat([0, pi/2, 0])'];
x2 = [1; 1; 0; eul2quat([0, pi/4, 0])'];
tf_x0_x1q = get_tf_between_states(x0, x1);
tf_x1_x2q = get_tf_between_states(x1, x2);
tf0 = state_to_transform(x0);
tf1 = state_to_transform(x1);
tf2 = state_to_transform(x2);
figure();
hold on;
plot_frame(tf0);
plot_frame(tf1);
plot_frame(tf2);
pt_in_tf0 = tf0*[1, 0, 0, 1]';
plot_point(pt_in_tf0);

%test quaternion transform from quaternion state
tf_x0_x1 = state_to_transform(tf_x0_x1q);
pt_in_tf1 = tf_x0_x1*pt_in_tf0;
plot_point(pt_in_tf1);

tf_x1_x2 = state_to_transform(tf_x1_x2q);
pt_in_tf1 = inv(tf1)*pt_in_tf1;
pt_in_tf2 = tf_x1_x2*pt_in_tf1;
plot_point(pt_in_tf2);


axis([-5, 5, -5, 5, -5, 5]);
xlabel("x-axis")
ylabel("y-axis")
zlabel("z-axis")

function [x] = transform_to_state(tf)
    position = tf(1:3, 4);
    orientation = rotm2eul(tf(1:3, 1:3));
    x = [position;orientation'];
end

function [tf] = get_transform_between_global_tfs(tf0, tf1)
    tf = tf1*inv(tf0);

end

