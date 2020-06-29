x0 = [0; 0; 0; eul2quat([0, 0, 0])'];
x1 = [1; 1; 0; eul2quat([0, pi/2, 0])'];
tf = get_tf_between_states(x0, x1);
tf0 = state_to_transform(x0);
tf1 = state_to_transform(x1);
figure();
hold on;
plot_frame(tf0);
plot_frame(tf1);
pt_in_tf0 = tf0*[1, 0, 0, 1]';
plot_point(pt_in_tf0);

%test quaternion transform from quaternion state
tf_x0_x1 = state_to_transform(tf);
plot_point(tf_x0_x1*pt_in_tf0);


axis([-5, 5, -5, 5, -5, 5]);
xlabel("x-axis")
ylabel("y-axis")
zlabel("z-axis")

function [x] = transform_to_state(tf)
    position = tf(1:3, 4);
    orientation = rotm2eul(tf(1:3, 1:3));
    x = [position;orientation'];
end

function [tf] = state_to_transform(x)
    translation = x(1:3);
    rotation = eul2rotm(quat2eul(x(4:7)'));
    tf = [[rotation; 0, 0, 0], [translation; 1]];
end

function [] = plot_point(pt)
    scatter3(pt(1), pt(2), pt(3), "filled");
end

function [tf] = get_transform_between_global_tfs(tf0, tf1)
    tf = tf1*inv(tf0);

end

function [] = plot_frame(Rt_mat)
    quiver3(repmat(Rt_mat(1, 4), 1, 3), repmat(Rt_mat(2, 4), 1, 3), repmat(Rt_mat(3, 4), 1, 3), [Rt_mat(1, 1:3)], [Rt_mat(2, 1:3)], [Rt_mat(3, 1:3)],'linewidth',2);
end