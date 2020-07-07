function f = plot_frame(Rt_mat, color)
    f = quiver3(repmat(Rt_mat(1, 4), 1, 3), repmat(Rt_mat(2, 4), 1, 3), repmat(Rt_mat(3, 4), 1, 3), [Rt_mat(1, 1:3)], [Rt_mat(2, 1:3)], [Rt_mat(3, 1:3)],'linewidth',2, 'color', color);
end