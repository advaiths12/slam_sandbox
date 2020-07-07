function [tf] = get_tf_between_states(x0, x1) %prediction function
    trans = x1(1:3) - x0(1:3);
    q0 = x0(4:7)';
    q1 = x1(4:7)';
    rot =  mul_quat(q1, conj_quat(q0));
    tf = [trans;rot];
end

