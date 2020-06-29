function qr = mul_quat(q, p)
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    Q_matrix = [q0, -q1, -q2, -q3; 
                q1, q0, -q3, q2; 
                q2, q3, q0, -q1; 
                q3, -q2, q1, q0];
    qr = Q_matrix*p;
end