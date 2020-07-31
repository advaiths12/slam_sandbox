% generate simulated gt and noisy data

est_state = [0, 0, 0, 0, 0, 0, 0, 0, 0]';
dt = 1;

F = [1, dt, .5*dt^2, 0, 0, 0, 0, 0, 0; 
     0, 1, dt, 0, 0, 0, 0, 0, 0; 
     0, 0, 1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 1, dt, .5*dt^2, 0, 0, 0;
     0, 0, 0, 0, 1, dt, 0, 0, 0; 
     0, 0, 0, 0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, dt, .5*dt^2;
     0, 0, 0, 0, 0, 0, 0, 1, dt; 
     0, 0, 0, 0, 0, 0, 0, 0, 1]; %process
 
H = [1, 0, 0, 0, 0, 0, 0, 0, 0; 
     0, 0, 0, 1, 0, 0, 0, 0, 0; 
     0, 0, 0, 0, 0, 0, 1, 0, 0];%measurement matrix
 
P = 10000*eye(9); %state cov
Q = .00001*eye(9); %process cov noise
R = 1000*eye(3); %meas cov

space_data = [5*sin(linspace(0, 20, 10000)); 
               5*sin(linspace(0, 20, 10000)); 
               5*sin(linspace(0, 20, 10000))];
           
measurement_noise = mvnrnd([0, 0, 0], 10*eye(3) , length(space_data))';

noisy_data = space_data + measurement_noise;
% for t = 1:length(noisy_data)
%     if mod(t, 10
% end

filtered_state = zeros(size(noisy_data));
for t = 1:length(noisy_data)
    x_p = F * est_state;
    P = F*P*F' + Q;
    
    S = H*P*H' + R;
    K = P*H'*inv(S);
    y = noisy_data(:,t) - H*x_p;
    x_p = x_p + K*y;
    P = P - K*H*P;
    est_state = x_p;
    filtered_state(:,t) = est_state([1, 4, 7]);
    
end

z = conv2(noisy_data, [.25, .5, .25])
figure();


hold on;
plot(space_data(1, :), 'blue');
plot(noisy_data(1,:), 'green');
plot(z(1,:));
plot(filtered_state(1,:), 'red');
title("X Data");

figure()
hold on;
plot(space_data(2, :), 'blue');
plot(noisy_data(2,:), 'green');
plot(z(2,:));
plot(filtered_state(2,:), 'red');
title("Y Data");

figure()
hold on;
plot(space_data(3, :), 'blue');
plot(noisy_data(3,:), 'green');
plot(z(3,:));
plot(filtered_state(3,:), 'red');
title("Z Data");

