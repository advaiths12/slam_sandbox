% generate simulated gt and noisy data

est_state = [0, 0, 0, 0, 0, 0]';
dt = 1;

F = [1, 0, 0, 0, 0, 0; 
     0, 1, 0, 0, 0, 0; 
     0, 0, 1, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0; 
     0, 0, 0, 0, 0, 1]; %process
 
H = [1, 0, 0, 0, 0, 0; 
     0, 0, 1, 0, 0, 0; 
     0, 0, 0, 0, 1, 0];%measurement matrix
 
P = 10000*eye(6); %state cov
Q = .0001*eye(6); %process cov noise
R = 10000*eye(3); %meas cov
override = (10^10)*R;
space_data = repmat([50; 50; 50], 1, 10000);
           
measurement_noise = mvnrnd([0, 0, 0], 200*eye(3) , length(space_data))';

noisy_data = space_data + measurement_noise;
for t = 1:length(noisy_data)
    if mod(t, 1000) == 0
        noisy_data(:, t:t+500) = repmat([-1, -1, -1]', 1,  501);
        space_data(:, t:t+500) = repmat(space_data(:, t), 1, 501);
    end
end
figure();
plot(space_data(1, :));

filtered_state = zeros(size(noisy_data));
saved_vel = [0, 0, 0]
for t = 1:length(noisy_data)
    if mod(t, 1) == 0
        P
    end
    
    x_p = F * est_state;
    P = F*P*F' + Q;
    if all(noisy_data(:,t) == [-1, -1, -1]')
        S = H*P*H' + override;
    else
        S = H*P*H' + R;
    end
    K = P*H'*inv(S);
    y = noisy_data(:,t) - H*x_p;
    x_p = x_p + K*y;
    P = P - K*H*P;
    est_state = x_p;
    filtered_state(:,t) = est_state([1, 3, 5]);
   
    
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

