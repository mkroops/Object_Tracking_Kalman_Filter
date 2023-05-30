
function [xe, Pe] = kalmanUpdate(x, P, H, R, z)
% Update step of Kalman filter.
% x: state vector
% P: covariance matrix of x
% H: matrix of observation model
% R: matrix of observation noise
% z: observation vector
% Return estimated state vector xe and covariance Pe
x = csvread('x.csv');
y = csvread('y.csv');
a = csvread('a.csv');
b = csvread('b.csv');

S = H * P * H' + R; % innovation covariance
K = P * H' * inv(S); % Kalman gain
zp = H * x; % predicted observation

gate = (z - zp)' * inv(S) * (z - zp);
if gate > 9.21
    warning('Observation outside validation gate');
    xe = x;
    Pe = P;
    return
end

xe = x + K * (z - zp); % estimated state
Pe = P - K * S * K'; % estimated covariance
end

function [xp, Pp] = kalmanPredict(x, P, F, Q)
% Prediction step of Kalman filter.
% x: state vector
% P: covariance matrix of x
% F: matrix of motion model
% Q: matrix of motion noise
% Return predicted state vector xp and covariance Pp
xp = F * x; % predict state
Pp = F * P * F' + Q; % predict state covariance
end



function [px, py] = kalmanTracking(x, y, a, b)
% Track a target with a Kalman filter
% x, y: real coordinates of the moving target
% a, b: noisy coordinates of the moving target
% Return the estimated state position coordinates (px,py)

dt = 0.2; % time interval
N = length(a); % number of samples
F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1]; % CV motion model
Q = [0.01 0 0 0; 0 1 0 0; 0 0 0.01 0; 0 0 0 1]; % motion noise
H = [1 0 0 0; 0 0 1 0]; % Cartesian observation model
R = [4 0; 0 4]; % observation noise
x0 = [a(1) 0 b(1) 0]'; % initial state
P0 = Q; % initial state covariance
s = zeros(4,N);
for i = 1:N
    z = [a(i); b(i)]; % current observation
    [x_pred, P_pred] = kalmanPredict(x_est, P_est, F, Q); % predict next state
    [x_est, P_est] = kalmanUpdate(x_pred, P_pred, H, R, z); % update state estimate
    s_est(:, i) = x_est; % save current state estimate
end

px = s_est(1,:); % extract estimated x coordinates
py = s_est(3,:); % extract estimated y coordinates
plot(x, y, 'xb' , 'LineWidth', 2);
hold on;
plot(a, b, '+r');
plot(px, py, 'xg', 'LineWidth', 2);
legend('Actual (Blue)', 'Noisy (Red)', 'Estimated (Green)');
xlabel('x');
ylabel('y');
title('Estimated trajectory vs Actual and noisy');
abs_err_noisy = hypot(x - a, y - b);
abs_err_estimated = hypot(x - px, y - py);
rms_noisy = rms(abs_err_noisy);
rms_estimated = rms(abs_err_estimated);
mean_abs_err_noisy = mean(abs_err_noisy);
mean_abs_err_estimated = mean(abs_err_estimated);
std_abs_err_noisy = std(abs_err_noisy);
std_abs_err_estimated = std(abs_err_estimated);
% Print the error statistics
fprintf('Root Mean Squared Error (Noisy): %.4f\n', rms_noisy);
fprintf('Root Mean Squared Error (Estimated): %.4f\n', rms_estimated);
fprintf('Mean Absolute Error (Noisy): %.4f\n', mean_abs_err_noisy);
fprintf('Mean Absolute Error (Estimated): %.4f\n', mean_abs_err_estimated);
fprintf('Standard Deviation of Absolute Error (Noisy): %.4f\n', std_abs_err_noisy);
fprintf('Standard Deviation of Absolute Error (Estimated): %.4f\n',
std_abs_err_estimated); 
end