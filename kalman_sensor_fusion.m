function [ x_estimate, P_estimate ] = kalman_sensor_fusion(x_prev, u_prev, P_prev, f, F, h, H, z, Q, R)

x_predict = f(x_prev, u_prev);
P_predict = F(x_prev, u_prev) * P_prev * F(x_prev, u_prev)' + Q;

y = z - h(x_predict);
S = H(x_predict) * P_predict * H(x_predict)' + R;
K = P_predict * H(x_predict)' / S;
x_estimate = x_predict + K * y;
P_estimate = (eye(size(H(x_predict),1)) - K * H(x_predict)) * P_predict;

end

