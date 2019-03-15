% Linear basic bitch kalman filter

clc
clear

z = csvread('C:\Users\Bruger\Dropbox\Uni\6th _semester\Motion Planning and Path Planning\L_8_dist_data.csv');

autocorr(z)
% The noise seems to be white



R = 100;% Measurement noise / trust in sensors 
Phi = 1; % Model of the system
Q = 1*10^-4; % Process noise
H = 1; % maps state to measurement

x_pred = zeros(300,1);
z_pred = zeros(300,1);
P_pred = zeros();

x_post(1) = 1;
P_post(1) = 1;

for k = 2:300 
    x_pred(k)= Phi*x_post(k-1); %Make new state prediction, based on model and prev state
    z_pred(k)= x_pred(k)*H;     %Predict measurement based on state prediction
    P_pred(k)= Phi*P_post(k-1)*Phi'+Q; %
    
    
    K(k) = P_pred(k)*H*inv(H*P_pred(k)*H'+R);
    x_post(k) = x_pred(k)+K(k)*(z(k)-z_pred(k));
    P_post(k) = (1 - K(k)*H)*P_pred(k);
    
end 

plot(x_post)
hold on
plot(z)
hold off
