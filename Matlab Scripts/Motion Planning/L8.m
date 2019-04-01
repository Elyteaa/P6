%% Linear basic bitch kalman filter
clc
clear all

z = csvread('C:\Users\Bruger\Dropbox\Uni\6th _semester\Motion Planning and Path Planning\L_8_dist_data.csv');

autocorr(z)
% The noise seems to be white, since there is little/no self-correlation
% after lag0

%%
clc
clear all

z = csvread('C:\Users\Bruger\Dropbox\Uni\6th _semester\Motion Planning and Path Planning\L_8_dist_data.csv');

R = 1;% Measurement noise / trust in sensors 
Phi = 1; % Model of the system
Q = 1*10^-4; % Process noise
H = 1; % maps state to measurement

x_pred = zeros();
z_pred = zeros();
P_pred = zeros();

x_post(1) = 1; 
P_post(1) = 1;

for k = 2:size(z,1) 
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
%% Part 2, 3000 samples
clc
clear all
z = csvread('C:\Users\Bruger\Dropbox\Uni\6th _semester\Motion Planning and Path Planning\L_8_dist_data2.csv');

R = 1;% Measurement noise / trust in sensors 
Phi = 1; % Model of the system
Q = 1*10^-2; % Process noise
H = 1; % maps state to measurement

x_pred = zeros();
z_pred = zeros();
P_pred = zeros();

x_post(1) = 1;
P_post(1) = 1;

for k = 2:size(z,1) 
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
