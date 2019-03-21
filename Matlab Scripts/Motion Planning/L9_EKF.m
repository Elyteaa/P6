%% Extended Kalman Filter
clc
clear all

z = importdata('C:\Users\Bruger\Dropbox\Uni\6th _semester\Motion Planning and Path Planning\L_9_pendulum_data.csv');

plot(z)
%%

R = 1;% Measurement noise / trust in sensors. 
Q = 1*10^-4; % Process noise
H = [1 0]; % maps state to measurement
alpha = 0.5; %Friction
g = -9.82; %Gravity
l = 200; %Length of rod

x_pred = zeros();
Theta_pred = zeros();
P_pred = zeros();

x_post(1) = 1;
P_post(1) = 1;

%Additions in the Extended Kalman Filter
b = 0.7; %Sensor bias

for k = 2:size(z,1)
    %Theta(k,1) = Theta_dot
    %Theta(k,2) = Theta
    Theta(k) = [Theta(k-1,1)+T_s*(-(g/l)*sin(z(k-1))-alpha*Theta_d(k-1));
                Theta(k-1,2)+T_s*Theta_d(k-1,1)];
            %Confusion: are these the thetas from the measurement or the
            %ones we calculate
    %x_pred(k)= Phi*x_post(k-1); %Make new state prediction, based on model and prev state
    Theta_pred(k)= x_pred(k)*H;     %Predict measurement based on state prediction
    P_pred(k)= Phi*P_post(k-1)*Phi'+Q; %Predict 
    
    Phi = [1 T_s;
           -T_s*(g/l)*cos(z(k)) 1-T_s*alpha]; % Jacobian
     
    K(k) = P_pred(k)*H*inv(H*P_pred(k)*H'+R);
    x_post(k) = x_pred(k)+K(k)*(z(k)-Theta_pred(k));
    P_post(k) = (1 - K(k)*H)*P_pred(k);
end

plot(Theta)
hold on
plot(z)
hold off
