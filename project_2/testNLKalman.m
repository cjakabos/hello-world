%%Non-linear Kalman Filter
clear; close all; clc;
 
% True track
% Sampling period
T = 0.1;

% Length of time sequence
K = 600;

% Allocate memory
omega = zeros(1,K+1);

% Turn rate
omega(200:400) = -pi/205/T;

% Initial state
x0 = [0 0 20 0 omega(1)]';

% Allocate memory
X = zeros(length(x0),K+1);
X(:,1) = x0;

% Create true track
for i=2:K+1
    % Simulate
    X(:,i) = coordinatedTurnMotion(X(:,i-1), T);
    % Set turnrate
    X(5,i) = omega(i);
end

% Prior information
x_0 = [0 0 0 0 0]';
P_0 = diag([10 10 10 5*pi/180 pi/180].^2);
% Sensor positions
s1 = [380 -80]';
s2 = [280 -200]';

% measurement noise
R = diag([4*pi/180 4*pi/180].^2);
% generate measurement sequence
h = @(x) dualBearingMeasurement(x,s1,s2);
Y = genNonLinMeasurementSequence(X,h,R);

% Motion model
f = @(x) coordinatedTurnMotion(x,T);

% Process noise cases to plot
Qi = [1*1e-4 20*1e-4 0.1*1e-4;pi/180 pi/9 pi/1800];

for i=1:size(Qi,2)
fig=figure('Color','white','Position',[520  180  654  417]);
Q = diag([0 0 T*Qi(1,i)^2 0 T*Qi(2,i)^2]);   
[xf, Pf, xp, Pp] = nonLinKalmanFilter(Y, x_0, P_0, f, Q, h, R, 'CKF');

% calcualte unfiltered position from sensors given angles
Xm(1,:) = ( s2(2)-s1(2) + tan(Y(1,:))*s1(1) - tan(Y(2,:))*s2(1) ) ./ ( tan(Y(1,:)) - tan(Y(2,:)) );
Xm(2,:) = s1(2) + tan(Y(1,:)) .* ( Xm(1,:) - s1(1) );

grid on; hold on, axis equal;

p1 = plot(X(1,:),X(2,:), 'Color', 'blue', 'LineWidth',2, 'DisplayName','True position sequence');
p2 = plot(xf(1,:),xf(2,:), 'Color', 'red', 'LineWidth',2, 'DisplayName','Sensor position');
sc1 = scatter(s1(1), s1(2), 130, 'p', 'MarkerFaceAlpha',0.8, 'MarkerFaceColor', [0 0 0], 'MarkerEdgeColor', [0 0 0],'DisplayName','sensor 1 location');
sc2 = scatter(s2(1), s2(2), 130, 's', 'MarkerFaceAlpha',0.8, 'MarkerFaceColor', [0 0 0], 'MarkerEdgeColor', [0 0 0],'DisplayName','sensor 2 location');

xlabel 'pos x', ylabel 'pos y'
xlim([-50 600]) , ylim([-300 50])

if i==1
    title(sprintf('Kalman filter, noise: optimal'))
elseif i==2
    title(sprintf('Kalman filter, noise:     high'))
else
    title(sprintf('Kalman filter, noise:      low'))
end

legend([p1 p2 sc1 sc2], 'Location','west')


%save actual plot for the final gif
drawnow
hold off
frame = getframe(fig);
im{i} = frame2im(frame);   

end


filename = 'NL_Kalman_animated.gif'; % Specify the output file name
for i=1:size(Qi,2)
    [A,map] = rgb2ind(im{i},256);
    if i == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.5);
    end
end

