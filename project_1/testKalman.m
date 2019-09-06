%%Kalman Filter
clear; close all; clc;

T = 0.01;

% Motion parameters
A= [1 T; 0 1];
tao = [0;1];
Q = tao * 1.5 * tao';

% Measurement parameters
H = [1 0];
R = 2;
% Prior
x_0 = [1;3];
P_0 = 4*eye(2);

% General parameters
N = 50;

% Calculate state and measurement sequences
X = genLinStateSequence(x_0,P_0,A,Q,N);
Y = genLinMeasurementSequence(X, H, R);

% Noise cases to plot
Qi = [0.1 1.5 20 100];

for i=1:numel(Qi) 
    fig=figure('Color','white','Position',[192  538  421  303]);
    [x_k_k, P_k_k, x_k_km1, P_k_km1, v, S, K] = kalmanFilter(Y, x_0, P_0, A, tao*Qi(i)*tao', H, R);
    
    % plot position
    
    hold on, grid on;
    p3 = plot(0:N, H*X, 'b', 'LineWidth',3, 'DisplayName','true state');
    p3.Color = [p3.Color 0.2];
    p2 = plot(0:N, H*[x_0 x_k_k], 'b', 'LineWidth',1.5, 'DisplayName','state estimate');
    p1 = plot(1:N, Y, '*r', 'DisplayName','measurements');
    p4 = plot(0:N, H*[x_0 x_k_k] + 3*sqrt([P_0(1) squeeze(P_k_k(1,1,:))']), '--b', 'DisplayName','+3-sigma level');
    p5 = plot(0:N, H*[x_0 x_k_k] - 3*sqrt([P_0(1) squeeze(P_k_k(1,1,:))']), '--b', 'DisplayName','-3-sigma level');
    xlabel('k - time step');
    ylabel('position');
    title(sprintf('Kalman filter, Q=%.1f',Qi(i)))
    legend([p1 p2 p3 p4 p5],'Location','southeast');
    
    %save actual plot for the final gif
    drawnow
    hold off
    frame = getframe(fig);
    im{i} = frame2im(frame);   

end
close;

filename = 'Kalman_animated.gif'; % Specify the output file name
for i=1:numel(Qi)
    [A,map] = rgb2ind(im{i},256);
    if i == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.5);
    end
end
