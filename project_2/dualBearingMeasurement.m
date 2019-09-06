function [hx, Hx] = dualBearingMeasurement(x, s1, s2)
    % DUOBEARINGMEASUREMENT calculates the bearings from two sensors, located in 
    % s1 and s2, to the position given by the state vector x. Also returns the
    % Jacobian of the model at x.
    %
    % Input:
    %   x           [n x 1] State vector, the two first element are 2D position
    %   s1          [2 x 1] Sensor position (2D) for sensor 1
    %   s2          [2 x 1] Sensor position (2D) for sensor 2
    %
    % Output:
    %   hx          [2 x 1] measurement vector
    %   Hx          [2 x n] measurement model Jacobian
    %
    % NOTE: the measurement model assumes that in the state vector x, the first
    % two states are X-position and Y-position.
   
    % initialize outputs with correct sizes
    n  = size(x,1);
    N = size(x,2);
    hx = zeros(2,N);
    Hx = zeros(2,n);

    % calculate readings from the two sensors
    ang1 = atan2( x(2,:)-s1(2), x(1,:)-s1(1) );
    ang2 = atan2( x(2,:)-s2(2), x(1,:)-s2(1) );
    
    % output is the concatenation of the two readings
    hx(1:2,:) = [ang1;ang2];
    
    % jacobian of hx
    Hx(1:2,1:2) = [-(x(2)-s1(2)) / ( (x(1)-s1(1))^2 + (x(2)-s1(2))^2 ), (x(1)-s1(1)) / ((x(1)-s1(1))^2 + (x(2)-s1(2))^2);
                   -(x(2)-s2(2)) / ( (x(1)-s2(1))^2 + (x(2)-s2(2))^2 ), (x(1)-s2(1)) / ((x(1)-s2(1))^2 + (x(2)-s2(2))^2)];
end



