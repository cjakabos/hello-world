%% Driveline control
clear; close all; clc;

% Vehicle parameters
Jc=6250;                        % Chassis inertia [kgm^2]
Jf=0.625;                       % Flywheel inertia [kgm^2]
ds=1000;                        % Driveshaft damping coefficient [Nms/rad]
cs=75000;                       % Driveshaft spring coefficient [Nm/rad]
r=57;                           % Gear ratio [-]

% A, B, C and H matrices
A=[[-ds/Jf/r^2 ds/Jf/r -cs/Jf/r];[ds/Jc/r -ds/Jc cs/Jc];[1/r -1 0]];
B=[1/Jf 0 0]';
H=[0 -1/Jc 0]';
C=[1 0 0];

% Inital conditions for real plant
%---------------------------------------------------
x0=[120 2 0]';                % Initial conditions for the state variables

% Control design
%Section for observability matrix, you have to set C according 
%which variable is measured

%Measure first state x1
C=[1 0 0];
WO1=[C;C*A;C*A*A];
det(WO1);

%Measure third state x3
C=[0 0 1];
WO2=[C;C*A;C*A*A];
det(WO2);

%Controller design
syms v0 a b ksi om;
ksi=1;
om=5;
chA=charpoly(A);

Wr=[B A*B A^2*B];
a1=chA(1,2);
a2=chA(1,3);
a3=chA(1,4);

%define poles
p1=5*ksi*om;
p2=om*om*(1+6*ksi*ksi);
p3=3*ksi*om^3;

P=[p1-a1 p2-a2 p3-a3];
P=[p1-a1 p2-a2 p3-a3];
coff=[1 a1 a2;0 1 a1;0 0 1];
Wrh=inv(coff);
K=P*Wrh*inv(Wr);

%Original C again
C=[0 1 0];
kr=-1/(C*inv(A-B*K)*B);

%The eigenvalues can be determined from the closed loop system matrix
e = eig(A-B*K);

% Measure x1, update C
C=[1 0 0];
kr=-1/(C*inv(A-B*K)*B);

%Since the -15 is the most further out feedback pole, if we want 4 times
%faster observer poles, they should be placed at -60
%Observer pole placement A-LC
syms l_11 l_21 l_31;
L=[l_11;l_21;l_31];
realcharpol=charpoly(A-L*C);

%from (s+b)^3=s^3+3*s^2*b+3*s*b^2+b^3, b=60 from above
desiredcharpol=[1 3*60 3*60^2 60^3];

eqns = [desiredcharpol(1,2) == realcharpol(1,2), desiredcharpol(1,3) == ...
    realcharpol(1,3),desiredcharpol(1,4)==realcharpol(1,4)];
vars = [l_11 l_21 l_31];
S = solve(eqns, vars);

%Access the solutions by addressing the elements of the structure.
L=[vpa(S.l_11);vpa(S.l_21);vpa(S.l_31)];
L=[179.3475;101.5777;-3.7387];
L=round(L);
D=zeros(3,2);

%---------------------------------------------------
% For simulation purposes
%---------------------------------------------------
B1=[B H];   % Put the B and H matrix together as one matrix B1
B2=[B L];   % Put the B and L matrix together as one matrix B2
C1=eye(3);  % Output all state variables from the model
D1=zeros(5,2);

%Determine the eigenvalues for the estimator error dynamics,
%when using the controller gain values are rounded off to the 
%nearest respective integer value!
seb=eig(A-L*C)


% Determine the eigenvalues for the closed-loop system,A-BK
%when using the estimator gain values are rounded off to the
%nearest respective integer value!
seb2=eig(A-B*round(K))




