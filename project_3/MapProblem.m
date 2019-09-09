%% Particle filter with Known/UNknown starting location
clear; close all; clc;

% choose starting location, either "known" or "unknown"
SL="known"; 

% Allocate memory
Xk = [];

% Set particle number, more for the unknown starting position
if SL=="known" 
    N = 10000;
else
    N = 80000;
end


%This draws a map, and allows us to manually
%draw the trajectory of the vehicle.
figure(1)
drawhouse();

% Get reference measures from clicks
[Xk]=pickHouse(Xk);

% Performance measure
tic

% Allocate memory and calculate speed components
n=size(Xk,1);
k=size(Xk,2);
vk=zeros(n,k);
Yk=zeros(n,k);

Yk(:,1)=vk(:,1);
for i=2:k
   vk(:,i)=Xk(:,i)-Xk(:,i-1);
end

% Calculate noise components
Xk_diff1=[0 0 diff(diff(Xk(1,:)))];
Xk_diff2=[0 0 diff(diff(Xk(2,:)))];
vk_diff1=diff(diff(vk(1,:)));
vk_diff2=diff(diff(vk(2,:)));

% Calculate noise components
T=size(Xk,2);


for i=1:T
	if SL=="known"
		Q(:,:,i)=[0 0 0 0;0 0 0 0;0 0 Xk_diff1(i)^2 0;0 0 0 Xk_diff2(i)^2];
	else
		Q(:,:,i)=[0 0 0 0;0 0 0 0;0 0 0.5 0;0 0 0 0.5];
	end
end

Rn1=0.1;
R=Rn1^2*[1 0;0 1];

% Generate measures
for i=2:k
   Yk(:,i)=vk(:,i)+mvnrnd([0 0],R)';
end


    
%   MU_X   N=k here     [k x 1] Approximated mean of y.
    
mu_x_sum1=sum(Xk(1,:));
mu_x_sum2=sum(Xk(2,:));
mu_x=[1/k*mu_x_sum1;1/k*mu_x_sum2];

mu_v_sum1=sum(vk(1,:));
mu_v_sum2=sum(vk(2,:));
mu_v=[1/k*mu_v_sum1;1/k*mu_v_sum2];
    
%   SIGMA_Y     [n x n] Approximated covariance of y.

Sigma_x_sum  = zeros(2,2);
Sigma_v_sum  = zeros(2,2);

Sigma_x_sum=Sigma_x_sum+(Xk(:,1:k)-mu_x)*(Xk(:,1:k)-mu_x)';

Sigma_v_sum=Sigma_v_sum+(vk(:,1:k)-mu_v)*(vk(:,1:k)-mu_v)';


Sigma_x=Sigma_x_sum/(k-1);
Sigma_v=Sigma_v_sum/(k-1);
Sigma=[Sigma_x(1,1) Sigma_x(1,2) 0 0;
        Sigma_x(2,1) Sigma_x(2,2) 0 0;
            0 0 Sigma_v(1,1) Sigma_v(1,2);
                0 0 Sigma_v(2,1) Sigma_v(2,2)];
				
% Priors and inputs for PF filter
x_0=[Xk(1,1) Xk(2,1) vk(1,1) vk(2,1)]';
X_kmin1=x_0;
proc_f = @(Xkmin1,K) proc(Xkmin1,K);
meas_h = @(X_k_upd,Xkmin1,R) meas(X_k_upd,Xkmin1,R);

P_0=[0 0 0 0;
        0 0 0 0;
            0 0 0.5 0;
                0 0 0 0.5];
            
% Run PF filter with resampling				
[xfp, Pfp, Xp, Wp] = pfFilter(X_kmin1, P_0, Yk, proc_f, Q, meas_h, R, ...
                                  N, SL);

K=size(Xp,3);

fig = figure;
for T=1:K

drawhouse()

p1 = Xp(1,:,T);
p2 = Xp(2,:,T);
N=size(Xp,2);
K=size(Xp,3);
pointsize = 20;
h(1,1) = scatter( p1(:,1), p2(:,1), pointsize, 'r');
%hold on
h(1,2:N) = scatter( p1(:,2:N), p2(:,2:N), pointsize, 'r');
hold off
   drawnow
   frame = getframe(fig);
   im{T} = frame2im(frame);
end
close;

toc
%figure;
%for T = 1:K
%    subplot(6,6,T);
%   imshow(im{T});
%end

% Export gif
if SL=="known" 
    filename = 'PF_Known.gif'; % Specify the output file name
else
    filename = 'PF_UNKnown.gif'; % Specify the output file name
end

for T = 1:K
    [A,map] = rgb2ind(im{T},256);
    if T == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.8);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.3);
    end
end

function [xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Yk, proc_f, proc_Q, meas_h, meas_R, ...
                             N, SL)                         
	%PFFILTER Filters measurements Y using the SIS or SIR algorithms and a
	% state-space model.
	%
	% Input:
	%   x_0         [n x 1] Prior mean
	%   P_0         [n x n] Prior covariance
	%   yk           [m x K] Measurement sequence to be filtered
	%   proc_f      Handle for process function f(x_k-1)
	%   proc_Q      [n x n] process noise covariance
	%   meas_h      Handle for measurement model function h(x_k)
	%   meas_R      [m x m] measurement noise covariance
	%   N           Number of particles
	%   bResample   boolean false - no resampling, true - resampling
    %   SL          Starting location, "known" or "unknown"
	% Output:
	%   xfp         [n x K] Posterior means of particle filter
	%   Pfp         [n x n x K] Posterior error covariances of particle filter
	%   Xp          [n x N x K] Particles for posterior state distribution in times 1:K
	%   Wp          [N x K] Non-resampled weights for posterior state x in times 1:K

    K=size(Yk,2);
	 
    % Based on initial location, choose Gaussian for known and normal
    % distribution on all map for unknown location
    if SL=="known"
		X_kmin1=mvnrnd(x_0,P_0,N)';
    else
		rng(0,'twister');
		x1 = 1;
		x2 = 11;
		X_kmin1_x = (x2-x1).*rand(1,N) + x1;

		y1 = 1;
		y2 = 9;
		X_kmin1_y = (y2-y1).*rand(1,N) + y1;

		vx1=-0.1;
		vx2=0.1;
		X_kmin1_vx= (vx2-vx1).*rand(1,N) + vx1;

		vy1=-0.1;
		vy2=0.1;
		X_kmin1_vy= (vy2-vy1).*rand(1,N) + vy1;
		X_kmin1=[X_kmin1_x;X_kmin1_y;X_kmin1_vx;X_kmin1_vy];
		Xp_0(:,:,1)=X_kmin1;
	end
    
	%Road check
	for i=1:N    
		u(:,i)=isOnRoad(X_kmin1(1,i),X_kmin1(2,i),X_kmin1(3,i),X_kmin1(4,i));
		X_kmin1(:,i)=u(:,i);
    end
    
	W_kmin1=ones(1,N)*1/N;
	
	for s=1:K
	
		[X_k, W_k] = pfFilterStep(X_kmin1, W_kmin1, Yk(:,s), proc_f, proc_Q(:,:,s), meas_h, meas_R, K);
	
    %Road check
	for i=1:N    
		u=isOnRoad(X_k(1,i),X_k(2,i),X_k(3,i),X_k(4,i));
		X_k(:,i)=u;
    end
	
	%if bResample == 'true'
	[X_k, W_k, j] = resampl(X_k, W_k); %resample
	
	%For next k step input kmin1
	X_kmin1=X_k;
	W_kmin1=W_k;	
	
	%Particles and weights
	Xp(:,:,s)=X_k;
	Wp(1,:,s)=W_k;

	%Mean and covariance                          
	xfp(:,s)= Xp(:,:,s) * Wp(1,:,s)';
	Pfp(:,:,s) = (Xp(:,:,s) - xfp(:,s)) * ((Xp(:,:,s) - xfp(:,s))' .* Wp(:,s));   
	
	end
end


function [Xk, Wk, j] = resampl(Xk, Wk)
    N=size(Xk,2);    
    X_size=size(Xk,2);
    [y,idx]=datasample(Xk,N,2,'Weights',Wk);
    Xk=y;
    j=idx;
    Wk(:,1:N)=1/N;
end

function [X_k, W_k] = pfFilterStep(X_kmin1, W_kmin1, Yk, proc_f, proc_Q, meas_h, meas_R, K)
    N=size(W_kmin1,2);
    n=size(X_kmin1(:,1),1);
    m=size(Yk(:,1),1);
    q=mvnrnd(zeros(1,n),proc_Q,N)';
    r=mvnrnd(zeros(1,m),meas_R)';    
    q(:,1:N);
    proc_f(X_kmin1(:,1),K);
    q(:,1);
    X_k_upd(:,1:N)=proc_f(X_kmin1(:,1:N),K)+q(:,1:N);
     
    for i=1:N
        Y_k_upd(:,i)=meas_h(X_k_upd(:,i),X_kmin1(:,i),meas_R);
    end
    Y_k_upd;

    for i=1:N
        n=size(X_k_upd(:,1),1);
        vk_temp=[X_k_upd(3,i) X_k_upd(4,i)]';
        w_k_upd(:,i) = W_kmin1(:,i)*1/((2*pi)^(n/2)*sqrt(det(meas_R)))*exp(-1/2*transpose((vk_temp-Yk))*inv(meas_R)*(vk_temp-Yk));  
    end
    X_k=X_k_upd;
    
    % Normalize to form a probability distribution (i.e. sum to 1).
	W_k = w_k_upd/sum(w_k_upd);

end


function [Xk] = proc(Xkmin1,K)
	N=size(Xkmin1,2);
	Xkmin1;
	Xk=[1 0 1 0;0 1 0 1;0 0 1 0;0 0 0 1]*Xkmin1;
end

function [Yk] = meas(X_k_upd,X_kmin1,meas_R)
    %take first two rows of state vector to calculate velocity
    %[X_k_upd(1,1)-X_kmin1(1,1);X_k_upd(2,1)-X_kmin1(2,1)];
    r=mvnrnd([0 0],meas_R)';
    Yk=[X_k_upd(1,1)-X_kmin1(1,1);X_k_upd(2,1)-X_kmin1(2,1)]+r;
end

function [u] = isOnRoad(inp_x,inp_y,inp_vx,inp_vy)
	% Input:    vectors with x and y positions
	%
	% Output:   a vector u such that u(i) = 1 if (x(i),y(i)) is on the road
	%           and 0 otherwise. 
	%
	%   Make sure that x and y are column vectors
	n   =   length(inp_x);      
	x = reshape(inp_x,n,1); 
	y = reshape(inp_y,n,1);

	%   The number of buildings (including two rectangles in the middle)
	m = 9;             

	%   To check if any vector is in any building we create
	%   matrices of size n x m:
	X = x*ones(1,m);
	Y = y*ones(1,m);

	%   We should check that we are on the map
	bounds = ([1+1i 1+9*1i 11+9*1i 11+1i]);

	%   And that we are not in any of these houses
	house = zeros(m,5);
	house(1,:) = ([2+5.2*1i 2+8.3*1i 4+8.3*1i 4+5.2*1i 2+5.2*1i]);%House 1
	house(2,:) = ([2+3.7*1i 2+4.4*1i 4+4.4*1i 4+3.7*1i 2+3.7*1i]);%House 2
	house(3,:) = ([2+2*1i 2+3.2*1i 4+3.2*1i 4+2*1i 2+2*1i]);%House 3
	house(4,:) = ([5+1i 5+2.2*1i 7+2.2*1i 7+1i 5+1i]);%House 4
	house(5,:) = ([5+2.8*1i 5+5.5*1i 7+5.5*1i 7+2.8*1i 5+2.8*1i]);%House 5
	house(6,:) = ([5+6.2*1i 5+9*1i 7+9*1i 7+6.2*1i 5+6.2*1i]);%House 6
	house(7,:) = ([8+4.6*1i 8+8.4*1i 10+8.4*1i 10+4.6*1i 8+4.6*1i]);%House 7
	house(8,:) = ([8+2.4*1i 8+4*1i 10+4*1i 10+2.4*1i 8+2.4*1i]);%House 8
	house(9,:) = ([8+1.7*1i 8+1.8*1i 10+1.8*1i 10+1.7*1i 8+1.7*1i]);%House 9

	%   Let us check if we are in any of the houses:
	X1 = X >= ones(n,1)*real(house(:,1))';
	X2 = X <= ones(n,1)*real(house(:,3))';
	Y1 = Y >= ones(n,1)*imag(house(:,1))';
	Y2 = Y <= ones(n,1)*imag(house(:,2))';
	XX = X1 & X2;               % Finds houses that match the x-vector
	YY = Y1 & Y2;               % Finds houses that match the y-vector
	UU = XX & YY;               % Finds houses that match both x and y
	u1 = ~any(UU, 2); % Sets u(i)=0 if (x(i),y(i)) is in a house

	%   We should also make sure that the vectors are in the map
	x3 = x > ones(n,1)*real(bounds(1))';
	x4 = x < ones(n,1)*real(bounds(3))';
	y3 = y > ones(n,1)*imag(bounds(1))';
	y4 = y < ones(n,1)*imag(bounds(2))';

	xx = x3 & x4;        %   Checks that the x-coordinates are in the map
	yy = y3 & y4;        %   and that the y-coordinates are in the map
	u2 = xx & yy;        %   Both must be inside

	% Finally, we set the output to zero if (x,y) is either in a building
	% or outside the map:
	u = u1 & u2;

	if u==0
		inp_x=0;
		inp_y=0;
		inp_vx=0;
		inp_vy=0;
	else
		inp_x=inp_x;
		inp_y=inp_y;
		inp_vx=inp_vx;
		inp_vy=inp_vy;
	end
	u=[inp_x inp_y inp_vx inp_vy]';

end

function drawhouse()
    clf
    hold on
    plot([1+1i 1+9*1i 5+9*1i])
    plot([7+9*1i 11+9*1i 11+1i 7+1i]);plot([5+1i 1+1i])
    plot([2+5.2*1i 2+8.3*1i 4+8.3*1i 4+5.2*1i 2+5.2*1i])%House 1
    plot([2+3.7*1i 2+4.4*1i 4+4.4*1i 4+3.7*1i 2+3.7*1i])%House 2
    plot([2+2*1i 2+3.2*1i 4+3.2*1i 4+2*1i 2+2*1i])%House 3
    plot([5+1i 5+2.2*1i 7+2.2*1i 7+1i])%House 4
    plot([5+2.8*1i 5+5.5*1i 7+5.5*1i 7+2.8*1i 5+2.8*1i])%House 5
    plot([5+6.2*1i 5+9*1i]);plot([7+9*1i 7+6.2*1i 5+6.2*1i])%House 6
    plot([8+4.6*1i 8+8.4*1i 10+8.4*1i 10+4.6*1i 8+4.6*1i])%House 7
    plot([8+2.4*1i 8+4*1i 10+4*1i 10+2.4*1i 8+2.4*1i])%House 8
    plot([8+1.7*1i 8+1.8*1i 10+1.8*1i 10+1.7*1i 8+1.7*1i])%House 9

    axis([0.8 11.2 0.8 9.2])
    title('Map','FontSize',20)
end 

function [Xk] = pickHouse(Xk)
    disp('Start clicking in the graph to create a trajectory!')
    disp('Press "Return" to finish.')
    key = '1';
   
    clear h
    while true
        [X,Y,key]=ginput(1);
        if isempty(key) || key ~= 1
            break
        end
        Xk = [Xk, [X; Y]];
        if exist('h', 'var')
            h.XData = Xk(1,:);
            h.YData = Xk(2,:);
        else
            h = plot(Xk(1,:),Xk(2,:),'k-*');
        end
        drawnow;
    end
end
