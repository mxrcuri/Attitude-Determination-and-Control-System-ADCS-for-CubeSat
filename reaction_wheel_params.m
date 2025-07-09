global l cg m

%%% compute inertia of reation wheel
mw = 0.5;
rr = 0.05;
hr = 19/1000;
%IrR = mr*[(1/2)*rr^2 0 0; 0 (1/12)*(3*rr^2+hr^2) 0; 0 0 (1/12)*(3*rr^2+hr^2)];


Iwxx = 7e-4;
Iwyy = 4.67e-4;
%%% inertia matrices of reaction wheels (assumed to be a disk)
Iw1G = [Iwxx 0 0; 0 Iwyy 0; 0 0 Iwyy];
rw1 = [0; l/2; l/2];
Iw2G = [Iwyy 0 0; 0 Iwxx 0; 0 0 Iwyy];
rw2 = [l/2; 0; l/2];
Iw3G = [Iwyy 0 0; 0 Iwyy 0; 0 0 Iwxx];
rw3 = [l/2; l/2; 0];

%% rwi tilde's
rw1_skew = [0 -rw1(3) rw1(2); rw1(3) 0 -rw1(1); -rw1(2) rw1(1) 0];
rw2_skew = [0 -rw2(3) rw2(2); rw2(3) 0 -rw2(1); -rw2(2) rw2(1) 0];
rw3_skew = [0 -rw3(3) rw3(2); rw3(3) 0 -rw3(1); -rw3(2) rw3(1) 0];

%%% inertia matrices on pivot O
Iw1O = Iw1G + mw*(rw1_skew)*rw1_skew';
Iw2O = Iw2G + mw*(rw2_skew)*rw2_skew';
Iw3O = Iw3G + mw*(rw3_skew)*rw3_skew';

%%%combined inertia for 3 reaction wheels
Iw = [Iwxx 0 0; 0 Iwxx 0; 0 0 Iwxx];

%%% max speed
rpm = 8000;
maxSpeed = 100000000;
%%% max angular accn
maxTorque = 100000; %N-m
maxAplha = maxTorque/Iwxx;  %%%max torque allowed is 57.14

% %%% orientation of reaction wheel
% n2 = [0;1;0];
% %%% transformation from RW frame to body frame of satellite
% T2 = Rscrew(n2);
% %%% compute inertia of of RW in body frame of satellite
% % inertia of first reaction wheel in the body frame of satellite
% Ir2B = T2'*IrR*T2;
% 
% %%% compute the inertia of RW at the cg of satellite in body frame:
% %%% parallel axis theorem
% % Ir2Bcg = Ir2B + mr*skew(r2)'*skew(r2); %%% this was when the origin of reference frame was at the centre of cube
% %%% need to comput inerta matrix of cube at the edge 
% Icubepiv = Is + ms*skew(rs)'*skew(rs);
% %%% inertia matrix of reaction wheel at the edge
% Ir2Bpiv = Ir2B + mr*skew(rr)'*skew(rr);

