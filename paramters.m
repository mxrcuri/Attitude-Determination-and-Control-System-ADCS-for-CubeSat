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

%%% inertia of cube
ms = 0.64; %%%inertia of cube is 0.00167 kg-m^2 and torque is 0.53 Nm
l = 0.15;
rs = [l/2; l/2; l/2]; 
rs_skew = [0 -rs(3) rs(2); rs(3) 0 -rs(1); -rs(2) rs(1) 0];
IsG = (1/6)*ms*l^2*[1 0 0; 0 1 0; 0 0 1];  %inertia tensor of cube about principal axes (its com)
IsO = IsG + ms*(rs_skew)*rs_skew';

%%%Add everything up
m = ms + 3*mw;
IcO = IsO + Iw1O + Iw2O + Iw3O;
Ic_bar = IcO - Iw;

%%%compute com
cg = (ms*rs + mw*(rw1 + rw2 + rw3))/m;

%%%Invert the matrix
invI = inv(Ic_bar);
