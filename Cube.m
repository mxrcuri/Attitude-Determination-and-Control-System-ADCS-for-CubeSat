function dstatedt = Cube(t,state)
global invI I m lastSensorUpdate maxSpeed
global nextSensorUpdate pqrMeasured pqrNav
global pqrNavPrev Is Ir1Bcg Ir2Bcg Ir3Bcg n1 n2 n3
global maxAlpha Ir1B Ir2B Ir3B ptpMeasured ptpNavPrev ptpNav rwalphas
global AngFieldBias EulerBias
global AngFieldNoise EulerNoise
global TEf PEi
global timestep

%%load inertia parameters
inertia
reaction_wheel_params

g=9.81;

%current state
q0123 = state(1:4);
q0 = state(1);
q1 = state(2);
q2 = state(3);
q3 = state(4);
q_vector = state(2:4);
ptp = Quaternions2EulerAngles(q0123')';
p = state(5);
q = state(6);
r = state(7);
pqr = state(5:7);
w123 = state(8:10);

%PQRMAT = [0 -p -q -r;p 0 r -q;q -r 0 p;r q -p 0];
%q0123dot = 0.5*PQRMAT*q0123;

q_skew = [0 -q3 q2; q3 0 -q1; q2 q1 0];
G_q = [q_vector q0*eye(3) - q_skew];  %% G(q)
gamma = [1 1 -1 0; 1 -1 0 1; -1 0 -1 1; 0 1 1 1];

%%% rotation quaternion time derivative 
q0123dot = 0.5*G_q'*pqr;

%%%rotation matrix in terms of quaternions
R_q = q0*eye(3) + q_vector*q_vector' + 2*q0*q_skew + q_skew*q_skew;  %% R(q)

if t >= lastSensorUpdate
    %%%%SENSOR BLOCK
    lastSensorUpdate = lastSensorUpdate + nextSensorUpdate;
    [pqrMeasured,ptpMeasured] = Sensor(pqr,ptp); 
    
    %%%NAVIGATION BLOCK
    [pqrNav,ptpNav] = Navigation(pqrMeasured,ptpMeasured);   
end


%%%CONTROL BLOCK
rwalphas = Control(pqrNav,ptpNav,timestep);

%%%Reaction Wheels

w123dot = [0;0;0];
for i = 1:3 %%% iterate over wheels
    if abs(w123(i)) > maxSpeed
        w123dot(i) = 0;
    else
        if abs(rwalphas(i)) > maxAlpha
            rwalphas(i) = sign(rwalphas(i))*maxAlpha;
        end
        w123dot(i) = rwalphas(i);
    end
    % w123dot(i) = rwalphas(i);
end


%%% kinetic energy
T = 0.5*pqr'*Ic_bar*pqr + 0.5*(pqr + w123)'*Iw*(pqr + w123);
%%% potential energy
V = m*cg'*R_q*g;

mc_bar = ms + 2*mw;
%%% equations of motions
tau = Iw * w123dot; %% get torque required
%%% angular acceleration of cube
pqrdot = invI*(-cross(pqr, (Ic_bar*pqr + Iw*w123)) - mc_bar*g*l.*(G_q*gamma*q0123) - tau);
%disp(pqrdot);

%%%Return derivatives vector
dstatedt = [q0123dot;pqrdot;w123dot];
