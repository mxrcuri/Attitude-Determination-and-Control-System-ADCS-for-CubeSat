%%%Initialize
clear
clc
close all

tic

%%%Globals
global BB m I invI lastMagUpdate nextMagUpdate lastSensorUpdate nextSensorUpdate
global BfieldMeasured pqrMeasured BfieldNav pqrNav
global BfieldNavPrev pqrNavPrev current

BfieldNavPrev = [0;0;0];
pqrNavPrev = [0;0;0];

disp('Simulation started')

%%%Planet parameters
planet

%%get mass and inertia properties
inertia

%%%Initial conditions
altitude = 600*1000;
x0 = R + altitude;
y0 = 0;
z0 = 0;
xdot0 = 0; 
inclination = 56*pi/180;
semi_major = norm([x0;y0;z0]);
vcircular = sqrt(mu/semi_major);
ydot0 = vcircular*cos(inclination);
zdot0 = -vcircular*sin(inclination);

%%%Initial conditions for Attitude and Angular velocity
phi0 = 0;
theta0 = 0;
psi0 = 0;
ptp0 = [phi0;theta0;psi0];
q0123_0 = EulerAngles2Quaternions(ptp0);
p0 = 0.08;
q0 = -0.02;
r0 = 0.03;

state = [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0]; %stateinitial to state because state will be constantly changing

%%%%Need time window
period = 2*pi/sqrt(mu)*semi_major^(3/2);
number_of_orbits = 12;
tfinal = period*number_of_orbits;
timestep = 1;
tout = 0:timestep:tfinal;
stateout = zeros(length(tout),length(state));

%%%This is where we integrate the equations of motion
%%%[tout, stateout] = ode45(@Satellite,tspan,stateinitial);
%%%disp('Simulation Complete')

%%%Loop through stateout to extract magnetic field
%%%Loop through time to integrate
BxBout = 0*stateout(:,1);
ByBout = BxBout;
BzBout = BxBout;

BxBm = 0*stateout(:,1);
ByBm = BxBout;
BzBm = BxBout;
pqrm = zeros(length(tout),3);

BxBN = 0*stateout(:,1);
ByBN = BxBout;
BzBN = BxBout;
pqrN = zeros(length(tout),3);

ix = 0*stateout(:,1);
iy = ix;
iz = ix;
nextMagUpdate = 10; %updates every 10 seconds
lastMagUpdate = 0;

%%%Sensor paramaters
lastSensorUpdate = 0;
nextSensorUpdate = 1;
sensor_params

%%%Print next
next = 100;
lastPrint = 0;

for idx = 1:length(tout)
    %save the current state
    stateout(idx,:) = state';

    if tout(idx) > lastPrint
        disp(['Time = ', num2str(tout(idx))])
        lastPrint = lastPrint + next;
    end

    %dstatedt = Satellite(tout(idx),stateout(idx,:)'); %everytime you call this, it will populate the B vector and then u can extract it
    
    %%%Then we make the 4 function calls for the RK4
    k1 = Satellite(tout(idx),state);
    k2 = Satellite(tout(idx)+timestep/2,state+k1*timestep/2);
    k3 = Satellite(tout(idx)+timestep/2,state+k2*timestep/2);
    k4 = Satellite(tout(idx)+timestep,state+k3*timestep);
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    state = state + k*timestep; %next state is current state +...

    %%%Save the Current
    ix(idx) = current(1);
    iy(idx) = current(2);
    iz(idx) = current(3);

    %Save the magnetic field
    BxBout(idx) = BB(1);
    ByBout(idx) = BB(2);
    BzBout(idx) = BB(3);
    BxBm(idx) = BfieldMeasured(1);
    ByBm(idx) = BfieldMeasured(2);
    BzBm(idx) = BfieldMeasured(3);
    BxBN(idx) = BfieldNav(1);
    ByBN(idx) = BfieldNav(2);
    BzBN(idx) = BfieldNav(3);
    %%%Save the polluted pqr signal
    pqrm(idx,:) = pqrMeasured';
    %%%Save the filtered pqr signal
    pqrN(idx,:) = pqrNav';
end

%%Convert state to kilometers
stateout(:,1:3) = stateout(:,1:3)/1000;

%%%Extract state vector
xout = stateout(:,1);
yout = stateout(:,2);
zout = stateout(:,3);
q0123out = stateout(:,7:10);
ptpout = Quaternions2EulerAngles(q0123out);
pqrout = stateout(:,11:13);

%%%Make an earth
[X,Y,Z] = sphere(100);
X = X*R/1000;
Y = Y*R/1000;
Z = Z*R/1000;

%%%Plot X,Y,Z as a function of time
fig0 = figure();
set(fig0,'color','white')
plot(tout,xout,'b-','LineWidth',2)
hold on
grid on
plot(tout,yout,'r-','LineWidth',2)
plot(tout,zout,'g-','LineWidth',2)
xlabel('Time (sec)')
ylabel('Altitude (km)')
legend('X','Y','Z')

%%%Plot 3D orbit
fig = figure();
set(fig,'color','white');
plot3(xout,yout,zout,'b-','LineWidth',4)
grid on
hold on
surf(X,Y,Z, 'EdgeColor','none')
axis equal


%%%Plot Magnetic field
fig2 = figure();
set(fig2, 'color', 'white');
p1 = plot(tout,BxBout,'b-','LineWidth',2);
hold on
grid on
p2 = plot(tout,ByBout,'g-','LineWidth',2);
p3 = plot(tout,BzBout,'r-','LineWidth',2);
p1m = plot(tout,BxBm,'b:','LineWidth',2);
p2m = plot(tout,ByBm,'g:','LineWidth',2);
p3m = plot(tout,BzBm,'r:','LineWidth',2);
p1N = plot(tout,BxBN,'b--','LineWidth',2);
p2N = plot(tout,ByBN,'g--','LineWidth',2);
p3N = plot(tout,BzBN,'r--','LineWidth',2);
xlabel('Time(sec)')
ylabel('Mag field(T)')
legend('Bx','By','Bz','Bx Measured','By Measured','Bz Measured','Bx Nav','By Nav','Bz Nav');

%%%And Norm
Bnorm = sqrt(BxBout.^2 + ByBout.^2 + BzBout.^2);
fig3 = figure();
set(fig3,'color','white')
plot(tout,Bnorm,'LineWidth',2)
xlabel('Time (sec)')
ylabel('Norm of Magnetic Field (T)')
grid on

%%%Plot Euler Angles
fig3 = figure();
set(fig3, 'color', 'white');
plot(tout,ptpout,'LineWidth',2)
grid on
xlabel('Time(sec)')
ylabel('Angles(rad)')

%%%Plot Angular velocity
fig4 = figure();
set(fig4, 'color', 'white');
plot(tout,pqrout,'LineWidth',2)
grid on
hold on
plot(tout,pqrm,'--','LineWidth',2)
plot(tout,pqrN,'LineWidth',2)
xlabel('Time(sec)')
ylabel('Anglular velocity(rad/s)')
legend('p','q','r','p Measured','q Measured','r Measured','p Nav','q Nav','r Nav');

%%%Plot the current in the magnetorquers
fig6 = figure();
set(fig6,'color','white')
plot(tout,ix,'LineWidth',2)
hold on
plot(tout,iy,'LineWidth',2)
plot(tout,iz,'LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Current Magnetorquers (A)')
legend('ix','iy','iz')

toc

