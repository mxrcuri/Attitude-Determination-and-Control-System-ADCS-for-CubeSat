function dstatedt = Satellite(t,state) %function of t and the state vector
global BB invI I m nextMagUpdate lastMagUpdate lastSensorUpdate nextSensorUpdate
global BfieldMeasured pqrMeasured BfieldNav pqrNav current
global BfieldNavPrev pqrNavPrev

%%%get inertia parameters
inertia

x = state(1);
y = state(2);
z = state(3);
q0123 = state(7:10);
p =state(11);
q = state(12);
r = state(13);
pqr = state(11:13);

%%%Translational Kinematics
vel = state(4:6);

%%%Rotational kinematics
PQRMAT = [0 -p -q -r; p 0 r -q; q -r 0 p;r q -p 0];
q0123dot = 0.5*PQRMAT*q0123;


%%%GRavity model
planet
r = state(1:3);
rho = norm(r);
rhat = r/rho;
Fgrav = -(G*M*m/rho^2)*rhat;

%%%Call the magnetic field model
if t >= lastMagUpdate
    lastMagUpdate = lastMagUpdate + nextMagUpdate;
    %%%convert
    phiE = 0;
    thetaE = acos(z/rho);
    psiE = atan2(y,x);
    latitude = 90-thetaE*180/pi;
    longitude = psiE*180/pi;
    rhokm = rho/1000;
    [BN, BE, BD] = igrf('01-Jan-2020', latitude, longitude, rhokm, 'geocentric');
    BNED = [BN;BE;-BD];
    BI = TIB(phiE, thetaE+pi, psiE)*BNED;
    BB = TIBquat(q0123)'*BI;
    BB = BB*1e-9;
end

if t >= lastSensorUpdate
    %%%SENSOR BLOCK
    lastSensorUpdate = lastSensorUpdate + nextSensorUpdate; %for discrete nature
    [BfieldMeasured, pqrMeasured] = Sensor(BB, pqr);

    %%%NAVIGATION BLOCK
    [BfieldNav, pqrNav] = Navigation(BfieldMeasured, pqrMeasured); %%Complimentary filter
end

%%%CONTROL BLOCK %Detumbling (satellite at 600km would detumble in about 12 orbits ~ 15 hours)
current = Control(BfieldNav, pqrNav);
magtorquer_params
muB = current*n*A;
%%Magtorquer Model

LMN_magtorquers = cross(muB, BB);


%%%Translational Dynamics
F = Fgrav;
accel = F/m;

%%%Rotational Dynamics
H = I*pqr;
pqrdot = invI*(LMN_magtorquers - cross(pqr, H));


%%%Return derivatives vector
dstatedt = [vel;accel;q0123dot;pqrdot];