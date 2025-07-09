function R = Rscrew(nhat)
%%%Assume that nhat is a 3x1 vector in R3 space. 
%%compute R such that v(inertial) = R v(body)

x = nhat(1);
y = nhat(2);
z = nhat(3);

%%%yaw 
ps = atan2(y,x);
%%%Theta
theta = atan2(z,sqrt(x^2+y^2));
%%%Phi is always zero because it is a line
phi = 0;

R = TIB(phi,theta,ps);