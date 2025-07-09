function current = Control(BfieldNav, pqrNav)

k = 67200; %%gain
% muB = n*i*A
% pqr*bfield/(n*A) ~= 6e-7
magtorquer_params
current = k*cross(pqrNav, BfieldNav)/(n*A);
%want the current to be in amps ~= 40mA = 4e-2

%%%Add in saturation control
if sum(abs(current)) > 0.04
    current = current/norm(current)*0.04;
end