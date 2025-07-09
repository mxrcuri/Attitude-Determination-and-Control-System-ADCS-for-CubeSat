function [BfieldNav, pqrNav] = Navigation(BfieldMeasured, pqrMeasured)
global BfieldNavPrev pqrNavPrev

s = 0.6; %tells us how much we believe our measurement

if sum(BfieldNavPrev) + sum(pqrNavPrev) == 0
    BfieldNav = BfieldMeasured;
    pqrNav = pqrMeasured;
else %%%Complimentary filter
    BiasEstimate = [0;0;0]; %If you know ur bias, getting a bias is hard
    BfieldNav = BfieldNavPrev*(1-s) + s*(BfieldMeasured-BiasEstimate);
    pqrNav = pqrNavPrev*(1-s) + s*pqrMeasured;
end

BfieldNavPrev = BfieldNav;
pqrNavPrev = pqrNav;