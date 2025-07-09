function [BB, pqr] = Sensor(BB, pqr)
global MagFieldBias AngFieldBias 
global MagFieldNoise AngFieldNoise 

for idx= 1:3
    %%%Get our sensor params
    sensor_params
    %%%Pollute the data
    BB(idx) = BB(idx) + MagFieldBias + MagFieldNoise;
    pqr(idx) = pqr(idx) + AngFieldBias + AngFieldNoise;
end