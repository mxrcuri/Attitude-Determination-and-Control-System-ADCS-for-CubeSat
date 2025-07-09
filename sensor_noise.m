%%%Sensor Noise Parameters
global fsensor

AngscaleNoise = 0.001*fsensor; %%rad/s
AngFieldNoise = AngscaleNoise*(2*rand()-1); %%% -1 to 1 (0 to 1) (0 to 2)

EulerScaleNoise = 1*pi/180*fsensor;
EulerNoise = EulerScaleNoise*(2*rand()-1);