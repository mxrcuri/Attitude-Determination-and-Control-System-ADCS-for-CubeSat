%%%Bias and noise with the random number generator

MagscaleBias = (4e-7);
MagFieldBias = MagscaleBias*(2*rand()-1); %-1 to 1

MagscaleNoise = (1e-5);
MagFieldNoise = MagscaleNoise*(2*rand()-1);

AngscaleBias = 0.01;
AngFieldBias = AngscaleBias*(2*rand()-1); %-1 to 1

AngscaleNoise = 0.001;
AngFieldNoise = AngscaleNoise*(2*rand()-1);