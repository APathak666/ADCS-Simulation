MagScaleBias = 4e-7; %T
MagFieldBias = MagScaleBias*(2*rand() - 1); % (-1, 1)

MagScaleNoise = 1e-5; %T
MagFieldNoise = MagScaleNoise*(2*rand() - 1); % (-1, 1)

AngScaleBias = 0.01; %rad/s
AngFieldBias = AngScaleBias*(2*rand() - 1); % (-1, 1)

AngScaleNoise = 0.001; %rad/s
AngFieldNoise = AngScaleNoise*(2*rand() - 1); % (-1, 1)

EulerBias = pi/90; %rad
EulerNoise = EulerBias*(2*rand() - 1);
