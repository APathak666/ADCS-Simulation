function [BB, pqr, ptp] = Sensor(BB, pqr, ptp)

for idx = 1:3
    %Get sensor params
    sensor_params

    %Pollute data
    BB(idx) = BB(idx) + MagFieldBias + MagFieldNoise;
    pqr(idx) = pqr(idx) + AngFieldBias + AngFieldNoise;
    ptp(idx) = ptp(idx) + EulerBias + EulerNoise;
end