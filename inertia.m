%mass and inertia
global invI

ms = 2.6; %mass of sat in kilograms

%MoI of sat
lx = 0.1; %metres
ly = 0.1;
lz = 0.1;

Is = (ms/12)*[lx^2 + ly^2 0 0; 0 lx^2 + lz^2 0; 0 0 ly^2 + lz^2];

%Call RW params
RW_params

m = ms + no_of_RW*mr;
I = Is + Ir1_BCoM + Ir2_BCoM + Ir3_BCoM;

invI = inv(I);