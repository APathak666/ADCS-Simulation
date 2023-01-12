global invJ IrR

%Mass and MoI of RWs
mr = 0.13;
rr = 42/1000;
hr = 19/1000;

%Orientation of RWs
n1 = [1; 0; 0];
n2 = [0; 1; 0];
n3 = [1; 1; 1];
n3 = n3/norm(n3);

no_of_RW = 3;

%Displace RW from CoM
r1 = [4; 0; 0]/1000;
r2 = [0; 4; 0]/1000;
r3 = [0; 0; 4]/1000;

Idisk = (3*rr^2 + hr^2)/2;
IrR = mr*[(rr^2)/12 0 0; 0 Idisk 0; 0 0 Idisk];

%Max values
max_rpm = 8000;
maxSpeed = max_rpm*pi/30;
maxTorque = 0.004;
maxAlpha = maxTorque/IrR(1, 1);

%Transformation matrix from RW frame to satellite body frame
T1 = Rscrew(n1);
T2 = Rscrew(n2);
T3 = Rscrew(n3);

%MoI of RW in satellite body frame
Ir1_B = T1' * IrR * T1;
Ir2_B = T2' * IrR * T2;
Ir3_B = T3' * IrR * T3;

%MoI of RW about satellite CoM (// axes theorem)
sr1 = skew(r1);
Ir1_BCoM = Ir1_B + mr*(sr1')*sr1;

sr2 = skew(r2);
Ir2_BCoM = Ir2_B + mr*(sr2')*sr2;

sr3 = skew(r3);
Ir3_BCoM = Ir3_B + mr*(sr3')*sr3;

%J matrix for multi-RW control
J = [Ir1_B*n1, Ir2_B*n2, Ir3_B*n3];
invJ = (J')*(inv(J*J'));
