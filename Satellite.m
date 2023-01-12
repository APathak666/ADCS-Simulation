function statederiv = Satellite(t, state)

%Globals
global B_B invI I Is m nextMagUpdate lastMagUpdate nextSensorUpdate lastSensorUpdate
global BFieldMeasured pqrMeasured BFieldNav pqrNav ptpMeasured ptpNav ptpNavPrev
global Ir1_BCoM Ir2_BCoM Ir3_BCoM n1 n2 n3 maxSpeed maxAlpha Ir1_B Ir2_B Ir3_B

x = state(1);
y = state(2);
z = state(3);

%Translational kinematics
vel = state(4:6);

%Rotational kinematics
q0123 = state(7:10);
ptp = Quaternions2EulerAngles(q0123)';
p = state(11);
q = state(12);
r = state(13);
pqr = state(11:13);

%RW angular velocities
w123 = state(14:16);
w1 = w123(1);
w2 = w123(2);
w3 = w123(3);

%Compute total angular momentum
H = I*pqr + w1*Ir1_B*n1 + w2*Ir2_B*n2 + w3*Ir3_B*n3;

quatmat = [0 -p -q -r;
           p 0 r -q;
           q -r 0 p;
           r q -p 0];
q0123dot = 0.5*quatmat*q0123;

%Inertia and mass
m = 2.6; %kilograms

%Gravity model
planet
r = state(1:3);
rho = norm(r);
rhat = r/rho;
Fgrav = -(G*M*m/rho^2)*rhat;

%Convert Cartesian x, y, z into lat, lon, alt
phiE = 0;
thetaE = acos(z/rho);
psiE = atan2(y, x);
latitude = 90 - thetaE*180/pi;
longitude = psiE*180/pi;
rhokm = (rho)/1000;

%Magnetic field model
if t >= lastMagUpdate
    lastMagUpdate = lastMagUpdate + nextMagUpdate;

    [BN, BE, BD] = igrf('01-Jan-2020', latitude, longitude, rhokm, 'geocentric');

    %Convert NED to inertial frame
    B_NED = [BN; BE; BD];
    B_I = TIB(phiE, thetaE+pi, psiE)*B_NED;

    B_B = TIBquat(q0123)' * B_I;
    B_B = B_B * 1e-9;
end

if  t >= lastSensorUpdate
    %Sensor updates
    lastSensorUpdate = lastSensorUpdate + nextSensorUpdate;
    [BFieldMeasured, pqrMeasured, ptpMeasured] = Sensor(B_B, pqr, ptp);

    %Navigation block
    [BFieldNav, pqrNav, ptpNav] = Navigation(BFieldMeasured, pqrMeasured, ptpMeasured);    
end

%Call magnetorquer params
magnetorquer_params;

%Control block
[current, RWalphas] = Control(BFieldNav, pqrNav, ptpNav);

%Magnetorquers model
muB = current*n*A;
LMN_magnetorquers = cross(muB, B_B);

%RWs
w123dot = [0; 0; 0];
for idx = 1:3
    if abs(w123(idx)) > maxSpeed
        w123dot(idx) = 0;
    else
        if RWalphas(idx) >= maxAlpha
            RWalphas(idx) = sign(RWalphas(idx))*maxAlpha;
        end

        w123dot(idx) = RWalphas(idx);
    end
end

LMN_RWs = Ir1_B*w123dot(1)*n1 + Ir2_B*w123dot(2)*n2 + Ir3_B*w123dot(3)*n3;

LMN = LMN_magnetorquers - LMN_RWs;

%Rotational dynamics
pqrdot = invI*(LMN - cross(pqr, H));

%Translational dynamics
F = Fgrav;
accel = F/m;

%Return derivaties
statederiv = [vel; accel; q0123dot; pqrdot; w123dot];