%Initialize
purge

tic

%Globals
global m I invI Is B_B lastMagUpdate nextMagUpdate nextSensorUpdate lastSensorUpdate
global BFieldMeasured pqrMeasured BFieldNav pqrNav BFieldNavPrev pqrNavPrev ptpNavPrev ptpMeasured ptpNav
global Ir1_BCoM Ir2_BCoM Ir3_BCoM n1 n2 n3 maxSpeed maxAlpha Ir1_B Ir2_B Ir3_B current


BFieldNavPrev = [0; 0; 0];
pqrNavPrev = [0; 0; 0];
ptpNavPrev = [0; 0; 0];

%Simulation of LEOsat
disp("Simulation started")

%Setup IGRF model
addpath 'igrf/'

%Get planet params
planet

%Get mass, inertia properties
inertia

%Initial conditions (attitude, angular velocity)
phi0 = 0;
theta0 = 0;
psi0 = 0;
ptp0 = [phi0; theta0; psi0];
q0123_0 = EulerAngles2Quaternions(ptp0);
p0 = 0.8;
q0 = -0.2;
r0 = 0.3;
w10 = 0;
w20 = 0;
w30 = 0;

%Initial conditions (position, velocity)
altitude = 600*1000;
x0 = R + altitude;
y0 = 0;
z0 = 0;
inclination = 56*pi/180;
semi_major_axis = norm([x0; y0; z0]);
vcircular = sqrt(mu/semi_major_axis);
xdot0 = 0;
ydot0 = vcircular*cos(inclination);
zdot0 = vcircular*sin(inclination);
state = [x0; y0; z0; xdot0; ydot0; zdot0; q0123_0; p0; q0; r0; w10; w20; w30];

%Time period
period = 2*pi/sqrt(mu)*semi_major_axis^(3/2);
number_of_orbits = 1;
tfinal = number_of_orbits*period;
%tfinal = 500;
timestep = 1;
tout = 0:timestep:tfinal;
stateout = zeros(length(tout), length(state));

next = 100;
lastSecond = 0;

lastMagUpdate = 0;
nextMagUpdate = 1;

%Sensor params
lastSensorUpdate = 0;
nextSensorUpdate = 1;
sensor_params

%Initialize magnetic field to dummy values
BxBout = 0*stateout(:, 1);
ByBout = BxBout;
BzBout = BxBout;

%Initialize sensors to dummy values
BxBm = 0*stateout(:, 1);
ByBm = BxBout;
BzBm = BxBout;

%Initialize filtered magnetic field to dummy values
BxBN = 0*stateout(:, 1);
ByBN = BxBN;
BzBN = BxBN;

%Initialize pqr measured and filtered values
pqrm = zeros(length(tout), 3);
pqrN = pqrm;

%Initialize current values
ix = 0*stateout(:, 1);
iy = ix;
iz = ix;

%Initialize Euler angles (measured and filtered)
ptpm = zeros(length(tout), 3);
ptpN = ptpm;

k1 = Satellite(tout(1), state);

for idx = 1:length(tout)

    %Print time every second
    if tout(idx) >= lastSecond
        disp(['Time = ', num2str(tout(idx))])
        lastSecond = lastSecond + next;
    end

    %Save current state
    stateout(idx, :) = state';
        
    %Save magnetic field
    BxBout(idx) = B_B(1);
    ByBout(idx) = B_B(2);
    BzBout(idx) = B_B(3);

    BxBm(idx) = BFieldMeasured(1);
    ByBm(idx) = BFieldMeasured(2);
    BzBm(idx) = BFieldMeasured(3);
    
    BxBN(idx) = BFieldNav(1);
    ByBN(idx) = BFieldNav(2);
    BzBN(idx) = BFieldNav(3);
    
    %Save polluted pqr signal
    pqrm(idx, :) =  pqrMeasured';
    
    %Save filtered pqr signal
    pqrN(idx, :) = pqrNav';
    
    %Save the current values
    ix(idx) = current(1);
    iy(idx) = current(2);
    iz(idx) = current(3);
    
    %Save Euler angles
    ptpm(idx, :) = ptpMeasured';
    ptpN(idx, :) = ptpNav';

    %RK-4 integrator
    k1 = Satellite(tout(idx), state);
    k2 = Satellite(tout(idx) + timestep/2, state + k1*timestep/2);
    k3 = Satellite(tout(idx) + timestep/2, state + k2*timestep/2);
    k4 = Satellite(tout(idx) + timestep, state + k3*timestep);
    k = (k1 + 2*k2 + 2*k3 + k4)/6;

    state = state + k*timestep;

end

%Extract state
stateout(:, 1:6) = stateout(:, 1:6)/1000;
xout = stateout(:, 1);
yout = stateout(:, 2);
zout = stateout(:, 3);
q0123out = stateout(:, 7:10);
pqrout = stateout(:, 11:13);
ptpout = Quaternions2EulerAngles(q0123out);
w123 = stateout(:, 14:16);

disp('Simulation complete')

%Plot x, y, z coords variation
fig0 = figure();
set(fig0, 'color', 'white');
grid on
hold on
plot(tout, xout, 'b-', 'LineWidth', 2);
plot(tout, yout, 'r-', 'LineWidth', 2);
plot(tout, zout, 'g-', 'LineWidth', 2);
legend('X', 'Y', 'Z')

%Plot 3D orbit
[X,Y,Z] = sphere(100);
X = X*R/1000;
Y = Y*R/1000;
Z = Z*R/1000;
fig = figure();
set(fig,'color','white')
plot3(xout,yout,zout,'b-','LineWidth',4)
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
hold on
surf(X,Y,Z,'EdgeColor','none')
axis equal

%Plot Magnetic Field
fig2 = figure();
set(fig2,'color','white')
p1 = plot(tout,BxBout,'b-','LineWidth',2);
hold on
grid on
p2 = plot(tout,ByBout,'r-','LineWidth',2);
p3 = plot(tout,BzBout,'g-','LineWidth',2);
p1m = plot(tout,BxBm,'b-s','LineWidth',2);
p2m = plot(tout,ByBm,'r-s','LineWidth',2);
p3m = plot(tout,BzBm,'g-s','LineWidth',2);
p1N = plot(tout,BxBN,'b--','LineWidth',2);
p2N = plot(tout,ByBN,'r--','LineWidth',2);
p3N = plot(tout,BzBN,'g--','LineWidth',2);
legend([p1,p2,p3,p1m,p2m,p3m,p1N,p2N,p3N],'Bx','By','Bz','Bx Measured','By Measured','Bz Measured','Bx Nav','By Nav','Bz Nav')
xlabel('Time (sec)')
ylabel('Mag Field (T)')

%Plot magnetic field norm
Bnorm = sqrt(BxBout.^2 + ByBout.^2 + BzBout.^2);
fig3 = figure();
set(fig3,'color','white')
plot(tout,Bnorm,'LineWidth',2)
xlabel('Time (sec)')
ylabel('Norm of Magnetic Field (T)')
grid on

%Plot Euler Angles
fig4 = figure();
set(fig4,'color','white')
p1 = plot(tout,ptpout*180/pi,'-','LineWidth',2);
hold on
p2 = plot(tout,ptpm*180/pi,'-s','LineWidth',2);
p3 = plot(tout,ptpN*180/pi,'--','LineWidth',2);
grid on
xlabel('Time (sec)')
ylabel('Euler Angles (deg)')
legend('Phi','Theta','Psi')
legend([p1(1),p2(1),p3(1)],'Actual','Measured','Nav')

%Plot Angular Velocity
fig5 = figure();
set(fig5,'color','white')
p1=plot(tout,pqrout,'-','LineWidth',2);
hold on
p2=plot(tout,pqrm,'-s','LineWidth',2);
p3=plot(tout,pqrN,'--','LineWidth',2);
grid on
xlabel('Time (sec)')
ylabel('Angular Velocity (rad/s)')
legend([p1(1),p2(1),p3(1)],'Actual','Measured','Nav')

%Plot the current
fig6 = figure();
set(fig6,'color','white')
plot(tout,ix*1000,'LineWidth',2)
hold on
plot(tout,iy*1000,'LineWidth',2)
plot(tout,iz*1000,'LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Current (mAmps)')
legend('X','Y','Z')

%Plot the Total current
fig6 = figure();
set(fig6,'color','white')
plot(tout,(abs(ix)+abs(iy)+abs(iz))*1000,'LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Total Current (mAmps)')

%Plot w of RWs
fig7 = figure();
set(fig7, 'color', 'white')
plot(tout, w123, 'LineWidth', 2)
grid on
xlabel('Time (sec)')
ylabel('Angular velocity of RWs (rad/sec)')

toc