function [current, RWalphas] = Control(BFieldNav, pqrNav, ptpNav)

global current invJ IrR

%Call magnetorquer params
magnetorquer_params

%%Bdot controller
k = 67200; %gain
current = k*cross(pqrNav, BFieldNav);

%Saturation
if sum(abs(current)) > max_curr
    current = (current/sum(abs(current)))*max_curr;
end

%%RW Controller
if sum(pqrNav) < 0.1
    Kp = eye(3)*1*IrR(1, 1);
    ptpCommand = [0; 0; 0];
    pqrCommand = [0; 0; 0];
    Kd = eye(3)*45*IrR(1, 1);
    T_desired = -Kd*(pqrCommand - pqrNav) - Kp*(ptpCommand - ptpNav);
else
    T_desired = [0; 0; 0];
end

RWalphas = invJ*T_desired;