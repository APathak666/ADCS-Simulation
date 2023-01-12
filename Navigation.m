function [BFieldNav, pqrNav, ptpNav] = Navigation(BFieldMeasured, pqrMeasured, ptpMeasured)

global BFieldNavPrev pqrNavPrev ptpNavPrev

s = 0.3;

%Estimated bias values
BFieldBiasEstimate = [0; 0; 0];
pqrBiasEstimate = [0; 0; 0];
ptpBiasEstimate = [0; 0; 0];

if sum(BFieldNavPrev) + sum(pqrNavPrev) + sum(ptpNavPrev) == 0
    BFieldNav = BFieldMeasured; 
    pqrNav = pqrMeasured;
    ptpNav = ptpMeasured;
else
    BFieldNav = BFieldNavPrev*(1 - s) + s*(BFieldMeasured - BFieldBiasEstimate);
    pqrNav = pqrNavPrev*(1 - s) + s*(pqrMeasured - pqrBiasEstimate);
    ptpNav = ptpNavPrev*(1-s) + s*(ptpMeasured - ptpBiasEstimate);
end


BFieldNavPrev = BFieldNav;
pqrNavPrev = pqrNav;
ptpNavPrev = ptpNav;