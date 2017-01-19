function Fx = calcFx(wb, measurements_k)
% Multiplies the error state in the linearized continuous-time
% error state model
    Fx = zeros(6,6);
    omegaHat = measurements_k.omega - wb;
    Fx(1:3,1:3) = axisAngleToRotMat(omegaHat*measurements_k.dt)';
    Fx(1:3,4:6) = -eye(3)*measurements_k.dt;
    Fx(4:6,1:3) = zeros(3);
    Fx(4:6,4:6) = eye(3);
end