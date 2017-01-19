function Psi = axisAngleToRotMat(psi)
%
% Converts an axis-angle rotation vector into a rotation matrix with
% hamilton representation
%
% psi               - axis-angle rotation vector
%ref: joan sola 3D algebra for vision system in robotics p15
    theta = norm(psi);
    cp = cos(theta);
    sp = sin(theta);
    u = psi / norm(psi);
    uCross =  crossMat(u);
    Psi = eye(3) +  uCross*sp +  uCross'* uCross*(1-cp);
end