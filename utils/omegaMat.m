function bigOmega = omegaMat(omega)
%
% Computes the Omega matrix of a 3x1 vector, omega 
%joan sola Quaternion kinematics for the error state KF p25
    if(size(omega,1) ~= 3 || size(omega,2) ~= 1)
        error('Input vector must be 3x1');
    end
    
    bigOmega = [ 0      ,      -omega';
                 omega  , -crossMat(omega) ];
end