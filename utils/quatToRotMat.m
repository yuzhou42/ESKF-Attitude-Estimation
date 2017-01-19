function C = quatToRotMat(quat)
%
% Converts a quaternion into a 3x3 rotation matrix
% using the {1,i,j,k} convention
%
    if( size(quat,1) ~= 4 || size(quat,2) ~= 1 )
        error('Input quaternion must be 4x1');
    end
    
    if( abs(norm(quat) - 1) > eps )
        if abs(norm(quat) - 1) > 0.1
            warning(sprintf('Input quaternion is not unit-length. norm(q) = %f. Re-normalizing.', norm(quat)));
        end
        quat = quat/norm(quat);
    end
    
    R = quatRightComp(quatInv(quat))' * quatLeftComp(quat);
    C = renormalizeRotMat( R(2:4,2:4) );
      
end