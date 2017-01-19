function qInv = quatInv(quat)
%
% Computes the inverse (or conjugate) of a unit quaternion
% using the {1,i,j,k} convention
%
    if( size(quat,1) ~= 4 || size(quat,2) ~= 1 )
        error('Input quaternion must be 4x1');
    end
    
  %  if( abs(norm(quat) - 1) > eps )
   %     error('Input quaternion must be unit-length');
   % end
    
    qInv(1,1) = quat(1);
    qInv(2:4,1) = -quat(2:4);
end