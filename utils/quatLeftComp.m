function qLC = quatLeftComp(quat)
%
% Computes the right-hand compound operator form of a quaternion
% using the {1,i,j,k} convention (q^+ in Tim's book)%
%
    if( size(quat,1) ~= 4 || size(quat,2) ~= 1 )
        error('Input quaternion must be 4x1');
    end
    
    vector = quat(2:4);
    scalar = quat(1);
    
    qLC = [  scalar ,  -vector';
             vector , scalar*eye(3) + crossMat(vector)  ];
end