function [H,detZ] = calH(q,measurements_k)
    % Normalise magnetometer measurement
    if(norm(measurements_k.mag) == 0), return; end	% 
    measurements_k.mag = measurements_k.mag / norm(measurements_k.mag);	% normalise magnitude,very important!!!!
    % Normalise accelerometer measurement
    if(norm(measurements_k.acc) == 0), return; end	% handle NaN
    measurements_k.acc  = measurements_k.acc / norm(measurements_k.acc);	% normalise accelerometer ,very important!!!!
    % Reference direction of Earth's magnetic feild
    h = quaternProd(q, quaternProd([0; measurements_k.mag], quatInv(q)));
    b = [0 norm([h(2) h(3)]) 0 h(4)];
    Ha = [2*q(3),                 	-2*q(4),                    2*q(1),                         -2*q(2)
         -2*q(2),                 	-2*q(1),                   -2*q(4),                         -2*q(3)
          0,                         4*q(2),                    4*q(3),                         0];
    Hm = [-2*b(4)*q(3),                2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
          -2*b(2)*q(4)+2*b(4)*q(2),	   2*b(2)*q(3)+2*b(4)*q(1),    2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
           2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	   2*b(2)*q(1)-4*b(4)*q(3),        2*b(2)*q(2)];
    Hx = [Ha, zeros(3,3)
          Hm, zeros(3,3)];
    %Hx = [Ha, zeros(3,3)];
    Q_detTheta  = [-q(2),    -q(3),      -q(4)
                    q(1),    -q(4),       q(3) 
                    q(4),     q(1),      -q(2) 
                   -q(3),     q(2),       q(1)];
    Xx = [0.5*Q_detTheta , zeros(4,3)
          zeros(3)       , eye(3)];
    H = Hx*Xx; 
    
    detZ_a = [ 2*(q(2)*q(4)  - q(1)*q(3)) + measurements_k.acc(1)
               2*(q(1)*q(2) + q(3)*q(4)) + measurements_k.acc(2)
               2*(0.5 - q(2)^2 - q(3)^2) + measurements_k.acc(3)];
%     detZ   = detZ_a;
    detZ_m =[((2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3))) + measurements_k.mag(1))
             ((2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4))) + measurements_k.mag(2))
             ((2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)) + measurements_k.mag(3))]; 
    detZ   = [detZ_a;detZ_m];
end