function nominalState_prop = propagateNominalState(imuNominalStates_k,measurement_k,measurement_k_1)
    % quaternion state
    det_theta = ((measurement_k.omega+measurement_k_1.omega)/2 - imuNominalStates_k.wb)* measurement_k.dt;
    %intergrade q with approximate form , this two equation have the same result.
    det_q = [1;0.5*det_theta];
    nominalState_prop.q = quatLeftComp(imuNominalStates_k.q)*det_q;
    %nominalState_prop.q =imuNominalStates_k.q + 0.5 * omegaMat(det_theta) * imuNominalStates_k.q; 
    
%     %intergrade q with close form 
%     det_THETA = [0             -det_theta(1)        -det_theta(2)        -det_theta(3)
%                  det_theta(1)       0                det_theta(3)        -det_theta(2)
%                  det_theta(2)  -det_theta(3)             0                det_theta(1)
%                  det_theta(3)   det_theta(2)        -det_theta(1)           0];
%     norm_theta = norm(det_theta);
%     nominalState_prop.q = (cos(norm_theta/2)*eye(4)+sin(norm_theta/2)/norm_theta*det_THETA)*imuNominalStates_k.q;
    %Unit length quaternion
    nominalState_prop.q =  nominalState_prop.q/norm( nominalState_prop.q);
    % Bias states
    nominalState_prop.wb = imuNominalStates_k.wb;
end