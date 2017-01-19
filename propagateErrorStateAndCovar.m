function [errorState_prop] = propagateErrorStateAndCovar(wb,imuErrorStates_k, measurements_k, noiseParams)   
    %Propagate State
    Fx = calcFx( wb, measurements_k);
    det_x = [imuErrorStates_k.det_theta;imuErrorStates_k.det_wb];
    det_x_prop = Fx*det_x;                         %this propagate always be zero
    errorState_prop.det_theta = det_x_prop(1:3);
    errorState_prop.det_wb    = det_x_prop(4:6);
    %Propagate Convar
    Qi = noiseParams.Q*measurements_k.dt;
    Fi = eye(6);
    errorState_prop.P = Fx *  imuErrorStates_k.P * Fx' + Fi * Qi * Fi'; 
    errorState_prop.P = enforcePSD(errorState_prop.P);%this is important,it removes the NAN problem successfully~
end