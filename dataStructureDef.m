time    = zeros(length,1);
roll    = zeros(length,1);
pitch   = zeros(length,1);
yaw     = zeros(length,1);

% IMU states as Structures indexed in a cell array
%imuNominalStates
% imuNominalStates{k}.q         4x1 Global to IMU rotation quaternion
% imuNominalStates{k}.wb        3x1 Gyro bias
imuNominalStates = cell(1,length);

%imuErrorStates
%imuErrorStates{k}.det_theta    3x1 det theta
%imuErrorStates{k}.det_wb       3x1 det Gyro bias
%imuErrorStates{k}.P
imuErrorStates   = cell(1,length);

% Measurements as structures all indexed in a cell array
%dT = [0, diff(t)];
%measurements
%measurements{k}.acc            3x1 acc measurements
%measurements{k}.omega          3x1 gyro measurements
%measurements{k}.mag            3x1 mag measurements
%measurements{k}.dt
measurements = cell(1,length);
%Ground Truth
%groundTruthStates{k}.autitude
for state_k = 1:length 
    measurements{state_k}.dt    = 0.02;                      % sampling times 50Hz
    measurements{state_k}.omega = Data(state_k,27:29)';            
    measurements{state_k}.acc   = Data(state_k,9:11)';
    measurements{state_k}.mag   = Data(state_k,15:17)';
    time(state_k)=state_k*0.02;
end
rad2deg = 180/pi;
rollRef   = Data(:,30)*rad2deg;
pitchRef  = Data(:,31)*rad2deg;
yawRef    = Data(:,32)*rad2deg;