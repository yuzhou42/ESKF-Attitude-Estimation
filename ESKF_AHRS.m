clear;
close all;
clc;
addpath('utils');
tic
%% ==========================Input Data======================== %%
dataDir = '../datasets';
fileName = 'NAV_1'; 
Data = importdata(sprintf('%s/%s.mat',dataDir,fileName));
length = size(Data,1);
dataStructureDef;
%% ==========================Initial State======================== %%
%get init euler angle and quaternion
[qInit,roll(1),pitch(1),yaw(1)] = initializequat(Data);
%Set up the noise parameters
wn_var  = 1e-5 * ones(1,3);               % rot vel var
wbn_var = 1e-9* ones(1,3);                % gyro bias change var
an_var  = 1e-3 * ones(1,3);               % acc var
mn_var  = 1e-4 * ones(1,3);               % mag var
noiseParams.Q = diag([wn_var, wbn_var]); 
noiseParams.R = diag([an_var, mn_var]); 
%set uo P initial value
q_var_init = 1e-5 * ones(1,3);            % init rot var
wb_var_init = 1e-7 * ones(1,3);           % init gyro bias var
imuErrorStates{1}.P = diag([q_var_init, wb_var_init]);
%Use ground truth for the first state
imuNominalStates{1}.q       = qInit;
imuNominalStates{1}.wb      = wbn_var';
%set up the reset value of error state
detThetaReset = zeros(3,1);             
imuErrorStates{1}.det_theta = detThetaReset;
imuErrorStates{1}.det_wb    = detThetaReset; 
%MAIN LOOP
for state_k = 1:length-1
    %% ==========================STATE PROPAGATION======================== %%
    %Propagate nominal state 
    imuNominalStates{state_k+1}  = propagateNominalState(imuNominalStates{state_k},measurements{state_k},measurements{state_k+1});
    %Propagate error state and covariance
    imuErrorStates{state_k+1}    = propagateErrorStateAndCovar(imuNominalStates{state_k}.wb,imuErrorStates{state_k}, measurements{state_k}, noiseParams); 
    %% ==========================FILTER UPDATE======================== %%
    %Calculate H and detZ = y-h(det_x)
    [H,detZ] = calH(imuNominalStates{state_k+1}.q,  measurements{state_k+1});
    P = imuErrorStates{state_k+1}.P;
    % Calculate Kalman gain
    K = (P*H') / ( H*P*H' + noiseParams.R); 
    % State correction
    det_x =  K * detZ;
    imuErrorStates{state_k+1}.det_theta = det_x(1:3);
    imuErrorStates{state_k+1}.det_wb    = det_x(4:6);
    % Covariance correction
    imuErrorStates{state_k+1}.P = P -  K *(H*P*H'+ noiseParams.R)*K';
  %% ==========================STATE CORRECTION======================== %%
    det_q = buildUpdateQuat(imuErrorStates{state_k+1}.det_theta);                                       %it seems more safety to use this to update the euation q=[1 1/2*det_theta]
    imuNominalStates{state_k+1}.q  = quatLeftComp(imuNominalStates{state_k+1}.q)*det_q;                 %joan sola_Quaternion kinematics for the error_state KF p24
    imuNominalStates{state_k+1}.q  = imuNominalStates{state_k+1}.q /norm(imuNominalStates{state_k+1}.q ); 
    imuNominalStates{state_k+1}.wb = imuNominalStates{state_k+1}.wb + imuErrorStates{state_k+1}.det_wb;
    [roll(state_k+1),pitch(state_k+1),yaw(state_k+1)] = quattoeuler(imuNominalStates{state_k+1}.q);     %transform the quaternion to euler for plot
    %% ==========================ERROR STATE RESET======================== %%
    imuErrorStates{state_k+1}.det_theta = detThetaReset;
    imuErrorStates{state_k+1}.det_wb    = detThetaReset;
    imuErrorStates{state_k+1}.P         = eye(6)*imuErrorStates{state_k+1}.P*eye(6)';
%     err_sigma(:,state_k) = sqrt(diag(imuErrorStates{state_k+1}.P));
end 
% err_sigma(:,length)=err_sigma(:,end);
toc
plotFigure


