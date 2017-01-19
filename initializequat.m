function [q,roll,pitch,yaw] = initializequat(Data)
ax=Data(:,9);  %acc data
ay=Data(:,10); 
az=Data(:,11); 
mx=Data(:,15); %%mag data
my=Data(:,16); 
mz=Data(:,17); 
rad2deg=180/pi;
%% Process sensor data through algorithm
Pitch0=asin(ax(1)/sqrt(ax(1)*ax(1)+ay(1)*ay(1)+az(1)*az(1)));
Roll0=atan2(-ay(1),-az(1));
Yaw0=atan2(-my(1)*cos(Roll0)+mz(1)*sin(Roll0),mx(1)*cos(Pitch0)+my(1)*sin(Pitch0)*sin(Roll0)+mz(1)*sin(Pitch0)*cos(Roll0))-8.3*pi/180;
[q0,q1,q2,q3]=quatfromeuler(Roll0,Pitch0,Yaw0);
q     = [q0;q1;q2;q3];
q     = q/norm(q);
pitch = Pitch0*rad2deg; 
roll  = Roll0*rad2deg;
yaw   = Yaw0*rad2deg;
end