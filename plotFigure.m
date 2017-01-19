 %% ==========================plot======================== %%
figure('Name', 'Roll Angle');
hold on;
plot(time, roll, '--r',time,rollRef,'-g');
% title('Roll Angle');
xlabel('Time (s)');
ylabel('Roll(deg)');
legend('横滚角估计值','参考真值');
hold off;

figure('Name', 'Pitch Angle');
hold on;
plot(time, pitch, '--r',time,pitchRef,'-g');
% title('Pitch Angle');
xlabel('Time (s)');
ylabel('Pitch(deg)');
legend( '俯仰角估计值','参考真值');
hold off;

figure('Name', 'Yaw Angle');
hold on;
plot(time, yaw, '--r',time,yawRef,'-g');
% title('Yaw Angle');
xlabel('Time (s)');
ylabel('Yaw(deg)');
legend('偏航角估计值','参考真值');
hold off;

 %% ==========================plot error======================== %%
 rotLim = [-5 5];
  rotLimYaw = [-5 5];
% Rotation Errors
figure
subplot(3,1,1)
rollError = roll-rollRef;
plot(time, 0.5*rollError, '-r','LineWidth', 1)
% hold on
% plot(time, 3*err_sigma(1,:), '--k')
% plot(time, -3*err_sigma(1,:), '--k')
ylim(rotLim)
xlim([time(1) time(end)])
ylabel('roll error')

subplot(3,1,2)
pitchError = pitch-pitchRef;
plot(time, 0.5*pitchError, '-r','LineWidth', 1)
% hold on
% plot(time, 3*err_sigma(2,:), '--k')
% plot(time, -3*err_sigma(2,:), '--k')
ylim(rotLim)
xlim([time(1) time(end)])
ylabel('pitch error')

subplot(3,1,3)
yawError = yaw-yawRef;
plot(time, 0.5*yawError, '-r','LineWidth', 1)
% hold on
% plot(time, 3*err_sigma(3,:), '--k')
% plot(time, -3*err_sigma(3,:), '--k')
ylim(rotLimYaw)
xlim([time(1) time(end)])
ylabel('yaw error')
