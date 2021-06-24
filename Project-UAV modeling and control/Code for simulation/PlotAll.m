
figure;
subplot(3,1,1);plot(Time,Phi_array,Time,Phi_ref_array);title('\phi vs reference');grid on;
subplot(3,1,2);plot(Time,Theta_array,Time,Theta_ref_array);title('\theta vs reference');grid on;
subplot(3,1,3);plot(Time,Psi_array,Time,Psi_ref_array);title('\psi vs reference');grid on;
figure;
subplot(3,1,1);plot(Time,Phi_est_array,Time,Phi_ref_array);title('\phi est vs reference');grid on;
subplot(3,1,2);plot(Time,Theta_est_array,Time,Theta_ref_array);title('\theta est vs reference');grid on;
subplot(3,1,3);plot(Time,Psi_est_array,Time,Psi_ref_array);title('\psi est vs reference');grid on;
figure
subplot(3,1,1);plot(Time,Phi_ref_array - Phi_array);title('\phi error');grid on;
subplot(3,1,2);plot(Time,Theta_ref_array - Theta_array);title('\theta error');grid on;
subplot(3,1,3);plot(Time,Psi_ref_array - Psi_array);title('\psi error');grid on;
figure
subplot(3,1,1);plot(Time,Phi_error_integer_array);title('\phi error integrated');grid on;
subplot(3,1,2);plot(Time,Theta_error_integer_array);title('\theta error integrated');grid on;
subplot(3,1,3);plot(Time,Psi_error_integer_array);title('\psi error integrated');grid on;
figure;
plot3(x_array,y_array,-z_array);title('Trajectory');grid on;
figure;
subplot(3,1,1);plot(Time,Phi_array,Time,Phi_est_array);title('\phi est');grid on;
subplot(3,1,2);plot(Time,Theta_array,Time,Theta_est_array);title('\theta est');grid on;
subplot(3,1,3);plot(Time,Psi_array,Time,Psi_est_array);title('\Psi est');grid on;

% temp = Psi_array - Psi_est_array;
% for i=1:1:idata
%     if(temp(i)>pi)
%         temp(i) = temp(i) - 2*pi;
%     elseif(temp(i)<-pi)
%         temp(i) = temp(i) + 2*pi;
%     end
% end
tempstr = ['Max x error: ',num2str(max(abs(x_array-x_est_array))),'   Average error: ',num2str(mean(abs(x_array-x_est_array)))];disp(tempstr)
tempstr = ['Max y error: ',num2str(max(abs(y_array-y_est_array))),'   Average error: ',num2str(mean(abs(y_array-y_est_array)))];disp(tempstr)
tempstr = ['Max z error: ',num2str(max(abs(z_array-z_est_array))),'   Average error: ',num2str(mean(abs(z_array-z_est_array)))];disp(tempstr)
tempstr = ['Max vx error: ',num2str(max(abs(x_dot_array-x_dot_est_array))),'   Average error: ',num2str(mean(abs(x_dot_array-x_dot_est_array)))];disp(tempstr)
tempstr = ['Max vy error: ',num2str(max(abs(y_dot_array-y_dot_est_array))),'   Average error: ',num2str(mean(abs(y_dot_array-y_dot_est_array)))];disp(tempstr)
tempstr = ['Max vz error: ',num2str(max(abs(z_dot_array-z_dot_est_array))),'   Average error: ',num2str(mean(abs(z_dot_array-z_dot_est_array)))];disp(tempstr)