th0 = 45*pi/180;
x0 = 0;
y0 = 0;

th0_est = 35*pi/180;

dT = 0.1;
t = (0:dT:100)';
gyroz = t*0;
gyroz(10/dT:12/dT) = 5*pi/180;
gyroz(30/dT:33/dT) = -5*pi/180;

enc = t*0+0.1;
enc(1) = 0;

x = t*0; y=t*0; th=t*0;
x(1) = x0;
y(1) = y0;
th(1) = th0;

x_est=t*0; y_est=t*0; th_est=t*0;
x_est(1) = x0;
y_est(1) = y0;
th_est(1) = th0_est;

gyroz_meas = t*0;
enc_meas = t*0;

x_gps = x;
y_gps = y;

% Q_enc = %(perc_error*enc_meas(k))^2, done in loop
% Q_th = 0 if enc_meas = 0 ?
sig_th = 0.1*pi/180;
enc_perc_error = 0.05;
Q = diag([0 0 (sig_th*dT)^2 0]);

R = diag([100 100]);

n = length(t);
P_out = zeros(n,4);
P = diag([1 1 (10*pi/180)^2 0.0]);
P_out(1,:) = [P(1,1),P(2,2),P(3,3),P(4,4)];

H_gps = [1 0 0 0;
         0 1 0 0];

for k = 2:n
  th(k) = th(k-1) + gyroz(k)*dT;
  x(k) = x(k-1) + enc(k)*cos(th(k));
  y(k) = y(k-1) + enc(k)*sin(th(k));
  
  x_gps(k) = x(k) + (2*rand(1)-1)*10;
  y_gps(k) = y(k) + (2*rand(1)-1)*10;

  gyroz_meas(k) = gyroz(k) + (2*rand(1)-1)*0.5*pi/180;
  th_est(k) = th_est(k-1) + gyroz_meas(k)*dT;
  
  enc_meas(k) = enc(k)*(1+(2*rand(1)-1)*0.1);
  x_est(k) = x_est(k-1) + enc_meas(k)*cos(th_est(k));
  y_est(k) = y_est(k-1) + enc_meas(k)*sin(th_est(k));
  
  Q(4,4) = (enc_perc_error*enc_meas(k))^2;
  % Model Update
  A = [1 0 -enc_meas(k)*sin(th_est(k)) cos(th_est(k));
       0 1  enc_meas(k)*cos(th_est(k)) sin(th_est(k));
       0 0 1 0;
       0 0 0 0];
       
  P = A*P*A' + Q;
  
  % GPS Measurement Update
  if(mod(k,round(5/dT)) == 0) % every 10 seconds
    if(abs(x_gps(k) - x_est(k)) < 3*sqrt(P(1,1))...
        && abs(y_gps(k) - y_est(k)) < 3*sqrt(P(2,2)) )
      S = H_gps*P*H_gps' + R;
      K = P*H_gps'*inv(S);
      state = [x_est(k) y_est(k) th_est(k) enc_meas(k)]';
      state_err = [x_gps(k); y_gps(k)] - H_gps*state;
      state = state + K*state_err;
      x_est(k) = state(1); y_est(k) = state(2);
      th_est(k) = state(3); enc_meas(k) = state(4);
      P = (eye(4)-K*H_gps)*P;
    end
  end
  P_out(k,:) = [P(1,1) P(2,2) P(3,3) P(4,4)];
  
end

figure(1)
hold off
plot(x,y)
hold on
plot(x_est,y_est,'r')
plot(x_gps(5:5:end),y_gps(5:5:end),'gx')
legend('true','est','gps')
grid on
axis equal

figure(2)
% x
subplot(4,1,1); hold off
plot(t,x_est); hold on
plot(t,x_est + 2*sqrt(P_out(:,1)),'m--')
plot(t,x_est - 2*sqrt(P_out(:,1)),'m--')
ylabel('x (m)')
grid on
% y
subplot(4,1,2); hold off
plot(t,y_est); hold on
plot(t,y_est + 2*sqrt(P_out(:,2)),'m--')
plot(t,y_est - 2*sqrt(P_out(:,2)),'m--')
ylabel('y (m)')
grid on
% th
subplot(4,1,3); hold off
plot(t,th_est*180/pi); hold on
plot(t,(th_est + 2*sqrt(P_out(:,3)))*180/pi,'m--')
plot(t,(th_est - 2*sqrt(P_out(:,3)))*180/pi,'m--')
ylabel('theta est (deg)')
grid on
% enc
subplot(4,1,4); hold off
plot(t,enc_meas); hold on
plot(t,enc_meas + 2*sqrt(P_out(:,4)),'m--')
plot(t,enc_meas - 2*sqrt(P_out(:,4)),'m--')
ylabel('delta enc meas (m)')
xlabel('time (sec)')
grid on