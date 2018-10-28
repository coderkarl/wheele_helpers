gps_updates_enabled = true;
compass_updates_enabled = true;

% N0, E0
init_lat_lon;

%data = csvread('~/wheele_bags/wheele_raw_gps_20Jan2018/2018-01-20-17-03-39.csv');
%data = csvread('~/wheele_bags/wheele_raw_gps_20Jan2018/2018-01-20-16-52-37.csv');
%val_ind = find(gps_status>0)(1)
%N0 = N0 - 7/deg2meters;
%E0 = E0 + 5/deg2meters/cos(N0*3.14159/180);

%data = csvread('~/wheele_bags/wheele_add_compass_GPS_freeze_02Feb2018/2018-02-02-17-54-48.csv');
%data = csvread('~/wheele_bags/wheele_add_compass_GPS_freeze_02Feb2018/2018-02-02-17-50-02.csv');
data = csvread('~/wheele_bags/wheele_add_compass_GPS_freeze_02Feb2018/2018-02-02-17-26-11.csv');

%data = csvread('~/wheele_bags/wheele_more_speed_03Feb2018/2018-02-03-17-31-59.csv');
%data = csvread('~/wheele_bags/wheele_more_speed_03Feb2018/2018-02-03-17-42-00.csv');

t = data(:,1);
enc_meas_raw = (data(:,3) + data(:,4) )/2;
gyroz_degx100 = data(:,5);
gps_status = data(:,7);
gps_lon = data(:,9);
gps_lat = data(:,10);

compass_status = data(:,14);
heading_degx100 = data(:,15);

COUNTS_PER_METER = 900;

% Past Data Sets start/end rel to N0, E0
% raw_gps_20Jan2018, no compass
%  16-14-12, start 10 m South, 3 m East (end=start)
%  16-26-59, start 10 m South, 3 m East (end 4m S, 1m E), steer failure
%  16-52-37, start 10 m South, 7 m East (end ??)
%  17-03-39, start 10 m South, 15 m East (end 7m S, 10m E)

% gps_and_leddar_filtered_26Jan2018, no compass
%  15-12-53, start 0, 0 (end=start)
%  15-37-52, start 0, 0 (end=start)
%  15-47-57, start 10m S, 6m E (end=start)
%  15-55-44  start 10m S, 6m E (end=start)

COMPASS_OFFSET_DEG = 180;
heading_meas = (heading_degx100/100 + COMPASS_OFFSET_DEG)*pi/180;
compass_status_count = 0;
ck = 0;
heading_plot = t*0;
heading_time = t*0;
H_compass = [0 0 1 0];
R_compass = 5*pi/180;


deg2meters = 111111.11;
lat_factor = cos(N0*3.14159/180.0);

x0_est = 0;
y0_est = 0;
th0_est = 0*pi/180;

x_est=t*0; y_est=t*0; th_est=t*0;
x_est(1) = x0_est;
y_est(1) = y0_est;
th_est(1) = th0_est;

gyroz_meas = gyroz_degx100/100 * 360/375 * pi/180;
enc_meas = enc_meas_raw / COUNTS_PER_METER;

x_gps = (gps_lon - E0)*deg2meters*lat_factor; % update lat_factor dynamically?
y_gps = (gps_lat - N0)*deg2meters;
gps_status_count = 0;
gk = 0;
x_gps_plot = t*0;
y_gps_plot = t*0;
gps_lon_plt = t*0;
gps_lat_plt = t*0;

% Q_enc = %(perc_error*enc_meas(k))^2, done in loop
% Q_th = 0 if enc_meas = 0 ?
sig_th = 1*pi/180; %rad/sec
%enc_perc_error = 0.05; %Why is this not working?
enc_error_factor = 0.02;
dT = mean(diff(t));
%Q = diag([0 0 (sig_th*dT)^2 0]); %Why is this not working as expected? Units? dT?
Q = diag([0 0 3e-5 0]);

R = diag([300 300]);

n = length(t);
P_out = zeros(n,4);
% INITIAL ESTIMATE OF STATE ERROR (1sigma) IN P
th_error_est = 3*pi/180;
x_error_est = 1;
y_error_est = 1;
P = diag([x_error_est^2 y_error_est^2 th_error_est^2 0.0]);
P_out(1,:) = [P(1,1),P(2,2),P(3,3),P(4,4)];

H_gps = [1 0 0 0;
         0 1 0 0];

for k = 2:n
  dT = t(k) - t(k-1);
  th_est(k) = th_est(k-1) + gyroz_meas(k)*dT;
  if(th_est(k) >= pi)
    th_est(k) = th_est(k) - 2*pi;
  elseif(th_est(k) < -pi)
    th_est(k) = th_est(k) + 2*pi;
  end
  
  x_est(k) = x_est(k-1) + enc_meas(k)*cos(th_est(k));
  y_est(k) = y_est(k-1) + enc_meas(k)*sin(th_est(k));
  
  %Q(4,4) = (enc_perc_error*enc_meas(k))^2; %Why is this not working?
  Q(4,4) = abs( enc_error_factor*enc_meas(k) );
  % Model Update
  A = [1 0 -enc_meas(k)*sin(th_est(k)) cos(th_est(k));
       0 1  enc_meas(k)*cos(th_est(k)) sin(th_est(k));
       0 0 1 0;
       0 0 0 0];
       
  P = A*P*A' + Q;
  
  % Compass Measurement update
  if(compass_status(k) > 0)
    compass_status_count = compass_status_count + 1;
    if(heading_meas(k) >= pi)
      heading_meas(k) = heading_meas(k) - 2*pi;
    elseif(heading_meas(k) < -pi)
      heading_meas(k) = heading_meas(k) + 2*pi;
    end
    
    heading_diff = heading_meas(k) - th_est(k);
    if(heading_diff > 200)
      heading_meas(k) = heading_meas(k) - 360;
    elseif(heading_diff < -200)
      heading_meas(k) = heading_meas(k) + 360;
    end
    heading_diff = heading_meas(k) - th_est(k);
    
    if(compass_status_count >= 3)
      compass_status_count = 0;
      ck = ck+1;
      heading_plot(ck) = heading_meas(k);
      heading_time(ck) = t(k);
    
      if(abs(heading_diff) < 3*sqrt(P(3,3)) && compass_updates_enabled)
        S = H_compass*P*H_compass' + R_compass;
        K = P*H_compass'*inv(S);
        state = [x_est(k) y_est(k) th_est(k) enc_meas(k)]';
        state_err = heading_diff;
        state = state + K*state_err;
        x_est(k) = state(1); y_est(k) = state(2);
        th_est(k) = state(3); enc_meas(k) = state(4);
        P = (eye(4)-K*H_compass)*P;
      
        if(th_est(k) >= pi)
          th_est(k) = th_est(k) - 2*pi;
        elseif(th_est(k) < -pi)
          th_est(k) = th_est(k) + 2*pi;
        end
      end
      
    end
  end
  
  % GPS Measurement Update
  if(gps_status(k) > 0)
    gps_status_count = gps_status_count + 1;
    if(gps_status_count >= 5) % every 5 gps measurements
      gps_status_count = 0;
      gk = gk+1;
      x_gps_plot(gk) = x_gps(k);
      y_gps_plot(gk) = y_gps(k);
      gps_lon_plot(gk) = gps_lon(k);
      gps_lat_plot(gk) = gps_lat(k);
      if(abs(x_gps(k) - x_est(k)) < 3*sqrt(P(1,1))...
          && abs(y_gps(k) - y_est(k)) < 3*sqrt(P(2,2))
          && gps_updates_enabled)
        S = H_gps*P*H_gps' + R;
        K = P*H_gps'*inv(S);
        state = [x_est(k) y_est(k) th_est(k) enc_meas(k)]';
        state_err = [x_gps(k); y_gps(k)] - H_gps*state;
        state = state + K*state_err;
        x_est(k) = state(1); y_est(k) = state(2);
        th_est(k) = state(3); enc_meas(k) = state(4);
        P = (eye(4)-K*H_gps)*P;
        
        if(th_est(k) >= pi)
          th_est(k) = th_est(k) - 2*pi;
        elseif(th_est(k) < -pi)
          th_est(k) = th_est(k) + 2*pi;
        end
      end
    end
  end
  P_out(k,:) = [P(1,1) P(2,2) P(3,3) P(4,4)];
  
end

x_gps_plot = x_gps_plot(1:gk);
y_gps_plot = y_gps_plot(1:gk);

heading_plot = heading_plot(1:ck);
heading_time = heading_time(1:ck);

figure(1)
hold off
plot(x_est,y_est,'r')
hold on
plot(x_gps_plot, y_gps_plot,'gx')
legend('est','gps')
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
plot(heading_time, heading_plot*180/pi,'g')
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

%figure(3)
%plot(t,sqrt(P_out(:,2)))
