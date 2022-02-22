% Simulate a two wheeled inverted pendulum (Segway-like)

global first_time

if first_time
 m_w = 1; % kg
 m_p = 1; % kg
 l_p = 1; % m
 I_p = 0.1;
 I_w = 0.1*I_p;
 g = 9.81; % m/s^2
 r_w = 0.2; % m
 k = [ 0 0 0 0 ];
 goal = [ 0 0 0 0 ];
 samples_per_second = 100;
 duration = 10.0;
% initial state
 x0 = transpose( [ 0 0.1 0 0 ] );
 first_time = 0;
end

N = duration*samples_per_second
dt = duration/N

score = 0;

% set up arrays
x_array = zeros(N,4); % states
x_est_array = zeros(N,4);
u_array = zeros(N,1); % commands
a_array = zeros(N,2); % acceleration
C = [1,0,0,0;0,0,1,0];

% initial conditions
xx = x0;
x_est = x0;
yy = [x0(1); x0(3)];
uu = 0;

init_plots( xx, N, r_w, l_p );

aa = [ 0 0 ];

% simulation
for i = 1:N
  x_array(i,:) = transpose( xx );
  x_est_array(i,:) = transpose(x_est);
  [uu, x_est] = samedtKalman(xx, K_ctrl, x_est, uu, A, B, C, snr, dt, K_KF);
  u_array(i,1) = uu;
 [xdd, thdd] = twip( xx(3), xx(4), uu, m_w, r_w, I_w, m_p, l_p, I_p, g );
 aa = [xdd thdd];
  a_array(i,:) = transpose( aa );
 if ( i < N )
   for j = 1:2
     vv_new(j) = xx(2*j) + aa(j)*dt;
     xx(2*j-1) = xx(2*j-1) + 0.5*(vv_new(j) + xx(2*j))*dt;
     xx(2*j) = vv_new(j);
   end
  % apply one step cost here.
  % score = score + dt*((xx(i) - goal)*(xx(i) - goal) + 0.05*uu(i)*uu(i));
 else
  % apply terminal state penalty here.
  % score = score + dt*((xx(i) - goal)*(xx(i) - goal));
 end
 % if ( rem( i, 2 ) == 0 )
   plot_it( xx, i, r_w, l_p );
 % end
 if ( rem( i, samples_per_second ) == 0 )
   i
 end
end

figure(2)
hold on
plot(1:N,x_array(:,1));
plot(1:N, x_est_array(:,1), 'r--')
title( 'x' )
hold off

figure(3)
hold on
plot(1:N,x_array(:,3));
plot(1:N, x_est_array(:,3), 'r--')
title( 'angle' )
hold off

figure(4)
hold on
plot(1:N,x_array(:,2));
plot(1:N, x_est_array(:,2), 'r--')
title( 'forward velocity' )
hold off

figure(5)
hold on
plot(1:N,x_array(:,4));
plot(1:N, x_est_array(:,4), 'r--')
title( 'angular velocity' )
hold off

figure(6)
plot(1:N,u_array(:,1));
title( 'torque' )

figure(1)

% To zoom in on a plot
% axis([0 10000 -0.01 0.01])

function [uu, x_est] = noKalman(xx, K_ctrl, ~, ~, ~, ~, ~, ~, ~, ~)
    uu = -K_ctrl*xx;
    %disp(K_ctrl);
    x_est = xx;
end

function [uu, x_est] = samedtKalman(xx, K_ctrl, x_est, uu, A, B, C, snr, dt, K_KF)
    yy = C*xx;
    yy = awgn(yy, snr); %add noise to observer
    Ad = A*dt+eye(4);
    Bd = B*dt;
    x_est = Ad*x_est+Bd*uu-K_KF*(C*x_est-yy);
    uu = -K_ctrl*x_est;
end

function [uu, x_est] = fastdtKalman(xx, K_ctrl, x_est, uu, A, B, C, snr, dt, K_KF)
    dt_kalman = dt/5; % Faster
    yy = C*xx;
    yy = awgn(yy, snr); %add noise to observer
    Ad = A*dt_kalman+eye(4);
    Bd = B*dt_kalman;
    x_est = Ad*x_est+Bd*uu-K_KF*(C*x_est-yy);
    uu = -K_ctrl*x_est;
end
