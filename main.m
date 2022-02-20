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
u_array = zeros(N,1); % commands
a_array = zeros(N,2); % acceleration

% initial conditions
xx = x0;

init_plots( xx, N, r_w, l_p );

aa = [ 0 0 ];

% simulation
for i = 1:N
  x_array(i,:) = transpose( xx );
 uu = -k*xx;
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
plot(1:N,x_array(:,1));
title( 'x' )

figure(3)
plot(1:N,x_array(:,3));
title( 'angle' )

figure(4)
plot(1:N,x_array(:,2));
title( 'forward velocity' )

figure(5)
plot(1:N,x_array(:,4));
title( 'angular velocity' )

figure(6)
plot(1:N,u_array(:,1));
title( 'torque' )

figure(1)

% To zoom in on a plot
% axis([0 10000 -0.01 0.01])
