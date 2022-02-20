function plot_it( xx, i, wheel_radius, length );
%

global l_pendulum;
global l_wheel;

l_pendulum.XData = [xx(1) length*sin(xx(2))+xx(1)];
l_pendulum.YData = [wheel_radius wheel_radius+length*cos(xx(2))];
l_wheel.XData = [xx(1) xx(1)];
drawnow;
  
