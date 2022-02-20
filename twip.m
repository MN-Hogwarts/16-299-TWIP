function [xdd thdd] = twip( th, thd, trq,  m_w, r_w, I_w, m_p, l_p, I_p, g )
% compute a two wheeled inverted pendulum's forward dynamics
%  Compute the "mass" matrix:
M = [ (I_w/(r_w^2) + m_p + m_w)  l_p*m_p*cos(th)
      l_p*m_p*cos(th)            (I_p + l_p^2*m_p) ];
Minv = inv(M);
v = [ (r_w*(-trq + thd^2*l_p*m_p*r_w*sin(th))) (trq + g*l_p*m_p*sin(th)) ];
rhs = transpose( v );
result = Minv*rhs;
xdd = result(1);
thdd = result(2);
end
