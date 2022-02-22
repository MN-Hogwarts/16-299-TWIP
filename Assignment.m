xm_w = 1; % kg
m_p = 1; % kg
l_p = 1; % m
I_p = 0.1;
I_w = 0.1*I_p;
g = 9.81; % m/s^2
r_w = 0.2; % m

%Derive State Space Matrices A, B (continuous)
syms th x u thd xd

m_w = 1; % kg
m_p = 1; % kg
l_p = 1; % m
I_p = 0.1;
I_w = 0.1*I_p;
g = 9.81; % m/s^2
r_w = 0.2; % m

M = [ (I_w/(r_w^2) + m_p + m_w)  l_p*m_p*cos(th)
      l_p*m_p*cos(th)            (I_p + l_p^2*m_p) ];
v = [ (r_w*(-u + thd^2*l_p*m_p*r_w*sin(th))); (u + g*l_p*m_p*sin(th)) ];
K = M\v;
X = [x; xd; th; thd; u];
J = jacobian(K, X); %[dx, dxd, dth, dthd, du]

A_sym = [0, 1, 0, 0;...
     J(1,1:4);...
     0, 0, 0, 1;...
     J(2,1:4)];
B_sym = [0; J(1,5); 0; J(2,5)];
A = double(subs(A_sym, {th, thd}, {0, 0}))
B = double(subs(B_sym, {th, thd}, {0, 0}))

%Use LQR to find poles and optimal control gains K_ctrl
figure(99)
hold on
for i=-4:4
    mag = 10^i;
    Q = mag.*diag([1, 0, 1, 0]);
    R = mag.*[1];
    [K, S, P] = lqr(A, B, Q, R);
    plot(P, 'o')
    if i == 0
        K_ctrl = K;
        poles = P;
    end
end
hold off

K_ctrl
poles

%Establish discrete state space matrices Ad, Bd and use to find optimal
%Kalman filter K_KF. Also establsih output matrix C and noise.
Q_KF = diag([1,0.00001,1,0.00001]);
R_KF = 1;
C = [1, 0, 0, 0; 0, 0, 1, 0];
dt = 0.01;
Ad = A*dt+eye(4);
K_KF = dlqr(Ad', C', Q_KF, R_KF)'
snr = 30 %dB
