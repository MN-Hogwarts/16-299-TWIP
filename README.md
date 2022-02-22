# 16-299-TWIP
Step 1: run init.m

Step 2: run Assignment.m (This will establish A, B, C, Ad, Bd, K_ctrl, K_KF, Q, R, Q_KF, R_KF, and SNR for observer noise)
        
Step 3: run main.m (modified ours to use state = [x, xd, th, thd] everywhere)

# A
What are your best manual gains?
- K_ctrl = [1, 5, 50, 10]
#
# B
What are your A and B matrices?
- After running Assignment these matrices are outputted. Enter A or B into the command window to view again
Which of the symbolic terms in the dynamics in the M(x)a = v(x) equation could you have ignored?
- It's assumed that there's no friction in this system, so the xdot terms could be ignored
#
# C1
What is your Q, R and corresponding K?
- After running Assignment these matrices are outputted. Enter Q, R, K_ctrl to view again
- We chose Q to have ones at x and th and R to be one because the design criteria was to minimize (xx+thth+tautau) so all should be weighted equally
# C2
See figure 1 after running Assignment

The Q and R components matter only when the corresponding variables in the state vector are observed (on from C)

Need one component for each independent variable for pole location control
#
# D
Switch the function called here between noKalman, samedtKalman, and fastdtKalman to run without the Kalman filter or either of the two different types of Kalman filters: https://github.com/MN-Hogwarts/16-299-TWIP/blob/38add8c566b2d4cf450fa157ca84f55d31a3ed80/main.m#L48

What is the difference in command response between this KF+LQR combination, and the same LQR design with full state feedback?
- With the KF+LQR combination, the commands look jerkier and do not get the system to its goal as smoothly. This makes sense since the system no longer has as much information and noise has been introduced into the system.
