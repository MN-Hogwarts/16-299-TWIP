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
# 
