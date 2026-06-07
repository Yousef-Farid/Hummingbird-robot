clc
clear all
close all
format short
%% Dynamic Parameters *****************************************************
m = 23.5*(10^-3);
K = 22.3*(10^-3);
K_p = 0.008*(10^-3);
K_q = 0.008*(10^-3);
K_r = 0.065*(10^-3);
N_r = -K_r;
I_xx = 56000*(10^-3)*(10^-3)^2;
I_yy = 56000*(10^-3)*(10^-3)^2;
I_zz = 3800*(10^-3)*(10^-3)^2;
T = 38*10^-3;
g = 9.81;

zd = 15*10^-3;
zb = 60*10^-3 + zd;

d_new = -40*1e-3;
%% Pitch parameters
X_u = -K;
X_q = -K*zd;
M_u = X_q;
M_q = -K_q - K*zd^2;

X_hat_u = X_u/m;
M_hat_q = M_q/I_yy;
M_hat_u = M_u/I_yy;
X_hat_q = X_q/m;
N_hat_r = N_r/I_zz;

%% Pitch dynamics matrice
A_pitch = [X_hat_u  X_hat_q   g  0;
           M_hat_u  M_hat_q   0  1/I_yy;
           0           1      0  0;
           0           0      0 -1/T];
B_pitch = [0 0 0 1/T]';
C_pitch = [X_hat_u+d_new*M_hat_u  X_hat_q+d_new*M_hat_q   0   d_new/I_yy;...
     0        1           0   0];

%% Yaw dynamics matrice
A_yaw = [N_hat_r  1/I_zz;
           0      -1/T];
B_yaw = [0   1/T]';
C_yaw = [1   0];
%%
A_py = blkdiag(A_pitch, A_yaw);
B_py = blkdiag(B_pitch, B_yaw);
C_py = blkdiag(C_pitch, C_yaw);



x_roll  = [0.1 0 2*pi/180 0]';    % Initial State Vector
x_pitch = [0.1 0 2*pi/180 0]';    % Initial State Vector
x_yaw   = [5*pi/180 2*pi/180 0]';    % Initial State Vector


kth_p = 192e-5;
kd_p = 0.27*kth_p;

k_r = 0.8e-5;
T_psi = 0.15;



x_hat_py = [0 0 0 0 0 0]';
Q_n_py = 1*diag([2,200,0,0,200,0]);
R_n_py = diag([10,1,1]);
[X1_py,L_py,G_py] = care(A_py',C_py',Q_n_py,R_n_py,0,eye(6));
L_ob_py = X1_py*C_py'*(R_n_py^-1);
Ob_p_py = eig(A_py-L_ob_py*C_py); 


u_gain = -X_hat_u/g;
z1 = 0.8*2*pi;
p1 = 8*2*pi;
gain = 1.8;
LC_NUM = gain*(p1/z1)*[1 z1];
LC_DEN = [1 p1];


ktha = 192e-5;
kda = 0.27*ktha;
kpa = ktha;
%%

x = [0 0 0*pi/180 0]';    % Initial State Vector Estimate

fa = 2*pi*2;
fg = 2*pi*8;
fc = 2*pi*.1;
kp = 2.0*0.707*fc;
ki = fc^2;
fak = 2*pi*2;
fgk = 2*pi*8;