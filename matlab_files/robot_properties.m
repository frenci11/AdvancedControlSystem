
%link length
l_0_val=0.15;
l_1_val=0.4;
l_2_val=0.3;
l_3_val=0.16;

joint_type=["rev","prism","rev"];

%-------------dynamic parameters-------------

%          theta_0  d_1  theta_2
q_joints = [pi/4,0,pi/2];
dq_joints=[pi/4, 0.2, pi/5];
ddq_joints=[pi/4, pi/50, 0.002];
%link mass
m=[2,3,0.6];
%gravity
g_0=[0,0,-9.81]';
%frictions viscousf_ee
F_v=[0,0,0];
%friction static
F_s=[0,0,0];

%-------------------------------------------

%------newton euler initial conditions------

omega_0= [0,0,0]';
d_omega_0=[0,0,0]';
ddP_0=[0,0,0]'-[1,0,0;
                0,0,1;
                0,-1,0]*g_0;

f_ee=[0,0,0]';
mu_ee=[0,0,0]';

%------------------------------------------

%choose to plot lot or less
verbose=false;


