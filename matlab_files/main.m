
%robot = importrobot('iiwa14.urdf');

%path to simulink project
addpath('simulink-files\')

%close all
clear
clc

%load all kinetic-dynamic robot properties
run('robot_properties.m');

if verbose

    fprintf("\n\nverbose mode activated, to see less output, set verbose to false in robot_properties\n")
end

robot = importrobot('RPR_yzx FRANCESCO MANTOAN.urdf');

showdetails(robot)
figure(1);
clf;
subplot(2,1,1);

config = homeConfiguration(robot);

config(1).JointPosition = q_joints(1);
config(2).JointPosition = q_joints(2);
config(3).JointPosition = q_joints(3);

robot.Bodies{1}.CenterOfMass=[(l_1_val/2),0,0];
robot.Bodies{2}.CenterOfMass=[0,0,(l_2_val/2)];
robot.Bodies{3}.CenterOfMass=[-l_3_val/2,0,0];

h = show(robot,config);
hold on
robot.DataFormat="column";

%plotting center of mass of the urdf robot
for i = 1:robot.NumBodies

    bodyName = robot.BodyNames{i};

    % Compute the CoM for the current body
    com = robot.Bodies{i}.CenterOfMass;
    com=[com,1];
    T=robot.getTransform(q_joints',bodyName,"base_link");
    com=T*com';
    % Plot the CoM point for the current body
    plotTransforms(com(1:3)', rotm2quat(T(1:3, 1:3)), 'FrameSize', 0.1);
end


title("direct kinematics")
xlim([-0.5 0.8])
ylim([-0.5 0.5])
zlim([0 0.8])
grid minor
hold off

robot.DataFormat="struct";
if verbose
    logLinkPose(robot,config,'ee')
end
clear T com

%% dh table
syms a alpha d theta
syms theta_0 d_1 theta_2 l_0 l_1 l_3
assume([a alpha d theta theta_0 d_1 theta_2 l_0 l_1  l_3],"real")


A_matrix=[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,          sin(alpha),             cos(alpha),             d;
    0,          0,                          0,                  1];


A_b_0= subs(A_matrix,[a,alpha,d,theta],[0,pi/2,l_0,0]);
A_0_1= subs(A_matrix,[a,alpha,d,theta],[l_1,-pi/2,0,theta_0]);
A_1_2= subs(A_matrix,[a,alpha,d,theta],[0,-pi/2,d_1+l_2_val,-pi/2]);
A_2_3= subs(A_matrix,[a,alpha,d,theta],[l_3,0,0,theta_2-pi/2]);

clear a alpha d theta

rot_y_90=[0,0,1,0;
    0,1,0,0;
    -1,0,0,0;
    0,0,0,1];

rot_z_180=[-1,0,0,0;
    0,-1,0,0;
    0,0,1,0;
    0,0,0,1];


%A_e_nse= subs(A_matrix,[a,alpha,d,theta],[0,0,0,pi/2]);

A_b_0=subs(A_b_0,[l_0,l_1,l_3],[l_0_val,l_1_val,l_3_val]);
A_0_1=subs(A_0_1,[l_0,l_1,l_3],[l_0_val,l_1_val,l_3_val]);
A_1_2=subs(A_1_2,[l_0,l_1,l_3],[l_0_val,l_1_val,l_3_val]);
A_2_3=subs(A_2_3,[l_0,l_1,l_3],[l_0_val,l_1_val,l_3_val]);
A_3_ee=rot_y_90*rot_z_180;

%this contains the cumulative prod. of all the transformation matricies
%with q_joints variables SYMBOLIC
direct_transforms=cumulative_transform(A_b_0, A_0_1, A_1_2, A_2_3, A_3_ee);

%this contains the partials transformations
partial_direct_transform={A_b_0, A_0_1, A_1_2, A_2_3, A_3_ee};


%direct kin. frame plot
figure(1)
subplot(2,1,2);
%clf
hold on
grid on
grid minor
axis equal

% plot axes origin
plotTransforms([0 0 0], [1 0 0 0], FrameSize=0.1);

for i=1:size(direct_transforms,2)
    d=double(subs(direct_transforms{i},[theta_0,d_1,theta_2],q_joints));

    if i==size(direct_transforms,2)
        d=double(subs(direct_transforms{i},[theta_0,d_1,theta_2],q_joints));
        d(:,4)=draw_point_in_frame(d,[0.01,0,0.05]',0);

    end

    %plots the ee frame a bit bigger than the intemrediate frames
    if i<size(direct_transforms,2)
        plotTransforms(d(1:3,4)',rotm2quat(d(1:3, 1:3)), FrameSize=0.1);

        %plots a point of each frame, w.r.t base frame
        draw_point_in_frame(d,[0,0,0]',1);

    else
        plotTransforms(d(1:3,4)',rotm2quat(d(1:3, 1:3)), FrameSize=0.2);
        draw_point_in_frame(d,[0,0,0]',1);

    end
    %plots the line connecting each frame of the robot
    if i > 1
        d=subs(direct_transforms{i},[theta_0,d_1,theta_2],q_joints);
        prev_d=subs(direct_transforms{i-1},[theta_0,d_1,theta_2],q_joints);
        line([d(1,4);prev_d(1,4)],[d(2,4);prev_d(2,4)],[d(3,4);prev_d(3,4)],'LineWidth',2);
    end
end

%plot center of mass frames (w.r.t base frame) and also save it in symbolic value that depends
%on joints variables
%plotted CoM are slightly offset for better visualization, but they are saved with
%the correct position

l_link_1=[-l_1_val,0,0];
l_link_2=[0,l_2_val,0];
l_link_3=[-l_3_val,0,0];
l_link={l_link_1, l_link_2, l_link_3};
clear l_link_1 l_link_2 l_link_3

Com=cell(1,3);


d=double(subs(direct_transforms{2},[theta_0,d_1,theta_2],q_joints));
draw_point_in_frame(d,l_link{1}'/2+[0,0,0.01]',1,'k');


Cm_1=draw_point_in_frame(direct_transforms{2},l_link{1}'/2,0);
Com{1}=simplify(Cm_1);


d=double(subs(direct_transforms{3},[theta_0,d_1,theta_2],q_joints));
draw_point_in_frame(d,l_link{2}'/2+[0,0,0.01]',1,'k');


Cm_2=draw_point_in_frame(direct_transforms{3},l_link{2}'/2,0);
Com{2}=simplify(Cm_2);


d=double(subs(direct_transforms{4},[theta_0,d_1,theta_2],q_joints));
draw_point_in_frame(d,l_link{3}'/2+[0,0,0.01]',1,'k');


Cm_3=draw_point_in_frame(direct_transforms{4},l_link{3}'/2,0);
Com{3}=simplify(Cm_3);

clear d prev_d A_b_0 A_0_1 A_1_2 A_2_3 A_3_ee Cm_1 Cm_2 Cm_3
hold off

%% direct kin evaluation

%multiply for the final rotation A_3_ee, that rotates the ee according to t
%the robotic toolbox
A_tot_direct=subs(direct_transforms{end},[theta_0,d_1,theta_2],q_joints);
A_tot_direct=double(A_tot_direct);
disp('direct kin matrix evaluation')
disp(A_tot_direct)
rpy = rotm2eul(A_tot_direct(1:3, 1:3));
fprintf('MartrixOrientation: [phi: %.2f, theta: %.2f, psi: %.2f]\n', rpy);
clear rpy
fprintf("\n------------------\n")


%% inverse_kinematic
%this is the solution for direct_transform{4}, i.e the last frame of the ee w.r.t
%base without the final rotiation


syms x y z
assume ([x,y,z],'real');

q3=asin(-25/4*y);

q2= sqrt(x^2+(z-3/20)^2-(2/5)^2)-3/10-(4*cos(q3))/25;

M=3/10+4/25*cos(q3)+q2;

q1= 2*( atan((sqrt(25*M^2 - 25*x^2 + 4) - 5*M)/(5*x + 2)) );

inverse_kin=matlabFunction([q1,q2,q3],'Vars',[x,y,z]);


res=inverse_kin(A_tot_direct(1,4),A_tot_direct(2,4),A_tot_direct(3,4));


disp("inverse kin evaluation")
fprintf('q1= %.2f\n',res(1));

fprintf('q2= %.2f\n',res(2))

fprintf('q3= %.2f\n', res(3))



if verbose
    robot.DataFormat="struct";
    config_inverse = homeConfiguration(robot);

    config_inverse(1).JointPosition = real(res(1));
    config_inverse(2).JointPosition = real(res(2));
    config_inverse(3).JointPosition = real(res(3));


    figure(1)
    subplot(2,1,1);
    hold on
    h = show(robot,config_inverse);
    hold off
end

clear q1 q2 q3 x y z


%% partials geometric jacobians

J_partial_geometric= Partials_jacobians(direct_transforms,Com,["rev","prism","rev"]);


%% -----------kinetic energy------------------------------------------------------------------------

%I_c + m*((r_translation'*r_translation * eye(3) - r_translation*r_translation'));
%seems like both inertias need to be translated from their center of mass, according
%to their reference frame


%inertias according to robotic toolbox frames
r_translation=  robot.Bodies{1}.CenterOfMass';
Inertia_tensor=compute_inertia_tensor(l_1_val, 0, 0.03, r_translation,'cylindrical',m(1),'x');

I_robotic_toolbox_1=Inertia_tensor + 1*((r_translation'*r_translation * eye(3) - r_translation*r_translation'));

r_translation= - robot.Bodies{2}.CenterOfMass';
Inertia_tensor=compute_inertia_tensor(l_2_val, 0.03, 0.03, r_translation,'prismatic',m(2),'y');

I_robotic_toolbox_2=Inertia_tensor + 1*((r_translation'*r_translation * eye(3) - r_translation*r_translation'));

r_translation= - robot.Bodies{3}.CenterOfMass';
Inertia_tensor=compute_inertia_tensor(l_3_val, 0, 0.03, r_translation,'cylindrical',m(3),'x');

I_robotic_toolbox_3=Inertia_tensor + 1*((r_translation'*r_translation * eye(3) - r_translation*r_translation'));


%inertias according to dh frames
r_translation=[-l_1_val/2,0,0]';
I_link1=compute_inertia_tensor(l_1_val, 0, 0.03, r_translation,'cylindrical',m(1),'x');

r_translation=[0,(l_2_val/2),0]';
I_link2=compute_inertia_tensor(l_2_val, 0.03, 0.03, r_translation,'prismatic',m(2),'x');

r_translation=[0,0,-l_3_val/2]';
I_link3=compute_inertia_tensor(l_3_val, 0, 0.03, r_translation,'cylindrical',m(3),'z');


I_link={I_link1, I_link2, I_link3};

clear I_link1 I_link2 I_link3


%robotic toolbox inertia[Ixx Iyy Izz Iyz Ixz Ixy] vector

robot.Bodies{1}.Inertia=[diag(I_robotic_toolbox_1)',0,0,0];
robot.Bodies{2}.Inertia=[diag(I_robotic_toolbox_2)',0,0,0];
robot.Bodies{3}.Inertia=[diag(I_robotic_toolbox_3)',0,0,0];

robot.Bodies{1}.Mass=m(1);
robot.Bodies{2}.Mass=m(2);
robot.Bodies{3}.Mass=m(3);



i=1;
link1_inertia=m(1)*J_partial_geometric{i}(1:3,:)'*J_partial_geometric{i}(1:3,:)+ ...
    J_partial_geometric{i}(4:6,:)'*direct_transforms{i}(1:3,1:3)*I_link{1}*direct_transforms{i}(1:3,1:3)'*J_partial_geometric{i}(4:6,:);

link1_inertia= simplify(link1_inertia);

i=2;

link2_inertia=m(2)*J_partial_geometric{i}(1:3,:)'*J_partial_geometric{i}(1:3,:)+ ...
    J_partial_geometric{i}(4:6,:)'*direct_transforms{i}(1:3,1:3)*I_link{2}*direct_transforms{i}(1:3,1:3)'*J_partial_geometric{i}(4:6,:);

link2_inertia= simplify(link2_inertia);

i=3;

link3_inertia=m(3)*J_partial_geometric{i}(1:3,:)'*J_partial_geometric{i}(1:3,:)+ ...
    J_partial_geometric{i}(4:6,:)'*direct_transforms{i}(1:3,1:3)*I_link{3}*direct_transforms{i}(1:3,1:3)'*J_partial_geometric{i}(4:6,:);

link3_inertia= simplify(link3_inertia);

total_inertia=link1_inertia+link2_inertia+link3_inertia;


if verbose
    disp('matrix inertia with joints value substituted')
    check=double(subs(total_inertia,[theta_0,d_1,theta_2],q_joints));
    disp(check)
    disp('eigenvalue check')
    disp(eig(double(check)))
end


%------------------potential energy--------------------------------------------------------------------

link_potential=[-(m(1)*g_0'*Com{1}(1:3)), -(m(2)*g_0'*Com{2}(1:3)), -(m(3)*g_0'*Com{3}(1:3))]';

disp('link1 potential')
disp(double(subs(link_potential(1),[theta_0,d_1,theta_2],q_joints)))

disp('link2 potential')
disp(double(subs(link_potential(2),[theta_0,d_1,theta_2],q_joints)))

disp('link3 potential')
disp(double(subs(link_potential(3),[theta_0,d_1,theta_2],q_joints)))

total_potential=sum(link_potential);

if verbose
    disp('total potential')
    disp(double(subs(total_potential,[theta_0,d_1,theta_2],q_joints)))

end

clear I_link1 I_link2 I_link3 I_robotic_toolbox_1 I_robotic_toolbox_2 I_robotic_toolbox_3

%% -------equations of motion and lagrangian----------------------------------------------------------
%load all kinetic-dynamic robot properties
run('robot_properties.m');

%parameters for inverse dynamics
%joint positions are already defined at beginning of the script

robot.Gravity=g_0';
robot.DataFormat="column";


disp("kinetic energy")
kinetic=1/2*dq_joints*total_inertia*dq_joints';
kinetic=double(subs(kinetic,[theta_0,d_1,theta_2],q_joints));
disp(kinetic)

disp("torques from robotic toolbox")
torques=inverseDynamics(robot,q_joints',dq_joints',ddq_joints');
disp(torques)

%these symfun variables, are temporary just to use the diff(t), they will be
%substituted later with the corresponding sym that is not function of t
syms q_1(t) q_2(t) q_3(t)
q=[q_1(t) q_2(t) q_3(t)];

syms dq_1(t) dq_2(t) dq_3(t)
dq=[dq_1(t) dq_2(t) dq_3(t)];

syms ddq_1(t) ddq_2(t) ddq_3(t)
ddq=[ddq_1(t) ddq_2(t) ddq_3(t)];


%inertia matrix variables substitution so they depends on t
B_matrix=subs(total_inertia,[theta_0,d_1,theta_2],[q_1(t),q_2(t),q_3(t)]);

%potential enegry matrix variables substitution so they depends on t
U_matrix=subs(link_potential,[theta_0,d_1,theta_2],[q_1(t),q_2(t),q_3(t)]);

%grad_B = gradient([dq_1(t), dq_2(t), dq_3(t)] * B_matrix * [dq_1(t), dq_2(t), dq_3(t)].', [q_1(t), q_2(t), q_3(t)]);
grad_B = gradient(dq * B_matrix * dq.',[q_1(t), q_2(t), q_3(t)]);


total_U = sum(U_matrix);
grad_U=gradient(total_U,[q_1(t),q_2(t),q_3(t)]);

dB_matrix=diff(B_matrix, t);
dB_matrix_sub = subs(dB_matrix, [diff(q_1(t), t), diff(q_2(t), t), diff(q_3(t), t)], [dq_1(t), dq_2(t), dq_3(t)]);


tau_lagrange= B_matrix*ddq.' + dB_matrix_sub*dq.' - (1/2*(grad_B)) + grad_U + diag(F_v)*dq.' + diag(F_s)*sign(dq.');

disp("torques from lagrangian")
calculated_tau=double(subs(tau_lagrange,[q_1(t),q_2(t),q_3(t),dq_1(t),dq_2(t),dq_3(t),ddq_1(t),ddq_2(t),ddq_3(t),],[q_joints,dq_joints,ddq_joints]));
disp(calculated_tau)

%coriolis matrix calculation, to excorporate it from B inertia matrix.
C_matrix=sym(zeros(3,3,3));
for i=1:3

    for j=1:3

        for k=1:3

            C_matrix(i,j,k)=1/2 * (diff(B_matrix(i,j),q(k)) + diff(B_matrix(i,k),q(j)) - diff(B_matrix(j,k),q(i))) * dq(k);

        end
    end
end


C_matrix=simplify(sum(C_matrix,3));

disp("torques from lagrangian with explicit C_matrix")
tau_2_lagrange=B_matrix*ddq.' + C_matrix*dq.' + diag(F_v)*dq.'+ diag(F_s)*sign(dq.') + grad_U;

calculated_tau_2=double(subs(tau_2_lagrange,[q_1(t),q_2(t),q_3(t),dq_1(t),dq_2(t),dq_3(t),ddq_1(t),ddq_2(t),ddq_3(t)],[q_joints,dq_joints,ddq_joints]));
disp(calculated_tau_2)



%analytical jacobian as a function of q(t)

Ja=sym(zeros(6,3));

Ja(1,1:3)=gradient(direct_transforms{5}(1,4),[theta_0,d_1,theta_2]);
Ja(2,1:3)=gradient(direct_transforms{5}(2,4),[theta_0,d_1,theta_2]);
Ja(3,1:3)=gradient(direct_transforms{5}(3,4),[theta_0,d_1,theta_2]);

%using convention Z-Y-X yaw pitch roll for rotation matrix to euler angles
rotation= direct_transforms{5}(1:3,1:3);

phi_z=atan2(rotation(2,1),rotation(1,1));
theta_y=asin(-rotation(3,1));
psi_x=atan2(rotation(3,2),rotation(3,3));

Ja(4,1:3) = gradient(phi_z, [theta_0,d_1,theta_2]);   % Derivative of yaw
Ja(5,1:3) = gradient(theta_y, [theta_0,d_1,theta_2]); % Derivative of pitch
Ja(6,1:3) = gradient(psi_x, [theta_0,d_1,theta_2]);   % Derivative of roll

Ja=subs(Ja,[theta_0,d_1,theta_2],q);
Ja=simplify(Ja);

syms phi theta psi
assume([phi,theta,psi],'real')

rot_x=[1,0,0;
    0,cos(psi),-sin(psi);
    0,sin(psi),cos(psi)];

rot_y=[cos(theta),0,sin(theta);
     0,1,0;
     -sin(theta),0,cos(theta)];

rot_z=[cos(phi),-sin(phi),0;
    sin(phi),cos(phi),0;
    0,0,1];

%Z Y X convenction

d_phi= eye(3)*[0,0,1]';

d_theta= rot_z*[0,1,0]';

d_psi= rot_z*rot_y*[1,0,0]';

T_matrix=[d_phi,d_theta,d_psi];

%T matrix that maps J = T * Ja (the rotational part)
T_matrix=[0, -sin(phi_z), cos(phi_z)*cos(theta_y);
        0, cos(phi_z), sin(phi_z)*cos(theta_y);
        1,    0,      -sin(theta_y)];
%block diagonal matrix
T_matrix = blkdiag(eye(3), T_matrix);

T_matrix=subs(T_matrix,[theta_0,d_1,theta_2],q);

%derivate of analytical jacobian
d_Ja= simplify(diff(Ja,t));
d_Ja=subs(d_Ja,[diff(q_1(t), t), diff(q_2(t), t), diff(q_3(t), t)], [dq_1(t), dq_2(t), dq_3(t)]);



clear q dq ddq dB_matrix dB_matrix_sub %q_0 q_1 q_2 dq_0 dq_1 dq_2 ddq_0 ddq_1 ddq_2



%% ----------newton euler formulation----------------------

%load all kinetic-dynamic robot properties
run('robot_properties.m');

%i am doing a temporary structure to pass all the parameters to the Newton Euler
%function, just to heve it outside the main script

input_struct.direct_kin.partial_direct_transform = partial_direct_transform;
input_struct.direct_kin.direct_transforms = direct_transforms;
input_struct.direct_kin.Com= Com;

input_struct.direct_dyn.Inertia = I_link;
input_struct.direct_dyn.mass= m;
input_struct.direct_dyn.F_v= F_v;
input_struct.direct_dyn.F_s= F_s;

input_struct.initial_cond.omega_0 = omega_0;
input_struct.initial_cond.d_omega_0 = d_omega_0;
input_struct.initial_cond.ddP_0 = ddP_0;
input_struct.initial_cond.f_ee = f_ee;
input_struct.initial_cond.mu_ee = mu_ee;

input_struct.joints.joint_type=joint_type;

tau_euler=recursive_NE(input_struct,dq_joints,ddq_joints);

disp('torques from newton euler')
for i=1:size(tau_euler,2)
    disp(double(subs(tau_euler{i},[theta_0,d_1,theta_2],q_joints)))
end

clear input_struct



%% ---------export robot structure fro simulink-------------

%calculate the jacobian from base to end, exploiting the partial_jacobian function,
%giving as point the end frame instead of the Com.
point = {direct_transforms{5}(:,4),direct_transforms{5}(:,4),direct_transforms{5}(:,4)};

J=Partials_jacobians(direct_transforms,point, joint_type);
J=J{end};


syms q1 q2 q3
q=[q1 q2 q3];

syms dq1 dq2 dq3
dq=[dq1 dq2 dq3];

syms ddq1 ddq2 ddq3
ddq=[ddq1 ddq2 ddq3];

clear q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3


simrobot.kinematic.q = q;
simrobot.kinematic.dq = dq;
simrobot.kinematic.ddq = ddq;



direct_kin=subs(direct_transforms{4},[theta_0,d_1,theta_2],q);
simrobot.kinematic.direct_kin = matlabFunction(direct_kin,'Vars',q);
clear direct_kin

J=subs(J,[theta_0,d_1,theta_2],q);
simrobot.kinematic.jacobian_geometric=matlabFunction(J,'Vars',q);

%analytical jacobian
Ja=subs(Ja,[q_1(t),q_2(t),q_3(t)],q);
simrobot.kinematic.jacobian_analytical=matlabFunction(Ja, 'Vars',q);

%analytical jacobian pesudoinverse
pseudo_inv_Ja= simplify(inv( Ja(1:3,:).' * Ja(1:3,:)) * Ja(1:3,:).');
simrobot.kinematic.jacobian_analytical_pseudoinv=matlabFunction(pseudo_inv_Ja,'Vars',q);
clear pseudo_inv_Ja

%analytical jacobian transformation matrix
T_matrix=subs(T_matrix,[q_1(t),q_2(t),q_3(t)],q);

%d/dt of analytical jacobian (i substitute the old values used to calculate the
%derivative, that are symfun on t, with the new sym that do not depend on t)
d_Ja=subs(d_Ja,[q_1(t),q_2(t),q_3(t),dq_1(t),dq_2(t),dq_3(t)],[q,dq]);
simrobot.kinematic.d_jacobian_analytical=matlabFunction(d_Ja, 'Vars',[q, dq]);
clear d_Ja


% Include dynamic parameters
B_matrix=subs(B_matrix,[q_1(t),q_2(t),q_3(t)],q);
simrobot.dynamic.B_matrix = matlabFunction(B_matrix, 'Vars', simrobot.kinematic.q);

C_matrix=subs(C_matrix,[q_1(t),q_2(t),q_3(t),dq_1(t),dq_2(t),dq_3(t)],[q,dq]);
simrobot.dynamic.C_matrix = matlabFunction(C_matrix, 'Vars', [q, dq]);

grad_U=subs(grad_U,[q_1(t),q_2(t),q_3(t)],q);
simrobot.dynamic.grad_U = matlabFunction(grad_U, 'Vars', simrobot.kinematic.q);


% Add friction parameters
simrobot.dynamic.Fv = F_v(:)'; % These are already numeric
simrobot.dynamic.Fs = F_s(:)'; % These are already numeric

%add Md matrix for force control inertia

Md=[1,0,0;
    0,1,0;
    0,0,1];

simrobot.dynamic.Md= Md;
simrobot.dynamic.inv_Md= inv(Md);

%add Mt matrix for admittance control

Mt=6;

simrobot.dynamic.Mt= Mt;
simrobot.dynamic.inv_Mt= inv(Mt);








%% some trajectories to use in simulink whenever necessary
% A * sin(w*t)+phase

syms t;

A = pi/6;
w = 2;
phase= pi/3;

q_ref = A*sin(w*t+phase);
dq_ref = diff(q_ref);
ddq_ref = diff(dq_ref);

q_ref = matlabFunction(q_ref,Vars=t);
dq_ref = matlabFunction(dq_ref,Vars=t);
ddq_ref = matlabFunction(ddq_ref,Vars=t);


clear q_1 q_2 q_3 dq_1 dq_2 dq_3 ddq_1 ddq_2 ddq_3 Md
clear A W phase 



%% ----------------------environment----------------------------
%is just a placeholder to plot the env. plane on the figure. the function that
%calculates the environment is implemented on simulink.

%open simulink projects

% environment placeholder
[y,z] = meshgrid(-0.5:0.1:0.5);
z=z+0.5;

%plane normal
v=[1,0,0];
%offset along normal
d=-0.35;

x = -(v(2)*y+v(3)*z)+d/v(1);

figure(1)
subplot(2,1,2);
hold on


ee_position=subs(direct_transforms{4},[theta_0,d_1,theta_2],q_joints);
ee_position=ee_position(1:3,4);

distance=double(v(1)*ee_position(1)+v(2)*ee_position(2)+v(3)*ee_position(3)-d);
projection=ee_position-(v'*distance);

if verbose
    surf(x,y,z);
    plot3(projection(1), projection(2), projection(3), 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
    hold off
end

clear x y z distance v


% that's all folks!







