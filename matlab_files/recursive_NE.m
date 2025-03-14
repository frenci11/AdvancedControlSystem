


function [tau_euler] = recursive_NE(input_struct,dq,ddq)

%inputStruct - A structure with the following fields:
%       .direct_kin
%           .partial_direct_transform - A cell array of 4x4 transformation matrices (symbolic or numeric).
%           .direct_transforms         - A cell array of 4x4 transformation matrices (symbolic or numeric).          
%           .Com                        - A cell array of 3x1 center of mass
%       .direct_dyn
%           .I_link     - A cell array of 3x3 inertia matricies
%           .mass       - A n*1 array with joint mass
%       .initial_cond
%           .omega_0     - 3x1 vector of initial angular velocity.
%           .d_omega_0   - 3x1 vector of initial angular acceleration.
%           .ddP_0       - 3x1 vector of initial linear acceleration.
%           .f_ee        - 3x1 vector of external forces at the end effector.
%           .mu_ee       - 3x1 vector of external torques at the end effector.
%       .joints
%           .joint_type     - A string vector of joint types "rev" or "prism"


%   dq - Joint velocities as a \(1 \times n\) array of doubles.
%   ddq - Joint accelerations as a \(1 \times n\) array of doubles.
%
% OUTPUT:
%   tau_euler - Joint torques calculated using the Newton-Euler method.

%input check
if iscell(dq)
    dq = cell2mat(dq); % Convert cell array to matrix
end
if size(dq, 1) > 1
    dq = dq'; % Ensure it is 1 x n
end
validateattributes(dq, {'double'}, {'row'}, mfilename, 'dq');

if iscell(ddq)
    ddq = cell2mat(ddq); % Convert cell array to matrix
end
if size(ddq, 1) > 1
    ddq = ddq'; % Ensure it is 1 x n
end
validateattributes(ddq, {'double'}, {'row'}, mfilename, 'ddq');

%variables initialization from input struct
partial_direct_transform=input_struct.direct_kin.partial_direct_transform;
direct_transforms=input_struct.direct_kin.direct_transforms;
Com=input_struct.direct_kin.Com;

I_link=input_struct.direct_dyn.Inertia;
m=input_struct.direct_dyn.mass;
F_v=input_struct.direct_dyn.F_v;
F_s=input_struct.direct_dyn.F_s;

omega_0=input_struct.initial_cond.omega_0;
d_omega_0=input_struct.initial_cond.d_omega_0;
ddP_0=input_struct.initial_cond.ddP_0;
f_ee=input_struct.initial_cond.f_ee;
mu_ee=input_struct.initial_cond.mu_ee;

joint_type=input_struct.joints.joint_type;

dq_joints=dq;
ddq_joints=ddq;


omega=cell(1,4);
d_omega=cell(1,4);
ddP=cell(1,4);
ddP_cm=cell(1,4);

for i = 1:4
    omega{i} = sym(zeros(3, 1));
    d_omega{i} = sym(zeros(3, 1));
    ddP{i}=sym(zeros(3, 1));
    ddP_cm{i}=sym(zeros(3, 1));
end

omega{1}=omega_0;
d_omega{1}=d_omega_0;
ddP{1}=ddP_0;

z0=[0,0,1]';


%----------------------------------forward euler-----------------------------------------------

for i=2:4

    if(joint_type(i-1)=="rev")

        %_b_0 , A_0_1 , A_1_2 , A_2_3 , A_3_ee
        R=partial_direct_transform{i}(1:3,1:3);

        %Com is expressed w.r.t the base, so i take the transformation from base to
        %frame i, (so not the partial transform) invert it so i have the position of
        %Com w.r.t current frame
        r_i_ci=inv(direct_transforms{i})*Com{i-1};
        r_i_ci=simplify(r_i_ci(1:3));
        r_i=R.'*partial_direct_transform{i}(1:3,4);

        omega{i}= R.'* omega{i-1} + R.'* dq_joints(i-1) * z0;

        d_omega{i}= R.'* d_omega{i-1} + R.'* (ddq_joints(i-1)*z0 + cross(dq_joints(i-1)*omega{i-1},z0));

        ddP{i}= R.'* ddP{i-1}+ cross(d_omega{i},r_i ) +...
            cross(omega{i},cross(omega{i},r_i ));

    end

    if(joint_type(i-1)=="prism")

        R=partial_direct_transform{i}(1:3,1:3);

        r_i_ci=inv(direct_transforms{i})*Com{i-1};
        r_i_ci=simplify(r_i_ci(1:3));
        r_i=R.'*partial_direct_transform{i}(1:3,4);

        omega{i}= R.'* omega{i-1};

        d_omega{i}= R.'* d_omega{i-1};

        ddP{i}= R.'* ddP{i-1}+ cross(d_omega{i},r_i ) +...
            cross(omega{i},cross(omega{i},r_i ))+...
            R.'* ddq_joints(i-1)*z0+ cross(2*dq_joints(i-1)*omega{i}, (R.'*z0) );

    end

    ddP_cm{i}= ddP{i} + cross(d_omega{i}, r_i_ci )+ cross(omega{i},cross(omega{i},r_i_ci));

    omega{i}=simplify(omega{i});
    d_omega{i}=simplify(d_omega{i});
    ddP{i}=simplify(ddP{i});
    ddP_cm{i}=simplify(ddP_cm{i});

    %run('debug_print.m')
end



%--------------------------------backward euler--------------------------------------------

f=cell(1,5);
mu=cell(1,5);
tau_euler=cell(1,3);

R=cell(1,5);
R=partial_direct_transform;

for i = 1:4
    f{i} = sym(zeros(3, 1));
    mu{i} = sym(zeros(3, 1));
end

f{5}=f_ee;
mu{5}=mu_ee;

for i=4:-1:2


    R_next=R{i+1}(1:3,1:3);
    R_prev=R{i}(1:3,1:3);

    r_i_ci=inv(direct_transforms{i})*Com{i-1};
    r_i_ci=simplify(r_i_ci(1:3));
    r_i=R_prev.'*partial_direct_transform{i}(1:3,4);

    f{i}= R_next*f{i+1}+m(i-1)*ddP_cm{i};

    mu{i}= cross(-f{i},(r_i + r_i_ci)) + R_next*mu{i+1}+ cross(R_next*f{i+1},r_i_ci)+...
        R_prev'*I_link{i-1}*R_prev*d_omega{i}+ cross(omega{i}, (R_prev'*I_link{i-1}*R_prev*omega{i}));

    mu{i}=simplify(mu{i});
    f{i}=simplify(f{i});

    if(joint_type(i-1)=="rev")

        tau_euler{i-1}=mu{i}.'*R_prev.'*z0 + F_v(i-1)*dq_joints(i-1) + F_s(i-1)*sign(dq_joints(i-1));
    end

    if(joint_type(i-1)=="prism")

        tau_euler{i-1}=f{i}.'*R_prev.'*z0 +  F_v(i-1)*dq_joints(i-1) + F_s(i-1)*sign(dq_joints(i-1));
    end

end

end











