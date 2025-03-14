
function active_compliance_Ja(block)

setup(block);


function setup(block)

% Register number of ports
block.NumInputPorts  = 3; %ee pos, desired_frame pos both in operational space
block.NumOutputPorts = 1; %J matrix

%ee pos in op_space
block.InputPort(1).Dimensions        = [4,4]; %4*4 matrix
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;

%desired_frame pos in op_space
block.InputPort(2).Dimensions        = [4,4]; %4*4 matrix
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;

%joint position
block.InputPort(3).Dimensions        = 3; %q1 q2 q3
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = false;

% Output port for the Jacobian matrix (J)
block.OutputPort(1).Dimensions       = [6, 3]; % 6 rows, 3 columns
block.OutputPort(1).DatatypeID       = 0;     % double
block.OutputPort(1).Complexity       = 'Real';


% Setup port properties to be inherited or dynamic
%block.SetPreCompInpPortInfoToDynamic;
%block.SetPreCompOutPortInfoToDynamic;


% Register parameters
block.NumDialogPrms = 1;


% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';


block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup


function Start(block)


%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
%%
function Outputs(block)

simrobot= block.DialogPrm(1).data;

Jacobian_func = simrobot.kinematic.jacobian_geometric;

%desired frame transformation matrix.
Te=block.InputPort(1).Data;
Td=block.InputPort(2).Data;
q=block.InputPort(3).Data;


%end effector actual pose frame.


T_de=inv(Td)*Te;

%extracting the euler angles from the rotation matrix of T_de, with ZYX convenction
rotation=T_de(1:3,1:3);

phi_z=atan2(rotation(2,1),rotation(1,1));
theta_y=asin(-rotation(3,1));
psi_x=atan2(rotation(3,2),rotation(3,3));

%matrix for convertin from angular velocities to angle velocities
T_matrix=[0, -sin(phi_z), cos(phi_z)*cos(theta_y);
    0, cos(phi_z), sin(phi_z)*cos(theta_y);
    1,    0,      -sin(theta_y)];

Ta= blkdiag(eye(3), inv(T_matrix));

if all(Te(:)==0) || all(Td(:)==0)

    x_tilde_dot=eye(6,3);
else
    x_tilde_dot= inv(Ta)*blkdiag(Td(1:3,1:3)',Td(1:3,1:3)')*Jacobian_func(q(1),q(2),q(3));

end

%x_tilde_dot
block.OutputPort(1).Data=x_tilde_dot;

%end Outputs



function Terminate(block)

%end Terminate

