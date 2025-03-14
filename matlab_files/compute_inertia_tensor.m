

function I = compute_inertia_tensor(l, b, c, r_translation, shape, m, axis)
%COMPUTE_INERTIA_TENSOR Computes the inertia tensor matrix with respect to
%the center of mass of a prism or cylinder, and translates it by a given vector.
%
%   INPUTS:
%   l - length of the prism/cylinder
%   b - width (prism) or inner radius (cylinder)
%   c - height (prism) or outer radius (cylinder)
%   r_translation - 3x1 vector representing the translation in 3D space
%   shape - string ('prismatic' or 'cylindrical') specifying the object shape
%   m - mass of the object
%   axis - string ('x', 'y', or 'z') specifying the axis of rotation
%
%   OUTPUT:
%   I - 3x3 inertia tensor matrix translated by vector r_translation

% Validate input
if ~isequal(size(r_translation), [3, 1])
    error('The translation vector r_translation must be a 3x1 column vector.');
end

if ~ischar(shape) || (~strcmp(shape, 'prismatic') && ~strcmp(shape, 'cylindrical'))
    error('Shape must be either "prismatic" or "cylindrical".');
end

% Add axis of rotation handling
if ~ischar(axis) || ~ismember(axis, {'x', 'y', 'z'})
    error('Axis must be one of the following strings: "x", "y", or "z".');
end


% Compute the inertia tensor with respect to the center of mass
if strcmp(shape, 'prismatic')

    % Inertia tensor for a rectangular prism rotates along z axis
    base_I = (1/12) * m * [l^2+c^2,        0,      0;
        0,       l^2+b^2,    0;
        0,           0,   c^2+b^2];
elseif strcmp(shape, 'cylindrical')

    % Inertia tensor for a hollow cylinder (cylindrical shell)
    % l: length, b: inner radius, c: outer radius
    base_I = [m/12*(3*(c^2 + b^2)+l^2),         0,                       0;
        0,            m/12*(3*(c^2 + b^2)+l^2),              0;
        0,                          0,               m/2*(c^2 + b^2)];
end


% Reorder the inertia tensor based on the axis of rotation
Y_rotation=[0,0,-1;
    0,1,0;
    1,0,0];

x_rotation=[1,0,0;
    0,0,-1;
    0,1,0];

switch axis
    case 'x'
        I_c = Y_rotation * base_I * Y_rotation';
    case 'y'
        I_c = x_rotation * base_I * x_rotation';
    case 'z'
        I_c = base_I;
end



% Compute the translation matrix using the formula I = I_c + m * (r^T*r*I - r*r^T)
I_translation = I_c + m*((r_translation'*r_translation * eye(3) - r_translation*r_translation'));

I = I_translation;
end


