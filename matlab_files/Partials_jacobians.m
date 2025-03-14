
function [partials_jacobians] = Partials_jacobians(direct_transforms,Com,joints_order)

% Partials_jacobians computes the partial geometric Jacobians for a robotic system.
%
% INPUTS:
%   direct_transforms - A cell array of symbolic or numeric 4x4 transformation matrices,
%                       representing the direct transformations from the base to each joint.
%   Com               - A cell array of symbolic or numeric 4x1 homogeneous vectors,
%                       representing the center of mass of each link.
%   joints_order      - A string array specifying the joint type for each link.
%                       Valid values: "rev" (revolute joint) or "prism" (prismatic joint).
%
% OUTPUT:
%   partials_jacobians - A cell array where each element is a symbolic or numeric 6xN matrix.
%                        Each matrix is the combined partial geometric Jacobian for the corresponding link.
%                        matrix order:
%                        [JP;]
%                        [JO ]

% Input Validation
% Check if direct_transforms is a cell array of 4x4 matrices (numeric or symbolic)
if ~iscell(direct_transforms) || any(~cellfun(@(x) isequal(size(x), [4, 4]) && (isnumeric(x) || isa(x, 'sym')), direct_transforms))
    error('direct_transforms must be a cell array containing 4x4 matrices (numeric or symbolic).');
end

% Check if Com is a cell array of 4x1 vectors (numeric or symbolic)
if ~iscell(Com) || any(~cellfun(@(x) isequal(size(x), [4, 1]) && (isnumeric(x) || isa(x, 'sym')), Com))
    error('Com must be a cell array containing 4x1 vectors (numeric or symbolic).');
end

% Check if joints_order is a string array containing only "rev" or "prism"
if ~isstring(joints_order) || any(~ismember(joints_order, ["rev", "prism"]))
    error('joints_order must be a string array containing only "rev" or "prism".');
end

N = size(direct_transforms, 2) - 1;

if(N ~= length(Com))

   warning("PARTIAL JACOBIANS: size of direct transforms do not coincide with size of Com: default to Com size")
   N=length(Com);
end

JP_partial_geometric = cell(1, N);
JO_partial_geometric = cell(1, N);
partials_jacobians = cell(1,N);

for i = 1:N

    JP_partial_geometric{i} = sym(zeros(3, N));
    JO_partial_geometric{i} = sym(zeros(3, N));
end

for i =1:N

    for a=1:i

        if joints_order(a)=="rev"

            P_li= Com{i}(1:3);

            Z=direct_transforms{a}(1:3,3);
            P=direct_transforms{a}(1:3,4);

            JP_partial_geometric{i}(:,a)=simplify(cross(Z,(P_li-P)));
            JO_partial_geometric{i}(:,a)=simplify(Z);
        end

        if joints_order(a)=="prism"

            Z=direct_transforms{a}(1:3,3);

            JP_partial_geometric{i}(:,a)=simplify(Z);
            JO_partial_geometric{i}(:,a)=[0,0,0]';
        end
    end

    partials_jacobians{i}=[JP_partial_geometric{i};JO_partial_geometric{i}];
end






