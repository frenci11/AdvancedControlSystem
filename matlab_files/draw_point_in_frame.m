

function [transformed_point] = draw_point_in_frame(T, point, plot_flag, dot_color)
%   DRAW_POINT_IN_FRAME Transforms and optionally draws a 3D point with respect to a given transformation frame.
%   INPUT:
%       T - 4x4 transformation matrix representing the frame
%       point - 3x1 vector representing the 3D point in the original frame
%       plot_flag - Boolean flag indicating whether to plot the point (true/false)
%       dot_color - self explanatory, optional argument, default red
%   OUTPUT:
%       transformed_point - 3x1 vector representing the transformed 3D point coordinates

    % Validate input dimensions
    if ~isequal(size(T), [4, 4])
        error('The transformation matrix must be 4x4.');
    end
    if ~isequal(size(point), [3, 1])
        error('The point vector must be a 3x1 column vector.');
    end
    if nargin < 4 || isempty(dot_color)
        dot_color = 'r'; % Default color is blue
    end

    point_homogeneous = [point; 1];
    % Transform the point with the given transformation matrix
    transformed_point = T * point_homogeneous;

    % Plot if plot_flag is true
    if plot_flag
        
        T_shift=T;
        T_shift(1:3,4)=transformed_point(1:3);
        % Plot the transformed point in the new frame
        plot3(transformed_point(1), transformed_point(2), transformed_point(3), 'o', 'MarkerSize', 5, 'MarkerFaceColor', dot_color, 'Color', dot_color);
        % Plotting coordinate frame from the transformation matrix
        plotTransforms(T_shift(1:3,4)',rotm2quat(T_shift(1:3, 1:3)), FrameSize=0.05);  

       
    end
end

