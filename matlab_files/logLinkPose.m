function logLinkPose(robot, config, linkName)
    % LOGLINKPOSE Logs the position and orientation of a specified link in a robot.
    %
    % INPUT:
    %   robot - A rigidBodyTree object representing the robot model.
    %   config - A configuration object specifying the joint positions for the robot.
    %   linkName - A string specifying the name of the link to log the pose for.
    %
    % OUTPUT:
    %   Prints the position and orientation of the specified link in the format:
    %     - Position: [x, y, z]
    %     - Orientation: [phi, theta, psi] (in radians)
    
    % Get the transformation matrix of the specified link relative to the base frame
    tform = getTransform(robot, config, linkName);

    % Extract the position from the transformation matrix
    pos = tform(1:3, 4); % [x, y, z] position coordinates
    
    % Extract the orientation from the rotation matrix and convert it to Euler angles
    rpy = rotm2eul(tform(1:3, 1:3)); % [phi, theta, psi] Euler angles in radians

    % Display the position in a formatted string
    fprintf('Position: [x: %.2f, y: %.2f, z: %.2f]\n', pos);

    % Display the orientation in a formatted string
    fprintf('Orientation: [phi: %.2f, theta: %.2f, psi: %.2f]\n', rpy);
end