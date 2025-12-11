function traj_goal = convert2ROSPointVec(mat_joint_traj, robot_joint_names, traj_steps, traj_duration, traj_goal, optns)
%--------------------------------------------------------------------------
% convert2ROSPointVec
% Converts all of the matlab joint trajectory values into a vector of ROS
% Trajectory Points.
%
% Make sure all messages have the same DataFormat (i.e. struct)
%
% Inputs:
% mat_joint_traj (n x q) - matrix of n trajectory points for q joint values
% robot_joint_names {} - cell of robot joint names
% traj_goal (FollowJointTrajectoryGoal)
% optns (dict) - traj_duration, traj_steps, ros class handle
%
% Outputs
% vector of TrajectoryPoints (1 x n)
%--------------------------------------------------------------------------
  
   % Get robot handle. Will work with r.point
   r = optns{'rHandle'};
   % Compute time step as duration over steps
   timeStep = traj_duration / traj_steps;
  
   % TODO: Set joint names. Note: must remove finger at index 2
   if numel(robot_joint_names) >= 2
           robot_joint_names(2) = []; % remove joint
   end
   traj_goal.Trajectory.JointNames = cellfun(@char, robot_joint_names, 'UniformOutput', false);
   
   % expected a char
    %% Set Points
   % Set an array of cells (currently only using 1 pt but can be extended)
   points = cell(1,traj_steps);
  
   for i = 1: traj_steps
       % TODO: Create Point Message
        pointMsg = rosmessage('trajectory_msgs/JointTrajectoryPoint', 'DataFormat', 'struct');
      
        % Extract each waypoint and fill r.point
       q_new = mat2rosJoints(mat_joint_traj(i,:));
       r.point = q_new;
      
       pointMsg.Positions = r.point'; % assigning joint positions % transpose
  
       % Time from Start as a struct
       pointMsg.TimeFromStart.Sec  = int32(floor(i * timeStep));
       pointMsg.TimeFromStart.Nsec = int32(round(mod(i * timeStep, 1) * 1e9)); % Set time from start
       % Increasing step by step
  
       % TODO: Set inside points cell
       points{i} = pointMsg;
   end
   % TODO: Copy points to traj_goal.Trajectory.Points
   traj_goal.Trajectory.Points = [points{:}];
end
