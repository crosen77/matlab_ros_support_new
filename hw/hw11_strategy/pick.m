function grip_result = pick(strategy,objectData,optns)
    %----------------------------------------------------------------------
    % pick 
    % Top-level function to executed a complete pick. 
    % 
    % 01 Calls moveTo to move to desired pose
    % 02 Calls doGrip to execute a grip
    %
    % Inputs
    % mat_R_T_M [4x4]: object pose wrt to base_link
    % mat_R_T_G  [4x4]: gripper pose wrt to base_link used as starting point in ctraj (optional)    
    % optns (dict): options 
    %
    % Outputs:
    % ret (bool): 0 indicates success, other failure.
    %----------------------------------------------------------------------
     
    % Check type of objectData. If matrix, assign directly; otherwise
    % extract.
    if isequal(size(objectData), [4, 4]) && isnumeric(objectData)
        mat_R_T_M = objectData;
        label = "can";
    
    % Extract pose and label from object
    else
        label = objectData(1,1); 
        label = label{1};
        mat_R_T_M = objectData(1,2); 
        mat_R_T_M = mat_R_T_M{1};
    end

    %% 1) Determine z offset and grip distance required
    %   z offset includes offset for both the base and the gripper
    
    if strcmp(string(label), "pouch")
        zOffset = 0.15; %% try 0.15 current is too low
        doGripValue = 0.64;
    elseif strcmp(string(label), "vCan")
        zOffset = 0.17; % trying .239. was too low so next try .41
        doGripValue = 0.239; %% 
    elseif strcmp(string(label), "hCan")
        zOffset = 0.135;
        doGripValue = 0.233; 
    elseif strcmp(string(label), "hBottle")
        zOffset = 0.14; %% WORKS
        doGripValue = 0.21; %% WORKS
    elseif strcmp(string(label), "vBottle")
        zOffset = 0.24; %.25 was too high
        doGripValue = 0.515; % .5 almost worked.  .52 either too much or the
        %  placement was off because the bottle tipped over. .51 didnt
        %  work. i thinkn it was too low, so now trying .53 . was too much.
        %  maybe .515 would work. if not i think its the positioning is off
        %  
    end
    

    %% 2) Move to desired location
        % Account for base offset + Hover over object
        if strcmp(strategy,'topdown')
            over_R_T_M = lift(mat_R_T_M,0.25+zOffset);
            MoveToOb = moveTo(over_R_T_M, optns);
            disp(over_R_T_M);

            pick_R_T_M = lift(mat_R_T_M,zOffset);
            MoveToOb = moveTo(pick_R_T_M, optns);
        
        elseif strcmpi(strategy,'direct')
            traj_result = moveTo(mat_R_T_M,optns);
        end
        % Grip object
        
        [grip_result,grip_state] = doGrip('pick',optns,doGripValue); 
        grip_result = grip_result.ErrorCode;
        pause(1);

end