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
        zOffset = 0.155; % YES
        doGripValue = 0.674; % YES
        bin = 'blue';
    elseif strcmp(string(label), "vCan")
        zOffset = 0.17; % trying .239. was too low so next try .41
        doGripValue = 0.235; %%  .239
        bin = 'green';
    elseif strcmp(string(label), "hCan")
        zOffset = 0.135; % .135
        doGripValue = 0.233; 
        bin = 'green';
    elseif strcmp(string(label), "hBottle")
        zOffset = 0.1435; %% WORKS
        doGripValue = 0.21; %% WORKS try .205 becayse it currently sometimes holds on
        bin = 'blue';
    elseif strcmp(string(label), "vBottle")
        %if mat_R_T_M(2, 4) > .35
        %    zOffset = 0.245; 
        %else
        %    strategy = 'sidepick';
        %    mat_R_T_M(1:3,1:3) = [0 0 -1 ; 0 -1 0 ; -1 0 0];
        %end
        zOffset = 0.245; 
        doGripValue = 0.515;
        bin = 'blue';
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
            pick_R_T_M = lift(mat_R_T_M, zOffset);
            traj_result = moveTo(pick_R_T_M,optns);

        %elseif strcmpi(strategy,'sidepick')
            
        end
        % Grip object
        
        [grip_result,grip_state] = doGrip('pick',optns,doGripValue); 
        grip_result = grip_result.ErrorCode;
        pause(3);
        
        % Determine bin location
        if strcmp(bin, 'green')
            bin = [2.2 0 pi/2 -pi/2 0 0];
        else
            bin = [-0.8*pi 0 pi/2.3 -pi/2.3 0 0];
        end

        % Place
        if strcmp(strategy,'topdown') 
            moveTo(over_R_T_M, optns);
            moveToQ("custom",optns,bin);
            [grip_result,grip_state] = doGrip('place',optns);
            grip_result = grip_result.ErrorCode;
        end

end