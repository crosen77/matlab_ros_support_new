function staticPickAndPlace(optns)
%--------------------------------------------
% This a sub-optimal function. Requires manual work to update.
% Will change with new environments. 
%--------------------------------------------
    % yBottle1 is laying down
    % need to turn fingers
    yBottle1 = [ 0  1  0  -0.5262;...
                 1  0  0  -0.043;...
                 %0  0 -1   0.0335;...
                 0  0  -1 -0.0665;...
                 0  0  0   1];
    yBottle1 = yBottle1*trotz(pi/2);
    
    % gCan1 is standing on top of carboard box
    gCan1    = [ 0  1  0  -0.3628;...
                 1  0  0  -0.0071;...
                 0  0 -1   0.067;...
                 0  0  0   1];
    
    % rBottle1 is laying down
    % need to turn fingers
    rBottle1 = [ 0  1  0  -0.6193;...
                 1  0  0   0.32;...
                 0  0 -1   -0.0663;...
                 0  0  0   1];
    rBottle1 = rBottle1*trotz(pi/2);
    
    % rCan1 is standing up
    rCan1    = [ 0  1  0  -0.5024;...
                 %1  0  0   0.3953;...
                 1  0  0   0.27;... %% Changed and works now
                 0  0 -1   -0.0419;...
                 0  0  0   1];

    % spam1 is behind box -- WORKS
    spam1    = [ 0  1  0  -0.48;...
                 1  0  0   0.36;... 
                 0  0 -1   -0.0419;...
                 0  0  0   1];
    spam1 = spam1*trotz(deg2rad(-60));

    % marker1 is by green bin
    marker1    = [ 0  1  0  -0.66;...
                   1  0  0   -0.025;... 
                   0  0 -1   -0.0419;...
                   0  0  0   1];

    
    % bBottle1 is standing
    bBottle1 = [ 0  1  0   0.4602;...
                 1  0  0  -0.0702;...
                 0  0 -1   -0.0015;...
                 0  0  0   1];
    
    % rCan2 is standing on gCan3
    rCan2    = [ 0  1  0   0.663;...
                 1  0  0   0.0183;...
                 0  0 -1   0.0748;...
                 0  0  0   1];
    % gCan3 is under rCan2
    gCan3    = [ 0  1  0   0.663;... 
                 1  0  0   0.0192;...
                 0  0 -1   -0.0416;...
                 0  0  0   1];

    % pouchR
    pouchP   = [0 1 0 0.69;...
                1 0 0 -0.075;...
                0 0 -1 -0.105;...
                0 0 0 1];
    
    % easyObjects = { %Zone1
    %                "can" gCan1;...
    %                "bottle" yBottle1;...
    %                "can" rCan1;...
    %                "bottle" rBottle1;...
    %                % Zone 2
    %                "bottle" bBottle1;...
    %                "can" rCan2;...
    %                "can" gCan3};
    
    
     %% Set custome zone2 for the rest of these objects
    StartZone = [-1.8380   -0.0978    1.8084   -1.7105    0    4.4452];
    moveToQ("Custom",optns,StartZone); % move to Zone 2
    

% rCan2---------------------------------------------------------------

    % Hover
    hover = lift(rCan2, 0.3);
    moveTo(hover, optns);

    % Pick
    rCan2(3,4) = rCan2(3,4) + 0.18;
    moveTo(rCan2, optns);
    pause(1);
    doGrip("pick", optns, 0.237); % For sopme reason .23 works over .24 here
    pause(1);

    % Place w/ custom path
    moveTo(hover, optns);
    moveToQ("Custom", optns, [0 0 pi/2 -pi/2 0 0]); % pass through home position
    pause(2);

    moveToQ("Custom", optns, [5/8*pi 0 pi/2.3 -pi/2.3 0 0]); % Green
    doGrip("place", optns);
    pause(1);

    % Return to zone pose via home position - 2 motions
    moveToQ("Custom", optns, [0 0 pi/2 -pi/2 0 0]); % pass through home position
    pause(2);
    moveToQ("Custom",optns,StartZone);

% gCan3----------------------------------------------------------------
    
    % Hover
    hover = lift(gCan3, 0.3);
    moveTo(hover, optns);

    % Pick
    gCan3(3,4) = gCan3(3,4) + 0.18;
    moveTo(gCan3, optns);
    pause(1);
    doGrip("pick", optns, 0.24);
    pause(1);

    % Place through home
    moveTo(hover, optns);
    moveToQ("Custom", optns, [0 0 pi/2 -pi/2 0 0]); % pass through home position
    pause(1);

    moveToQ("Custom", optns, [5/8*pi 0 pi/2.3 -pi/2.3 0 0]); % Green
    doGrip("place", optns);
    pause(1);

    % Return to zone pose via home position - 2 motions
    moveToQ("Custom", optns, [0 0 pi/2 -pi/2 0 0]); % pass through home position
    pause(1);
    moveToQ("Custom",optns,StartZone);

% bBottle1------------------------------------------------------------

    % Hover
    hover = lift(bBottle1, 0.3);
    moveTo(hover, optns);

    % Pick
    bBottle1(3,4) = bBottle1(3,4) + 0.24;
    moveTo(bBottle1, optns);
    pause(1);
    doGrip("pick", optns, 0.515); % HA! Found the value that works!
    pause(1);

    % Place 
    moveTo(hover, optns);

    moveToQ("Custom", optns, [-0.8*pi 0 pi/2.3 -pi/2.3 0 0]); % Blue
    doGrip("place", optns);
    pause(1);

    % Move back to custom
    moveToQ("Custom", optns, StartZone); % pass through home position
    pause(2);
    
% pouches------------------------------------------------------------

    % Hover
    hover = lift(pouchP, 0.3);
    moveTo(hover, optns);

    % Pick
    pouchP(3,4) = pouchP(3,4) + 0.24;
    moveTo(pouchP, optns);
    pause(1);
    doGrip("pick", optns, 0.66); % HA! Found the value that works!
    pause(1);

    % Place 
    moveTo(hover, optns);

    moveToQ("Custom", optns, [-0.8*pi 0 pi/2.3 -pi/2.3 0 0]); % Blue
    doGrip("place", optns);
    pause(1);

    % Hover
    moveTo(hover, optns);

    % Pick
    pouchP(3,4) = pouchP(3,4) - 0.03;
    moveTo(pouchP, optns);
    pause(1);
    doGrip("pick", optns, 0.66); % HA! Found the value that works!
    pause(1);

    % Place 
    moveTo(hover, optns);

    moveToQ("Custom", optns, [-0.8*pi 0 pi/2.3 -pi/2.3 0 0]); % Blue
    doGrip("place", optns);
    pause(1);

    % Hover
    moveTo(hover, optns);

    % Pick
    pouchP(3,4) = pouchP(3,4) - 0.03;
    moveTo(pouchP, optns);
    pause(1);
    doGrip("pick", optns, 0.66); % HA! Found the value that works!
    pause(1);

    % Place 
    moveTo(hover, optns);

    moveToQ("Custom", optns, [-0.8*pi 0 pi/2.3 -pi/2.3 0 0]); % Blue
    doGrip("place", optns);
    pause(1);

    % Hover
    moveTo(hover, optns);

    % Pick
    pouchP(3,4) = pouchP(3,4) - 0.015;
    moveTo(pouchP, optns);
    pause(1);
    doGrip("pick", optns, 0.7); % HA! Found the value that works!
    pause(1);

    % Place 
    moveTo(hover, optns);

    moveToQ("Custom", optns, [-0.8*pi 0 pi/2.3 -pi/2.3 0 0]); % Blue
    doGrip("place", optns);
    pause(1);

    
    
    % move to Zone 1 config
    StartZone = [0.6482   -0.2317    1.4450   -1.2133    0.0003   -0.9226]; % Should not be set manually like this, but via a variable
    moveToQ("Custom",optns,StartZone); 
    pause(5);
        
% gCan1---------------------------------------------------------------
        
    % Hover over
    hover = lift(gCan1, 0.3);
    moveTo(hover, optns);
    
    % Pick
    gCan1(3,4) = gCan1(3,4) + 0.18;
    moveTo(gCan1, optns);
    doGrip("pick", optns, 0.24);
    
    % Place
    moveTo(hover, optns);
    moveToQ("Custom", optns, [5/8*pi 0 pi/2.3 -pi/2.3 0 0]); % Green
    doGrip("place", optns);
  
    % Return to zone pose
    moveToQ("Custom",optns,StartZone); 

% yBottle1------------------------------------------------------------        
    
    % Hover
    hover = lift(yBottle1, 0.3);
    moveTo(hover, optns);

    % Pick
    yBottle1(3,4) = yBottle1(3,4) + 0.14;
    moveTo(yBottle1, optns);
    doGrip("pick", optns, 0.21);
    
    % Return to custom pose
    moveTo(hover, optns);
    moveToQ("Custom", optns, [0 0 pi/2 -pi/2 0 0]); % pass through home position
    
    % Place
    moveToQ("Custom", optns, [-0.8*pi 0 pi/2.3 -pi/2.3 0 0]); % Blue
    doGrip("place", optns);
  
    % Return to zone pose
    moveToQ("Custom",optns,StartZone);         
% rCan1---------------------------------------------------------------
    
    % Hover
    hover = lift(rCan1, 0.3);        
    moveTo(hover, optns);

    % Pick
    rCan1(3,4) = rCan1(3,4) + 0.18;
    moveTo(rCan1, optns);
    doGrip("pick", optns, 0.24);

    % Place
    moveTo(hover, optns);
    moveToQ("Custom", optns, [5/8*pi 0 pi/2.3 -pi/2.3 0 0]); % Green
    doGrip("place", optns);
  
    % Return to zone pose
    moveToQ("Custom",optns,StartZone); 

% rBottle1------------------------------------------------------------

    % Hover
    hover = lift(rBottle1, 0.3);
    moveTo(hover, optns);
    
    % Pick
    rBottle1(3,4) = rBottle1(3,4) + 0.14;
    moveTo(rBottle1, optns);
    doGrip("pick", optns, 0.21);

    % Place
    moveTo(hover, optns);
    moveToQ("Custom", optns, [0 0 pi/2 -pi/2 0 0]); % pass through home position

    moveToQ("Custom", optns, [-0.8*pi 0 pi/2.3 -pi/2.3 0 0]); % Blue
    doGrip("place", optns);

% Spam1------------------SUCCESS----------------------------------

    % Hover
    hover = lift(spam1, 0.3);
    moveTo(hover, optns);
    
    % Pick
    spam1(3,4) = spam1(3,4) + 0.14;
    moveTo(spam1, optns);
    doGrip("pick", optns, 0.325); % could make higher. maybe .33 or so

    % Place
    moveTo(hover, optns);
    moveToQ("Custom", optns, [5/8*pi 0 pi/2.3 -pi/2.3 0 0]); % Green
    doGrip("place", optns);
  
    % Return to zone pose
    moveToQ("Custom",optns,StartZone); 
    
% marker1----------------------------------------------------------

    % Hover
    hover = lift(marker1, 0.3);
    moveTo(hover, optns);
    
    % Pick
    marker1(3,4) = marker1(3,4) + 0.11;
    moveTo(marker1, optns);
    pause(1);
    doGrip("pick", optns, 0.7); % worked in debug but not in trial. double check values
    pause(2);
    % Place
    moveTo(hover, optns);
    moveToQ("Custom", optns, [5/8*pi 0 pi/2.3 -pi/2.3 0 0]); % Green
    doGrip("place", optns);
  
    % Return to zone pose
    moveToQ("Custom",optns,StartZone); 


   

end

