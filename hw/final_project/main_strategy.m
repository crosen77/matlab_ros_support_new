
%% Main Code to Start ARM Pick and Place.
% Start the ROS connection, handle, goHome, and reset the world.
clear; close all; clc;
optns = startRobotWorld();

% Gray zone 1, easy 

% Can create a flag in optns to choose whether to do static/automated.
optns{'debug'} = true;

staticPickAndPlace(optns);

PickandPlaceARMChallenge('Zone3Pouch', optns);
PickandPlaceARMChallenge('Zone3', optns);

PickandPlaceARMChallenge('Zone4', optns);

% Blue zone 5, very hard
%PickandPlaceARMChallenge('Zone5', optns);
