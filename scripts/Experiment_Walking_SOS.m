function [] = Experiment_Walking_SOS(Fn)
%% Experiment_Walking_Chirp: runs a experiment using the LED arena
% Forr Panel Controller v3 and NiDAQ seesion mode
%   INPUT:
%       Fn      : fly #
%---------------------------------------------------------------------------------------------------------------------------------
% Fn = 1;
%---------------------------------------------------------------------------------------------------------------------------------
%% Set directories & experimental paramters %%
%---------------------------------------------------------------------------------------------------------------------------------
rootdir = '/media/jean-michel/Data_Audrey/Walking_Experiments/SOS';
roscmd = 'export LD_LIBRARY_PATH="/home/jean-michel/catkin/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu";';

% EXPERIMENTAL PARAMETERS
n.exp   = 20 + 2;   % experiment time [s] (add 2 second buffer)
n.rest  = 5;     	% rest time [s]
n.pause = 0.4;  	% pause between panel commands [s]
n.rep   = 10;     	% # of repetitions per fly


%% Kinefly Setup %%
%---------------------------------------------------------------------------------------------------------------------------------
system('killall -9 rosmaster'); % kill rosmaster
tic
fprintf('Begin Experiment \n \n')

% Open Kinefly  to set fly
system([roscmd 'roslaunch Kinefly main.launch' ' & echo $!']);

% Set gain parameters
pause(1)
system([roscmd 'rosrun dynamic_reconfigure dynparam set /kinefly/flystate2phidgetsanalog v1ha ' num2str(5) '& echo $!']);
pause(1)
disp('Focus fly in camera; press any key continue')
pause

% Change Kinefly  ROI
    system([roscmd 'rosrun dynamic_reconfigure dynparam set /camera/camera_nodelet format7_roi_width ' num2str( 500) '& echo $!']);
  	pause(1)
    system([roscmd 'rosrun dynamic_reconfigure dynparam set /camera/camera_nodelet format7_roi_height ' num2str( 300) '& echo $!']);    
    pause(1)
    system([roscmd 'rosrun dynamic_reconfigure dynparam set /camera/camera_nodelet format7_x_offset ' num2str(94) '& echo $!']);    
    pause(1)
    system([roscmd 'rosrun dynamic_reconfigure dynparam set /camera/camera_nodelet format7_y_offset ' num2str( 74) '& echo $!']);    
    pause(1)
disp('Center masks on fly; press any key continue')
pause
    
%% Run Experiment %%
%-----------------------------------------------------------------------------------------------------------------------------
for kk = 1:n.rep
    % Set gains & filename
  	filename = ['fly_' num2str(Fn) '_trial_' num2str(kk)];
    
    % Print counter to command line
    disp(['Trial ' num2str(kk) ': ' filename ]) 
    disp('-------------------------------------------------------')
    
    % Chirp trial
	Panel_com('stop'); pause(n.pause)
	Panel_com('set_pattern_id', 2); pause(n.pause)                	% set pattern
	Panel_com('set_position',[7, 5]); pause(n.pause)                % set starting position (xpos,ypos)
    Panel_com('set_posfunc_id', [1,1]);pause(n.pause)
	Panel_com('set_funcX_freq', 200); pause(n.pause)                % update rate for x-channel
    Panel_com('set_funcY_freq', 50); pause(n.pause)              	% update rate for y-channel
	Panel_com('set_mode',[4,0]); pause(n.pause)                     % 0=open,1=closed,2=fgen,3=vmode,4=pmode
	
    disp('Play Stimulus')               
    
    system([roscmd 'roslaunch Kinefly record.launch prefix:=' filename ...
        ' time:=' num2str(n.exp) ' root:=' num2str(rootdir) ' & echo $!']);
    pause(2) % buffer
    
    % Run experiment
    Panel_com('start');     % start
    pause(n.exp)            % experiment time
    Panel_com('stop');    	% stop experiment
    
	% Rest while saving .bag file
	fprintf('Saving...\n\n');
    Arena_CL(2,'x',-15)
    pause(n.exp/2)
    Panel_com('stop');
    
end
% Kill everything
system([roscmd 'rostopic pub -1 kinefly/command std_msgs/String exit' '& echo $']); % exit Kinefly
system('killall -9 rosmaster'); % kill rosmaster
fprintf('\n Experiment Done: ');
toc
end