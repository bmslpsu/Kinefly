function [] = Experiment_CL_HeadWing_Figure(Fn)
%% Experiment_CL_HeadWing_Figure: runs a experiment using the LED arena
% For Panel Controller v3 and NiDAQ seesion mode
%   INPUT:
%       Fn      : fly #
%---------------------------------------------------------------------------------------------------------------------------------
Fn = 1;
%---------------------------------------------------------------------------------------------------------------------------------
%% Set directories & experimental paramters %%
%---------------------------------------------------------------------------------------------------------------------------------
roscmd = 'export LD_LIBRARY_PATH="/home/jean-michel/catkin/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu";';
rootdir = '/home/jean-michel/Experiment_Data/TEST';

% EXPERIMENTAL PARAMETERS
n.exp   = 20 + 2;   % experiment time [s] (add 2 second buffer)
n.rest  = 5;     	% rest time [s]
n.pause = 0.2;  	% pause between panel commands [s]
n.rep   = 5;        % # of repetitions per condition per fly

%% Set Experimental Gain Sequence %%
%---------------------------------------------------------------------------------------------------------------------------------
WG = [1]; % wing gains
HG = [2]; % head gains
Gain = nan(length(unique(WG))*length(unique(HG)),2);
pp = 1;
for kk = 1:length(WG)
    for jj = 1:length(HG)
        Gain(pp,1) = WG(kk);
        Gain(pp,2) = HG(jj);
        pp = pp + 1;
    end
end
Gain_rand = Gain(randperm(size(Gain,1)),:); % reshuffle randomly
Gain_all = repmat(Gain_rand,n.rep,1); % repeat for n.rep
n.trial = length(Gain_all);

%% EXPERIMENT LOOP %%
%---------------------------------------------------------------------------------------------------------------------------------
system('killall -9 rosmaster'); % kill rosmaster
% Panel_com('reset',0)

% Start Kinefly with set AO parameters
system([roscmd 'roslaunch Kinefly main.launch' ' & echo $!']);
pause(3)

tic
fprintf('Begin Experiment \n \n')
for kk = 1:n.trial
    % Set gains & filename
	WGain = Gain_all(kk,1);
	HGain = Gain_all(kk,2);
    
  	filename = ['fly_' num2str(Fn) '_trial_' num2str(kk) '_HGain_' num2str(HGain) '_WGain_' num2str(WGain)];
    
    % Print counter to command line
    disp(['Trial ' num2str(kk) ': ' filename ]) 
    disp('-------------------------------------------------------')
    
    % Set gain parameters
    system([roscmd 'rosrun dynamic_reconfigure dynparam set /kinefly/flystate2phidgetsanalog v1l1 ' num2str( WGain) '& echo $!']);
  	system([roscmd 'rosrun dynamic_reconfigure dynparam set /kinefly/flystate2phidgetsanalog v1r1 ' num2str(-WGain) '& echo $!']);
  	system([roscmd 'rosrun dynamic_reconfigure dynparam set /kinefly/flystate2phidgetsanalog v1ha ' num2str( HGain) '& echo $!']);

    % Rest while opening Kinefly
    Arena_CL(2,'x',-15)
    pause(3)
    
    % Closed-loop trial
    disp('Closed-loop experiment:')
	Panel_com('stop'); pause(n.pause)
	Panel_com('set_pattern_id', 1);pause(n.pause)                   % set output to p_rest_pat (Pattern_Fourier_bar_barwidth=8)
	Panel_com('set_position',[1, 40]); pause(n.pause)           	% set starting position (xpos,ypos)
	Panel_com('set_mode',[1,0]); pause(n.pause)                     % closed loop tracking (NOTE: 0=open, 1=closed)
	Panel_com('send_gain_bias',[-10,0,0,0]); pause(n.pause)         % [xgain,xoffset,ygain,yoffset]
    
  	system([roscmd 'roslaunch Kinefly record.launch prefix:=' filename ...      % start recording
        ' time:=' num2str(n.exp) ' root:=' num2str(rootdir) ' & echo $!']);
    
    % Run experiment
    Panel_com('start');     % start closed-loop
    pause(n.exp)            % experiment time
    Panel_com('stop');    	% stop experiment
    
	% Rest while saving .bag file
	disp('Saving...');
    Arena_CL(2,'x',-15)
    pause(n.exp/2)
	Panel_com('stop')
end
% Kill everything
system([roscmd 'rostopic pub -1 kinefly/command std_msgs/String exit' '& echo $']); % exit Kinefly
system('killall -9 rosmaster'); % kill rosmaster
fprintf('\n Experiment Done');
toc
end