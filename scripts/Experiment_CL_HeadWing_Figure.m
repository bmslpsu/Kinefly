function [] = Experiment_CL_HeadWing_Figure(Fn)
%% Experiment_CL_HeadWing_Figure: runs a experiment using the LED arena
% For Panel Controller v3 and NiDAQ seesion mode
%   INPUT:
%       Fn      : fly #
%---------------------------------------------------------------------------------------------------------------------------------
% rootdir = '/home/jean-michel/bagfiles/Experiment_Wing_CL/'
% Fn = 1;
%---------------------------------------------------------------------------------------------------------------------------------
%% Set directories & experimental paramters %%
%---------------------------------------------------------------------------------------------------------------------------------
% addpath(genpath('/home/jean-michel/Documents/MATLAB/MatlabCodes')) % add controller functions to path
roscmd = 'export LD_LIBRARY_PATH="/home/jean-michel/catkin/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu";';
Panel_com('stop')

% EXPERIMENTAL PARAMETERS
n.exp   = 20 + 2;   % experiment time [s] (add 2 second buffer)
n.rest  = 5;     	% rest time [s]
n.pause = 0.2;  	% pause between panel commands [s]
n.rep   = 5;        % # of repetitions per fly

%% Set Experimental Gain Sequence %%
%---------------------------------------------------------------------------------------------------------------------------------
WG = [0]; % wing gains
HG = [4 8 11]; % head gains
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
[~,~] = system('killall -9 rosmaster'); % kill rosmaster
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
    
    % Start Kinefly with set AO parameters
	[status,~] = system([roscmd 'roslaunch Kinefly main_GAIN.launch WGain:=' num2str(WGain) ' HGain:=' num2str(HGain) ' & echo $!']);
    if status~=0
        error('Kinefly did not launch')
    else
        disp('Kinefly: initialized')
    end
    
    % Rest for 5 seconds while opening Kinefly
    CL_X(5)
    
    % Closed-loop trial
    disp('Closed-loop experiment:')
	Panel_com('stop'); pause(n.pause)
	Panel_com('set_pattern_id', 1);pause(n.pause)                   % set output to p_rest_pat (Pattern_Fourier_bar_barwidth=8)
	Panel_com('set_position',[1, 90]); pause(n.pause)                % set starting position (xpos,ypos)
	Panel_com('set_mode',[1,0]); pause(n.pause)                     % closed loop tracking (NOTE: 0=open, 1=closed)
	Panel_com('send_gain_bias',[-10,0,0,0]); pause(n.pause)         % [xgain,xoffset,ygain,yoffset]
    
    [status,~] = system([roscmd 'roslaunch Kinefly record.launch prefix:=' filename ' time:=' num2str(n.exp) ' & echo $!']);
    if status~=0
        error('Record did not launch')
    else
        disp('Recording...')
    end
    
    % Run experiment
    Panel_com('start');     % start closed-loop
    pause(n.exp)            % experiment time
    Panel_com('stop');    	% stop experiment
    
	% Rest while saving .bag file
	disp('Saving...');
    CL_X(n.exp/2) % wait for .bag to save
    
  	% Kill everything
    [~,~] = system([roscmd 'rostopic pub -1 kinefly/command std_msgs/String exit' '& echo $']); % exit Kinefly
    [status,~] = system('killall -9 rosmaster'); % kill rosmaster

    if status~=0
        error('Kinefly did not exit')
    else
        fprintf('Kinefly: exit \n \n')
    end
    pause(2)
end
fprintf('\n Experiment Done');
toc
end

function [] = CL_X(time)
    n.pause = 0.2;
    Panel_com('stop');
    Panel_com('set_pattern_id', 2); pause(n.pause)
    Panel_com('set_position',[1, 5]); pause(n.pause)
    Panel_com('set_mode', [1,0]); pause(n.pause) % 0=open,1=closed,2=fgen,3=vmode,4=pmode
	Panel_com('send_gain_bias',[-15,0,0,0]); pause(n.pause) % [xgain,xoffset,ygain,yoffset]
	Panel_com('start')
    pause(time)
	Panel_com('stop')
end

function [] = rest(time,rep)
    n.pause = 0.2;
    Panel_com('stop');
    Panel_com('set_pattern_id', 1); pause(n.pause)                	% set pattern
    Panel_com('set_position',[15, 4]); pause(n.pause)               % set starting position (xpos,ypos)
    Panel_com('set_mode', [0,0]); pause(n.pause)                    % 0=open,1=closed,2=fgen,3=vmode,4=pmode
    for kk = 1:rep
        Panel_com('send_gain_bias',[-30,0,0,0]); pause(n.pause)         % [xgain,xoffset,ygain,yoffset]
        Panel_com('start')
        pause(time/(rep*2))
        Panel_com('send_gain_bias',[30,0,0,0]); pause(n.pause)         % [xgain,xoffset,ygain,yoffset]
        pause(time/(rep*2))
        Panel_com('stop')
    end
end