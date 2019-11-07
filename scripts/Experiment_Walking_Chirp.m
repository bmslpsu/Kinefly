function [] = Experiment_Walking_Chirp(Fn)
%% Experiment_Walking_Chirp: runs a experiment using the LED arena
% Forr Panel Controller v3 and NiDAQ seesion mode
%   INPUT:
%       Fn      : fly #
%---------------------------------------------------------------------------------------------------------------------------------
% Fn = 1;
%---------------------------------------------------------------------------------------------------------------------------------
%% Set directories & experimental paramters %%
%---------------------------------------------------------------------------------------------------------------------------------
rootdir = '/home/jean-michel/Experiment_Data/Experiment_Walking_SOS';
roscmd = 'export LD_LIBRARY_PATH="/home/jean-michel/catkin/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu";';

% EXPERIMENTAL PARAMETERS
n.exp   = 20 + 2;   % experiment time [s] (add 2 second buffer)
n.rest  = 5;     	% rest time [s]
n.pause = 0.2;  	% pause between panel commands [s]
n.rep   = 5;        % # of repetitions per fly

%% Set Experimental Amplitude Sequence %%
%---------------------------------------------------------------------------------------------------------------------------------
Amp = 3.75*[ 2 3 4 5 ];
n.Amp = length(Amp);

% Repeat randomized cycle for n reptitions
Amp_ALL = [];
for kk = 1:n.rep
    Amp_ALL = [Amp_ALL , Amp(randperm(n.Amp))];
end
Amp_ALL = transpose(Amp_ALL);

% Create position function index to load fucntion
func_ALL = zeros(n.Amp*n.rep,1);
Amp_idx = [4 1 2 3]; % Create index from loaded function position in PControl gui
for jj = 1:n.Amp
    func_ALL(Amp_ALL == Amp(jj)) = Amp_idx(jj);
end
disp('Amplitude Map:')
disp(Amp_ALL)

%% Kinefly Setup %%
%---------------------------------------------------------------------------------------------------------------------------------
system('killall -9 rosmaster'); % kill rosmaster
tic
fprintf('Begin Experiment \n \n')

% Open Kinefly  to set fly
system([roscmd 'roslaunch Kinefly main.launch' ' & echo $!']);
disp('Focus fly in camera; press any key continue')
pause

% Change Kinefly  ROI
disp('Center masks on fly; press any key continue')
pause
    
%% Run Experiment %%
%-----------------------------------------------------------------------------------------------------------------------------
for kk = 1:n.Amp*n.rep
    % Set gains & filename
  	filename = ['fly_' num2str(Fn) '_trial_' num2str(kk) '_Amp_' num2str(Amp_ALL(kk,1))];
    
    % Print counter to command line
    disp(['Trial ' num2str(kk) ': ' filename ]) 
    disp('-------------------------------------------------------')
    
    % Chirp trial
	Panel_com('stop'); pause(n.pause)
	Panel_com('set_pattern_id', 2); pause(n.pause)                	% set pattern
	Panel_com('set_position',[8, 4]); pause(n.pause)                % set starting position (xpos,ypos)
    Panel_com('set_posfunc_id',[1, func_ALL(kk)]); pause(n.pause)   % arg1 = channel (x=1,y=2); arg2 = funcid
	Panel_com('set_funcX_freq', 200); pause(n.pause)                % update rate for x-channel
    Panel_com('set_funcY_freq', 50); pause(n.pause)              	% update rate for y-channel
	Panel_com('set_mode',[4,0]); pause(n.pause)                     % 0=open,1=closed,2=fgen,3=vmode,4=pmode
	
    disp(['Play Stimulus: Amplitude = ' num2str(Amp_ALL(kk))])              
    
    system([roscmd 'roslaunch Kinefly record.launch prefix:=' filename ...
        ' time:=' num2str(n.exp) ' root:=' num2str(rootdir) ' & echo $!']);
    pause(2) % buffer
    
    % Run experiment
    Panel_com('start');     % start
    pause(n.exp)            % experiment time
    Panel_com('stop');    	% stop experiment
    
	% Rest while saving .bag file
	fprintf('Saving...\n\n');
    Arena_CL(1,'x',-15)
    pause(n.exp/2)
    Panel_com('stop');
    
end
% Kill everything
system([roscmd 'rostopic pub -1 kinefly/command std_msgs/String exit' '& echo $']); % exit Kinefly
system('killall -9 rosmaster'); % kill rosmaster
fprintf('\n Experiment Done');
toc
end