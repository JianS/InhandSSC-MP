clc
clear

close all 

%%
% This is a simulation for in-hand sliding on a Costomized object
% w/ spring-sliding compliant fingers
% In this program we are solving the motion planning using the inverse
% contact mechanics. The input is the initial and goal grasp, and outcome
% is the anchor motions. 

addpath('include/');	
addpath('include/plotting');

DefParam_MPI
global param

obj_data = load('newobj_data.mat');

% defining the external contact pts pos and normal direction
for j = 1:param.n_f
    param.p_e_B(:,j) = GetPtOnObj(0, j, obj_data);
    param.nhat_e_B(:,j) = [0;1];
end

param.p_g_B = obj_data.obj_center';
param.p_g_B(2) = 0.15;

%% define initial and goal

% the object is fixed 
q_o = zeros(3, param.N); % object pos
dq_o = zeros(3, param.N); % object vel

% define initial conditions
q_o_init = [1; 0; 0]; % object initial pose
for i = 1:param.N
    q_o(:,i) = q_o_init;
end

q_h_init = [1.0; 0.23; 0]; % hand initial pose

yfB_init = [0.17; 0.18]; % initial fingertip y-pos in object frame


% define goal fingertip y-pos
yfB_goal = [0.05; 0.02];


%%

fontsize = 15;

% path to the file storing pre-calculated Feasible Finger Pos Map
FFPmapPath = 'FFPmap--1.mat';

MPI = MotionPlanInverse(param, q_o, dq_o, q_h_init, yfB_init, yfB_goal,...
    obj_data, FFPmapPath);

[ret, MPI] = CheckInit(MPI);

forceScalar = 120;
PlotInit(MPI, [], fontsize, forceScalar, [])

if ret == 0
    error('Initial ill conditioned. Stopped.');
end

MPI = BuildSticking(MPI);

if ~exist(FFPmapPath,'file')
    %%
    disp(['File "' FFPmapPath '" is not found. Building the map now...']);
    
    stepsize = 0.002;
    MPI = BuildFeasiFingerPosMap(MPI, stepsize);
    MPI = FindMostRobustCurve(MPI);
    
    out.Iffeasi = MPI.Iffeasi;
    out.Ystepsize = MPI.Ystepsize;
    out.y1 = MPI.y1;
    out.y2 = MPI.y2;
    out.dy = MPI.dy;
    out.feasiBoundary = MPI.feasiBoundary; 
    out.MRC = MPI.MRC;
    out.l_MRC = MPI.l_MRC;
    out.param = param;
    
    save(FFPmapPath, '-struct','out')
end

MPI = PlanSlidingTraj(MPI);
PlotPath(MPI, fontsize, [])

%% animation

speed = 300; % speed of playback, smaller number means slower

animationMPI(MPI, speed, 15, forceScalar, 'twoplots', 0, '')
