% clc
close all
% clear

%%

% targetPath = '../../../data/MPI/';

addpath('../include/');		
addpath('../include/plotting');


targetPath = '../../../data/';

plan_res = load([targetPath 'plan_data.mat']); % plan result MPI class

n_finger = plan_res.MPI.param.n_f;

%% load p_f_H from ros bag

rosbag_path = [targetPath 'test.bag'];

[t_bag, p_f_H_raw] = GetFingerPosFromBag(rosbag_path);

n_bag = length(t_bag);

ss = 20;
p_f_H_filtered = p_f_H_raw;
for i = 1:n_finger
    p_f_H_filtered(:,:,i) = AverageFilter(p_f_H_raw(:,:,i),ss);
end

%% load camera data
obj_raw = load([targetPath 'cam_obj']);

hand_raw = load([targetPath 'cam_hand']);

% plan_data = load([targetPath 'plan_data.mat']);

n_cam = size(obj_raw,1);

n_cam = min([n_cam, plan_res.MPI.n]);

q_o_cam = zeros(3,n_cam);
q_h_cam = zeros(3,n_cam);
p_f_H_exp = zeros(2,n_cam,n_finger);
t_cam = obj_raw(1:n_cam,1);

ssp = 20;
i_ros = 1;

cut_start = 2;
cut_end = 0;
q_h_x_correct = -0.007;


obj_data = load('../newobj_data.mat');
obj_height = obj_data.h;
q_o_y_shift = -obj_height-0.016; % shift due to markers are not at the bottom of the bottle
for i = 1:n_cam
    if i < cut_start || i > cut_end
        qobj = quaternion(obj_raw(i,5:8));
        q_o_cam(3,i) = [1 0 0]*qobj.EulerAngles('123');
    %     Ro = qobj.RotationMatrix();
    %     obj_pt = obj_raw(i,2:4)' + Ro*[0; 0; q_o_y_shift];
    %     q_o_cam(1,i) = -obj_pt(2);
    %     q_o_cam(2,i) = obj_pt(3);

        q_o_cam(1,i) = -obj_raw(i,3);
        q_o_cam(2,i) = obj_raw(i,4);
    else
        q_o_cam(1:2,i) = q_o_cam(1:2,cut_start-1) + random('norm',0,0.0001,[2,1]);
%         q_o_cam(3,i) = q_o_cam(3,cut_start) + random('norm',0,0.002,[1,1]);
        q_o_cam(3,i) = random('norm',0,0.002,[1,1]);
    end
    
    q_h_cam(1,i) = -hand_raw(i,3)+q_h_x_correct;
    q_h_cam(2,i) = hand_raw(i,4);
    qhand = quaternion(hand_raw(i,5:8));
    q_h_cam(3,i) = [1 0 0]*qhand.EulerAngles('123') - pi/2;
    
    if t_cam(i)>t_bag(i_ros)
        i_ros = i_ros +1;
    end
    
    pmin = max([1 i_ros-ssp/2]);
    pmax = min([n_bag i_ros+ssp/2]);
    
    for j = 1:n_finger
        p_x = polyfit(t_bag(pmin:pmax)', p_f_H_filtered(3,pmin:pmax,j), 1);
        p_f_H_exp(1,i,j) = -p_x(1)*t_cam(i) - p_x(2);
        p_y = polyfit(t_bag(pmin:pmax)', p_f_H_filtered(2,pmin:pmax,j), 1);
        p_f_H_exp(2,i,j) = p_y(1)*t_cam(i) + p_y(2);
    end
end


q_h_cam(:,n_cam)-q_h_cam(:,1)
plan_res.MPI.q_h(:,n_cam)-plan_res.MPI.q_h(:,1)

%% Calculate finger contact pos from exp data: hand pos from vision, finger pos from hand

% define the hand and object motion
q_o_exp = AverageFilter(q_o_cam, 500); % object pos
q_h_exp = AverageFilter(q_h_cam, 200); % palm pos

dq_o_exp = zeros(3, n_cam);
dq_h_exp = zeros(3, n_cam);



if (0)
SimplePlot3(t_cam, q_o_cam, 'Object Pos from vision w.r.t. W frame')
SimplePlot3(t_cam, q_h_cam, 'Hand Pos from vision w.r.t. W frame')

SimplePlot3(t_cam, q_o_exp, 'Filtered Object Pos from vision w.r.t. W frame')
SimplePlot3(t_cam, q_h_exp, 'Filtered Hand Pos from vision w.r.t. W frame')

fingerNum = 1;
SimplePlot3(t_bag, p_f_H_raw(:,:,fingerNum), 'Raw fingertip Pos from ros w.r.t. H frame')
SimplePlot3(t_bag, p_f_H_filtered(:,:,fingerNum), 'Filtered fingertip Pos from ros w.r.t. H frame')

SimplePlot3(t_cam, p_f_H_exp(:,:,fingerNum), 'Second Filtered fingertip Pos from ros w.r.t. H frame')
end






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define anchor position in the Hand frame H
p_a_H = zeros(2, n_cam, n_finger);
p_a = zeros(2, n_cam, n_finger);
dp_a_H = zeros(2, n_cam, n_finger);
dp_a = zeros(2, n_cam, n_finger);

p_a_H_init(:,1) = [-0.07; 0.08]; [-0.05; 0.06];
p_a_H_init(:,2) = [0.045; 0.08]; [0.025; 0.06];
% p_a_H_init(:,3) = [-0.024; -0.04];

% define equilibrim point in P, as specified in allegro hand param.yaml
p_e_H = zeros(2, n_cam, n_finger);
p_e_H_init(:,1) = [-0.02; 0.04]; [-0.05; 0.016];
p_e_H_init(:,2) = [-0.00; 0.04]; [0.025; 0.020];
% p_e_H_init(:,3) = [-0.024; -0.005];

% define contact position
p_f_exp = zeros(2,n_cam, n_finger);
p_f_exp_proj = zeros(2,n_cam, n_finger);
p_f_B_exp = zeros(2,n_cam, n_finger);
p_f_B_exp_proj = zeros(2,n_cam, n_finger);
p_f_H_exp_proj = zeros(2,n_cam, n_finger);
s_exp = zeros(n_cam, n_finger);

% load('../obj_data.mat');
% obj_height = abs(min(obj_pts(2,:)));
% % shift the bottle pts up so that the origin of the object frame is at the
% % bottom
% obj_pts(2,:) = obj_pts(2,:) + obj_height; % left curve of the coke bottle
% obj_pts_plot(2,:) = obj_pts_plot(2,:) + obj_height; % full shape for plot in B frame

ssp = 200;
for i = 1:n_cam
%     if i==1
%         dq_o(:,i) = (q_o(:,i+1)-q_o(:,i))/param.t_int;
%         dq_h(:,i) = (q_h(:,i+1)-q_h(:,i))/param.t_int;
%     elseif i == n_cam
%         dq_o(:,i) = (q_o(:,i)-q_o(:,i-1))/param.t_int;
%         dq_h(:,i) = (q_h(:,i)-q_h(:,i-1))/param.t_int;
%     else
%         dq_o(:,i) = (q_o(:,i+1)-q_o(:,i-1))/(2*param.t_int);
%         dq_h(:,i) = (q_h(:,i+1)-q_h(:,i-1))/(2*param.t_int);
%     end
    
    % get obj and hand vel with constant vel filter
    pmin = max([1 i-ssp/2]);
    pmax = min([n_cam i+ssp/2]);
    for j = 1:3
        p_o = polyfit(t_cam(pmin:pmax)', q_o_exp(j, pmin:pmax), 1);
        dq_o_exp(j,i) = p_o(1);
        p_h = polyfit(t_cam(pmin:pmax)', q_h_exp(j, pmin:pmax), 1);
        dq_h_exp(j,i) = p_h(1);
    end
    
    % let p_o represent the bottom center (p_o_exp is at the center a little
    % above the bottom since the marker location)
    R_o = CalR2d(q_o_exp(3,i));
    q_o_exp(1:2, i) = q_o_exp(1:2,i) + R_o*[0; q_o_y_shift];    
    
    for j = 1:n_finger
        % equilibrim points
        p_e_H(:,i,j) = p_e_H_init(:,j);
        % calculate anchor pos and vel
        p_a_H(:,i,j) = p_a_H_init(:,j);
        R_h = CalR2d(q_h_exp(3,i));
        thetaP = q_h_exp(3,i); dthetaP = dq_h_exp(3,i);
        dR_h = [-sin(thetaP) -cos(thetaP); cos(thetaP) -sin(thetaP)]*dthetaP;
        p_a(:,i,j) = q_h_exp(1:2,i) + R_h*p_a_H(:,i,j);      
        dp_a(:,i,j) = dq_h_exp(1:2,i) + dR_h*p_a_H(:,i,j) + R_h*dp_a_H(:,i,j);
        
        % Calculate contact position
        p_f_exp(:,i,j) = q_h_exp(1:2,i) + R_h*p_f_H_exp(:,i,j);
        
        p_f_B_exp(:,i,j) = R_o'*( p_f_exp(:,i,j) - q_o_exp(1:2,i) );
        
       
        % project contact position to the edge (here just project horizontally...)

%         p_f_B_exp_proj(:,i,j) = p_f_B_exp(:,i,j);
%         p_f_B_exp_proj(:,i,j) = obj_pts(:, LookupObjPt( p_f_B_exp(2,i,j), obj_pts ) );
%         if j==2
%             p_f_B_exp_proj(1,i,j) = -p_f_B_exp_proj(1,i,j);
%         end
        p_f_B_exp_proj(:,i,j) = GetPtOnObj(p_f_B_exp(2,i,j), j, obj_data);
        
        p_f_exp_proj(:,i,j) = q_o_exp(1:2,i) + R_o*p_f_B_exp_proj(:,i,j);
        p_f_H_exp_proj(:,i,j) = R_h'*(p_f_exp_proj(:,i,j) - q_h_exp(1:2,i));
    end 
end

% %
% DefParam_EF
% global param_EF
% 
% param_EF.N = n_cam;
% param_EF.T = param_EF.t_int*param_EF.N;
% 
% param_EF.d0(:,1) = (p_e_H_init(:,1) - p_a_H_init(:,1))'; [0.07 -0.1];
% param_EF.d0(:,2) = (p_e_H_init(:,2) - p_a_H_init(:,2))'; [-0.07 -0.1];

if (0)
% animation the experiment data
% animation(q_o, q_h, p_f_exp, p_f_H_exp, p_a, p_a_H, param);
% animation(q_o, q_h, p_f_exp_proj, p_f_H_exp_proj, p_a, p_a_H, param);
addpath('../estimateFriction/')

animation_EF(q_o_exp, q_h_exp, p_f_exp, p_f_H_exp, p_a, p_a_H, [], [], [], [],...
    plan_res.MPI.param, obj_data.obj_pts_plot, 0, ' ', 50)

animation_EF(q_o_exp, q_h_exp, p_f_exp_proj, p_f_H_exp_proj, p_a, p_a_H, [], [], [], [],...
    plan_res.MPI.param, obj_data.obj_pts_plot, 0, 'data', 50)

% animation_EF(q_o, q_h, p_f_exp_proj, p_f_H_exp_proj, p_a, p_a_H, [], [], [], [], param_EF, obj_pts_plot, 1, 'data', 20)

% show finger contact relative location on the contact edges
% SimplePlot3(t_cam, s_exp', 'Exp finger contact relative location on edges "s_i" ')

SimplePlot3(t_cam, q_o_cam, 'Object Pos from vision w.r.t. W frame')
SimplePlot3(t_cam, q_h_cam, 'Hand Pos from vision w.r.t. W frame')

SimplePlot3(t_cam, q_o_exp, 'Object Pos from vision in W frame (smoothed)')
SimplePlot3(t_cam, q_h_exp, 'Hand Pos from vision in W frame (smoothed)')

SimplePlot3(t_cam, dq_o_exp, 'Object Vel from vision w.r.t. W frame')
SimplePlot3(t_cam, dq_h_exp, 'Hand Vel from vision w.r.t. W frame')
end


%% animation
speed = 100;
fontsize = 15;
forceScalar = 120;
Ifrecord = 0;
videoname = 'resultAll';

animationAll_MPI(plan_res, n_cam, q_o_exp, q_h_exp, p_f_exp_proj,...
    p_f_H_exp_proj, speed, fontsize, forceScalar, Ifrecord, videoname)

 %% Plot finger end pos trajes
plotAllPfB(plan_res, n_cam, p_f_B_exp_proj, fontsize)

%% Calculate final error in pfB
err = p_f_B_exp_proj(:,end,:) - plan_res.MPI.p_f_B(:,end,:);
total_travel_dis = zeros(plan_res.MPI.n_f,1);
for i = 1:plan_res.MPI.n-1
    for j = 1:plan_res.MPI.n_f
        total_travel_dis(j) = total_travel_dis(j) + ...
            norm(plan_res.MPI.p_f_B(:,i+1,j)-plan_res.MPI.p_f_B(:,i,j));
    end
end
norm(err(:,:,1))
norm(err(:,:,2))
%% save or load previous results
if (0)
    %% save
    filename = [targetPath 'dataAll'];
    if exist([filename '.mat'], 'file')
        filename = [filename '2']
    end
    save(filename, 'q_o_exp','q_h_exp','p_f_exp','p_f_exp_proj',...
        'p_f_H_exp','p_f_H_exp_proj','p_f_B_exp','p_f_B_exp_proj');
    
    %% load
    addpath('../include/');		
    addpath('../include/plotting');

%     targetPath = '../../../data/MPI/062117_good/';
    targetPath = '../../../data/new-obj/MPI/080717_2/';
    plan_res = load([targetPath 'plan_data.mat']); % plan result MPI class
    load([targetPath 'dataAll.mat'])
    
    n_cam = length(p_f_B_exp)
    plotAllPfB(plan_res, n_cam, p_f_B_exp_proj, 9)

    %%
    PlotInit(plan_res.MPI, pfH_init, fontsize, forceScalar, [])
    animationMPI(plan_res.MPI, speed, 10, forceScalar, 'twoplots', 0, '')
end




