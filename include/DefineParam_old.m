function DefineParam
%% Define parameters
global param

param.t_int = 0.002;

%%%%%% object properties
param.l = 0.12;  0.14; 0.1;
param.w = 0.09;  0.105; 0.06;

param.m_obj = 0.04;  0.0554; 0.0245; .023; % mass for the object
param.m_s = 0.009;  0.0063; 0.02; 0.010; 0.0012; % mass of the markers/weights at the corners
param.m = param.m_obj + param.m_s;
param.I = 1/12*param.m_obj*(param.l^2+param.w^2) + param.m_s*(param.l^2+param.w^2);
param.g = 0; -9.8;

param.G = -9.8*param.m; % gravity force of the object
param.Ls = 0.05;
param.L = 0.17;
param.S = -7*param.Ls/param.L;

param.M = [param.m, 0, 0; 0, param.m, 0; 0, 0, param.I];
param.M_inv = inv(param.M);

%%%%%% hand properties
param.l_hand = 0.1875;        0.287;   0.248; 0.247; 0.187;

%%%%%% robot properties
param.wamCO = 0.045; % Common Offset of elbow to the center line of wam
param.robot_l1 = norm([0.55, param.wamCO]);
param.robot_l2 = norm([0.3, param.wamCO]);

%%%%%% friction properties
param.mu = 0.065; 0.5; 0.38;  0.6/1.56;  % friction coefficient
% param.N = 0.23; 0.23; 0.38; 0.7; 1.25; % normal contact force, here assume N is constant

param.N1 = abs(param.S);
param.N2 = abs(param.G+param.S);

param.c = 0.6; 0.6; 0.635; % constant for calculating the soft finger moment
param.a = 0.0254; 0.023; 0.014;0.0174; % radius of the contact patch

param.Ft1 = param.mu*param.N1; % max tangent friction force
param.Ft2 = param.mu*param.N2;
param.Mn1 = param.c*param.a*param.mu*param.N1; % max normal moment
param.Mn2 = param.c*param.a*param.mu*param.N2;

param.stick_guard = -0.4; 0.4; % min distance (from inside) to the limit surface that will be regard as outside 


%%%% finger acc limit
% param.AccLimit = [-20 20; -30 20; -100 100]; % [Xmin Xmax; Ymin Ymax; AngularMin AngularMax]
param.AccLimit = [-7 7; -7 7; -50 50]; % [Xmin Xmax; Ymin Ymax; AngularMin AngularMax]
%%%% finger workspace limit, here defined as a circle
param.WSlimit_center = [0; 0.55];
param.WSlimit_r = 0.3;
ang = 0.3*pi;
ang_center = pi-pi/12;
param.WSlimit_angle = [ang_center-ang; ang_center+ang];
%%%% finger vel limit
param.VelLimit = [-2 2; -2 2; -4 4];


%%%% 3R robot joint position limit
Jsafe = 0.2;
% param.JPLimit = [-2+Jsafe 2-Jsafe; -0.9+Jsafe 3.1-Jsafe; -1.6+Jsafe 1.6-Jsafe];
param.JPLimit = [-2.6+Jsafe 2.6-Jsafe; -0.9+Jsafe 3.1-Jsafe; -1.6+Jsafe 1.6-Jsafe];
%%%% 3R robot joint velocity limit
J1velMax = 100/42; 100/28;
J2velMax = 100/18;
J3velMax = 192/9.5;
param.JVLimit = [-2 2; -5 5; -20 20];
%%%% 3R robot joint acceleration limit
% 0.9^2/3*10; % estimate rotational interia 
% 0.4^2/3*3.1;
% 0.1^2/3*1;
J1accMax = 1.5*42/3; 1.5*28/3;
J2accMax = 1.5*18/0.2;
J3accMax = 0.35*9.5/0.01;
param.JALimit = [-12 12; -80 80; -100 100];



end
