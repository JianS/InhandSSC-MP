function DefParam_planHand
%% Define param_pheters
global param_ph

param_ph.t_int = 0.002;

%%%%%% hand properties
param_ph.l_hand = 0.2; 0.1875;        0.287;   0.248; 0.247; 0.187;

%%%%%% robot properties
param_ph.wamCO = 0.045; % Common Offset of elbow to the center line of wam
param_ph.robot_l1 = norm([0.55, param_ph.wamCO]);
param_ph.robot_l2 = norm([0.3, param_ph.wamCO]);

%%%% 3R robot joint position limit
Jsafe = 0.2;
% param_ph.JPLimit = [-2+Jsafe 2-Jsafe; -0.9+Jsafe 3.1-Jsafe; -1.6+Jsafe 1.6-Jsafe];
param_ph.JPLimit = [-2.6+Jsafe 2.6-Jsafe; -0.9+Jsafe 3.1-Jsafe; -1.6+Jsafe 1.6-Jsafe];
%%%% 3R robot joint velocity limit
J1velMax = 100/42; 100/28;
J2velMax = 100/18;
J3velMax = 192/9.5;
param_ph.JVLimit = [-2 2; -5 5; -20 20];
%%%% 3R robot joint acceleration limit
% 0.9^2/3*10; % estimate rotational interia 
% 0.4^2/3*3.1;
% 0.1^2/3*1;
J1accMax = 1.5*42/3; 1.5*28/3;
J2accMax = 1.5*18/0.2;
J3accMax = 0.35*9.5/0.01;
param_ph.JALimit = [-12 12; -80 80; -100 100];



end
