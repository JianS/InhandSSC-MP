function CalWamInit(q_h_init_wam, q_h_init_angle)

DefParam_planHand							
global param_ph

% T_total = param_MPI.T;

% n = param_MPI.N;

q_h_ref = zeros(3,1,2);

% q_f_init = [0.; 0.98; 0.0];
q_h_ref(1,1,1) = -q_h_init_wam(3);
q_h_ref(2,1,1) = q_h_init_wam(2);
q_h_ref(3,1,1) = q_h_init_angle;
q_h_ref(:,1,2) = q_h_ref(:,1,1);
%Generate WAM data
hand_end = Calhandend(q_h_ref, param_ph);
JPos = Cal3Rangles(q_h_ref, hand_end, param_ph);

% prompt = 'Show planned traj? (1/0)';			
% answer = input(prompt);
% 
% if (answer == 1)
%     animation_wam(q_h_ref ,hand_end, param_ph,0);
%     Plot3RJoint(JPos, param_ph);
% end        
%% Save planned traj
% plan_data.q_f_ref = q_f_ref;
% plan_data.n = n;

% save('../../../data/plan_data.mat','q_h_ref','n')

%%
jpWAM = PosTrans2WAM(JPos, param_ph);

csvwrite('../../../data/Init_wamjp', jpWAM);


end