%% Transfer the position of the finger into the WAM workspace

function jp_wam = PosTrans2WAM(JPos, param)

n = size(JPos,1);

jp_wam = zeros(n, 8); % 8 by 1 vector: first element is time, the next 
                       % seven are the joint positions
                
% the offsets are added in Cal3Rangles() in simu_all()
% offset2 = atan(45/550); 
% offset4 = atan(45/300);

for i = 1:n
    
    t_tem = (i-1)*param.t_int;

    jp_wam(i, 1) = t_tem; %time
%     jp_wam(i, 2) = JPos(i,1);   % joint 1
    jp_wam(i, 2) = pi/2;   % joint 1
%     jp_wam(i, 3) = -pi/2; % joint 2
    jp_wam(i, 3) = -JPos(i,1); % joint 2
    jp_wam(i, 4) = 0;
    jp_wam(i, 5) = JPos(i,2); % joint 4
    jp_wam(i, 6) = 0;
    jp_wam(i, 7) = JPos(i,3)-0.074; % joint 6
    jp_wam(i, 8) = 0;
end

% csvwrite('../../data/Traj_wamjp', jp_wam);
% csvwrite('ObjTraj.txt', q_o_wam);

% system('cp ToolTraj.txt ~/Dropbox/wam116/code/In-hand-sliding')
% system('cp ToolTraj.txt ~/Dropbox/wam116/code/In-hand-sliding/data')

end
