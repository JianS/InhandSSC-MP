%function animation(q_f, hand_end, q_o, param, record)
function animation_wam(q_f, hand_end, param, record)

hf = figure('color','white');
axis equal
grid on
% axis([-2 1.5 -2 1.5]);

% set(gcf, 'Position', get(0,'ScreenSize'));
set(gcf, 'Position', [700 200 900 720]);

% x1 = [x(1)-l_cube/2 x(1)-l_cube/2 x(1)+l_cube/2 x(1)+l_cube/2 x(1)-l_cube/2];
% z1 = [z(1)+l_cube/2 z(1)-l_cube/2 z(1)-l_cube/2 z(1)+l_cube/2 z(1)+l_cube/2];
% ht = line('XData',x1,'YData',z1);
% set(ht,'XDataSource','x1')
% set(ht,'YDataSource','z1')

%%%% parameters of robot
% l_robot = 0.45;
r_ws = param.robot_l1 + param.robot_l2;

t_int = param.t_int;

MUL = 200;

n = size(q_f, 3);

Ifrecord = record; % flag of if recording avi 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% record speed = MUL*FrameRate*t_int
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if Ifrecord
    F(round(n/MUL)+1) = struct('cdata',[],'colormap',[]); 
    j = 1;
end

% theta = zeros()
%Xlow = min(0,min(q_o(1,1,:)))-l;
%Xup = max(q_o(1,1,:))+l;
%Ylow = min(0, min(q_o(2,1,:)))-l;
%Yup = max(q_o(2,1,:))+l; % global view

% [Xlow Xup Ylow Yup]
for i = [1:MUL:n-1 n]
    
    x_f = q_f(1,1,i); y_f = q_f(2,1,i);
    scatter(x_f, y_f,600,'.r','linewidth',1); % contact point

%     axis([-1 1. -0.5 6]); 
    %axis([Xlow Xup Ylow Yup]); 
    axis equal; grid on;
    
    hold on

    title(['Time: ', num2str(i*t_int), ' sec' ], 'FontSize', 15, 'FontWeight', 'bold');
    xlabel('x-axis (m)', 'FontSize', 15, 'FontWeight', 'bold'); 
    ylabel('y-axis (m)', 'FontSize', 15, 'FontWeight', 'bold'); 


    % draw hand   
    plot(hand_end(:,1)', hand_end(:,2)', '-', 'color', 'c'); % hand_end traj
    plot(reshape(q_f(1,1,:),1,[]),reshape(q_f(2,1,:),1,[]), 'r'); % contact pt traj

    x1 = [x_f hand_end(i,1)];
    z1 = [y_f hand_end(i,2)];

    line('XData',x1,'YData',z1, 'LineWidth', 2,'color',[192/255, 137/255, 103/255]);

    scatter(x_f, y_f,30,'.r','linewidth',2); % contact point

    
    %%%% draw robot
    scatter(0,0,30,'r','linewidth',2); % robot origin
%     circle(0,0,r_ws,'k','-.'); % workspace circle
    scatter(hand_end(i,1), hand_end(i,2) ,20,'c','linewidth',2); % hand end point
    
%     [theta1, theta2] = TwoR_InvKin(hand_end(i,1), hand_end(i,2), l_robot, 0);
    [theta1, theta2] = TwoR_InvKin(hand_end(i,1), hand_end(i,2), param);
    elbow = [-param.robot_l1*sin(theta1) param.robot_l1*cos(theta1)];
    endPt = [param.robot_l1*cos(theta1+pi/2)+param.robot_l2*cos(theta1+theta2+pi/2),...
        param.robot_l1*sin(theta1+pi/2)+param.robot_l2*sin(theta1+theta2+pi/2)];
%     xr = [0 elbow(1) hand_end(i,1)];
%     zr = [0 elbow(2) hand_end(i,2)];
    xr = [0 elbow(1) endPt(1)];
    zr = [0 elbow(2) endPt(2)];
    
    line(xr,zr, 'color','k','linewidth',2);
    %%%%
   
    %%% draw hand workspace
%     circle(param.WSlimit_center(1),param.WSlimit_center(2), param.WSlimit_r,'m','-.');
    
    
%     ht = line('XData',x1,'YData',z1, 'color','b');
%     refreshdata(hf,'caller')


    hold off
    drawnow
    
    if Ifrecord
        F(j) = getframe(gcf); j=j+1;
    end
    
%     pause(0.1)
%     pause();

    if (i == 1)
        pause();
    end
   
end

disp(['total cycle# = ' num2str(i) ', Time = ' num2str(i*t_int) ' sec']);


%% write .avi file
if Ifrecord
    vid = VideoWriter('animation2_0.1x.avi');
    vid.Quality = 100;
    vid.FrameRate = 25;
    open(vid);
    for k = 1:size(F,2)
        writeVideo(vid, F(k));
    end
    close(vid);
end


end
