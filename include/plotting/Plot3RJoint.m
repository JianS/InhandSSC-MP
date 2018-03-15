function Plot3RJoint(JPos, param)
% JPos = Jtangles;

n = size(JPos, 1);
t_int = param.t_int;
%% plot joint position

figure

fontsize = 12;
set(gcf,'defaulttextinterpreter','latex');
set(gcf,'DefaultAxesFontSize',fontsize);
    
subplot(3,1,1)
hold on
grid on
title('Joint POS plot')

plot((1:n)*t_int, JPos(:,1), 'linewidth',2);
% drawing pos limit 
plot([0, n*t_int],[param.JPLimit(1,1), param.JPLimit(1,1)],'g')
plot([0, n*t_int],[param.JPLimit(1,2), param.JPLimit(1,2)],'g')

% xlabel('time (sec)'); 
ylabel('$\theta_1$ (rad)');
hl1 = legend('planned traj','POS limit');
set(hl1,'interpreter','latex', 'FontSize', fontsize);
% axis tight
axis([0 n*t_int -inf inf])

subplot(3,1,2)
hold on
grid on

plot((1:n)*t_int, JPos(:,2), 'linewidth',2);
% drawing pos limit 
plot([0, n*t_int],[param.JPLimit(2,1), param.JPLimit(2,1)],'g')
plot([0, n*t_int],[param.JPLimit(2,2), param.JPLimit(2,2)],'g')
% xlabel('time (sec)'); 
ylabel('$\theta_2$ (rad)');
axis([0 n*t_int -inf inf])

subplot(3,1,3)
hold on
grid on
plot((1:n)*t_int, JPos(:,3), 'linewidth',2);
% drawing pos limit 
plot([0, n*t_int],[param.JPLimit(3,1), param.JPLimit(3,1)],'g')
plot([0, n*t_int],[param.JPLimit(3,2), param.JPLimit(3,2)],'g')
xlabel('time (sec)'); ylabel('$\theta_3$ (rad)');
axis([0 n*t_int -inf inf])



end