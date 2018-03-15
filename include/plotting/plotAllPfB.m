function plotAllPfB(plan_res, N, pfBexp, fontsize)



figure()

set(gcf,'defaulttextinterpreter','latex');
set(gcf,'DefaultAxesFontSize',fontsize);

w = 4;
h = 2.5;
set(gcf,'Units','inches','PaperSize',[w h],'PaperPositionMode','auto', 'Position', [3 4 w h])


pfB = plan_res.MPI.p_f_B;
param = plan_res.MPI.param;

colors = colormap(lines(param.n_f));


subplot(2,1,1)
hold on 
grid on
set(gca,'TickLabelInterpreter','latex')
set(gca,'DefaultAxesFontSize',fontsize);


plot((1:N)*param.t_int, pfB(1,1:N,1), '-', 'color', colors(1,:));
plot((1:N)*param.t_int, pfBexp(1,1:N,1), '--', 'color', colors(1,:));

plot((1:N)*param.t_int, pfB(1,1:N,2),'-', 'color', colors(2,:));
plot((1:N)*param.t_int, pfBexp(1,1:N,2), '--', 'color', colors(2,:));

% xlabel('time (sec)')
ylabel('$x_f^\mathcal{B}$ (m)')

title(['$\mu = ' num2str(param.mu) '$'])


subplot(2,1,2)
hold on 
grid on
set(gca,'TickLabelInterpreter','latex')
set(gca,'DefaultAxesFontSize',fontsize);

plot((1:N)*param.t_int, pfB(2,1:N,1),'-', 'color', colors(1,:));
plot((1:N)*param.t_int, pfBexp(2,1:N,1), '--', 'color', colors(1,:));

plot((1:N)*param.t_int, pfB(2,1:N,2),'-', 'color', colors(2,:));
plot((1:N)*param.t_int, pfBexp(2,1:N,2), '--', 'color', colors(2,:));

xlabel('time (sec)')
ylabel('$y_f^\mathcal{B}$ (m)')

hleg = legend('finger 1 planned', 'finger 1 exp', 'finger 2 planned', 'finger 2 exp');
set(hleg,'interpreter', 'latex');

end