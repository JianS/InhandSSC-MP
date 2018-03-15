function PlotAllMotions(o, col, row, fontsize, ss, figureNum)

    if ~isempty(figureNum)
        figure(figureNum)
    else 
        figure
    end
    
%     w = 10.2;
%     h = 4.5;
    w = col*1.7;
    h = row*2.25;
    set(gcf,'Units','inches','PaperSize',[w h],'PaperPositionMode','auto', 'Position', [3 4 w h])

            
    set(gcf,'defaulttextinterpreter','latex')

    % fontsize = 11;

    set(gcf,'defaulttextinterpreter','latex')
    set(gcf,'DefaultAxesFontSize',fontsize);
    % set(gca, 'TickLabelInterpreter','latex')

    

% col = 6;
% row = 2;

N = o.n; % total steps of the traj
N1 = o.param.N1;
N2 = o.param.N2;
t_int = o.param.t_int;

step = round(N2/(col*row-3))+1;
Ncap_vec = [0:N1/2:(N1-1), N1:step:N N]; % a vector define the time of capturing

n_fig = length(Ncap_vec);


% n_blur = 4;
% step_blur = 30;

q_o = o.q_o;
q_h = o.q_h;
p_f = o.p_f;
p_f_H = o.p_f_H;
p_a = o.p_a;
p_a_H = o.p_a_H;
f_c = o.f_c;
f_N = o.f_N;
f_t = o.f_t;
Ifslide = o.Ifslide;
param = o.param;
obj_pts_plot = o.obj_pts_plot;

colors = colormap(lines(param.n_f));

cr = 0.006; % radius of the contact point
    
s_w = 0.008; % spring width
s_ratio = 0.4; % ratio of spring curved section
    
 w = 0.14; h = 0.02; % table size
    tx = q_o(1,1); % x pos of the table center 
    ty = q_o(2,1); % y pos of the table top 
    tp = [-w/2+tx ty; -w/2+tx ty-h; w/2+tx ty-h; w/2+tx ty; -w/2+tx ty];
    
    bb = 0.015;
    Xmin = min([min(min(p_a(1,:,:))) min(q_o(1,:)-0.03) min(tp(:,1))])-bb; 
    Xmax = max([max(max(p_a(1,:,:))) max(q_o(1,:)+0.03) max(tp(:,1))])+bb;
    Ymin = min([min(min(p_a(2,:,:))) min(tp(:,2))])-bb; 
    Ymax = max([max(max(p_a(2,:,:))) max(tp(:,2)) q_o(2,1)+max(obj_pts_plot(:,2))])+bb;
    
for jj = 1:n_fig
    subplot(row, col, jj);
    
    hold on; axis equal; box on;
    set(gca, 'TickLabelInterpreter','latex');
    
    title([num2str(round(Ncap_vec(jj)*t_int, 1)), '\,s'], 'FontSize', fontsize, 'FontWeight', 'bold');
    
    i = Ncap_vec(jj); 
    if i == 0
        i=1;
    end
    
    % draw the table
%     w = 0.2; h = 0.03;
%     tp = [-w/2+tx ty; -w/2+tx ty-h; w/2+tx ty-h; w/2+tx ty; -w/2+tx ty];
    fill(tp(:,1), tp(:,2),[1 0.7 0], 'edgecolor', [0.7 0.5 0.3], 'linewidth',2);

    % draw the object
    line(q_o(1,i)+obj_pts_plot(1,:), q_o(2,i)+obj_pts_plot(2,:), 'color','k','linewidth',1);
        
      
    % draw fingers
    
    Rh = CalR2d(q_h(3,i));
    thetao = q_o(3,i);
    Ro = CalR2d(thetao);

    for j = 1:param.n_f
        % draw goal locations
        pf_goal = q_o(1:2,i) + Ro*o.pfB_goal(:,j);
        circle(pf_goal(1),pf_goal(2), cr*0.8, 'k', '-', 'k')

        % draw spring (simulation)
        smid = q_h(1:2,i) + Rh*[p_a_H(1,j); p_f_H(2,i,j)];
%         line([p_a(1,i,j) smid(1) p_f(1,i,j)],[p_a(2,i,j) smid(2) p_f(2,i,j)],...
%             'color',colors(j,:), 'linewidth',2)
        DrawSpring2D(p_a(:,i,j),smid,s_w, s_ratio, 3, colors(j,:),1);
        DrawSpring2D(smid, p_f(:,i,j), s_w, s_ratio, 3, colors(j,:),1);
         
        % draw anchor traj history
        plot(p_a(1,1:i,j),p_a(2,1:i,j),'color',colors(j,:), 'linewidth',0.5)
            
        % draw anchor point
        scatter(p_a(1,i,j),p_a(2,i,j),20,colors(j,:),...
                's','filled','MarkerEdgeColor','k')

%         ss = 500; %norm(f_c(:,i,j))*20;

         % draw initial contact point
%             p_f_init = q_o(1:2,i) + Ro*p_f_B(:,1,j);
%             scatter(p_f_init(1),p_f_init(2),70, colors(j,:), 'x','linewidth',2.5);

        % draw contact point, red when sliding

%             circle(p_f_exp(1,i,j),p_f_exp(2,i,j), cr , colors(j,:),'--','w')

        if ~isempty(Ifslide)
            if Ifslide(i,j)==1
%                 if i > o.param.N1
%                 scatter(q_f(1,i,j),q_f(2,i,j),500, '.r')
                circle(p_f(1,i,j),p_f(2,i,j), cr , colors(j,:),'-','r')
            else
                circle(p_f(1,i,j),p_f(2,i,j), cr , colors(j,:),'-','b')
            end

            % draw friction cone
            ff_l = 0.03;
            tt = [f_N(2,i,j); -f_N(1,i,j)];
            ff1 = p_f(1:2,i,j) + ff_l*f_N(:,i,j)/norm(f_N(:,i,j)) + param.mu*ff_l*tt/norm(tt);
            ff2 = p_f(1:2,i,j) + ff_l*f_N(:,i,j)/norm(f_N(:,i,j)) - param.mu*ff_l*tt/norm(tt);
            fcp(:,1) = ff1; fcp(:,2) = p_f(1:2,i,j); fcp(:,3) = ff2;
            fill(fcp(1,:),fcp(2,:),[0 .8 0],'facealpha',0.1, 'edgecolor','none')
%             plot([ff1(1) q_f(1,i,j)],[ff1(2) q_f(2,i,j)],'--k','linewidth',.5)
%             plot([ff2(1) q_f(1,i,j)],[ff2(2) q_f(2,i,j)],'--k','linewidth',.5)
            plot([ff1(1) p_f(1,i,j)],[ff1(2) p_f(2,i,j)],'color', [0 .8 0],'linewidth',.5)
            plot([ff2(1) p_f(1,i,j)],[ff2(2) p_f(2,i,j)],'color', [0 .8 0],'linewidth',.5)

            % draw contact force
            fNN = f_N(:,i,j)/ss;
            quiver(p_f(1,i,j),p_f(2,i,j), fNN(1), fNN(2),1, 'color','r','linewidth',1 ,'showarrowhead','off')
%             line([p_f(1,i,j) p_f(1,i,j)+fNN(1)], [p_f(2,i,j) p_f(1,i,j)+fNN(2)], 'color','r','linewidth',1)
            ftN = f_t(:,i,j)/ss;
            quiver(p_f(1,i,j),p_f(2,i,j), ftN(1), ftN(2),1, 'color',[0 0.8 0.2],'linewidth',1 ,'showarrowhead','off')
%             line([p_f(1,i,j) p_f(1,i,j)+ftN(1)], [p_f(2,i,j) p_f(1,i,j)+ftN(2)], 'color',[0 0.8 0.2],'linewidth',1.5)
            
            fcN = f_c(:,i,j)/ss;
            quiver(p_f(1,i,j),p_f(2,i,j), fcN(1), fcN(2),1, 'color','b','linewidth',1 ,'showarrowhead','off')
%             line([p_f(1,i,j) p_f(1,i,j)+fcN(1)], [p_f(2,i,j) p_f(1,i,j)+fcN(2)], 'color','b','linewidth',1)
              
        else
            circle(p_f(1,i,j),p_f(2,i,j), cr , colors(j,:),'-','b')
        end

    end
    
    axis([Xmin Xmax Ymin Ymax])
    set(gca,'XTickLabel','')    
    set(gca,'YTickLabel','')
end




end