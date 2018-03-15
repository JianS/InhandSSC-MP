function PlotInit(o, pfH_init, fontsize, ss, figureNum)
% plot initial state of the system
% ss -- scalar for scale the plotted forces

    if ~isempty(figureNum)
        figure(figureNum)
    else 
        figure
    end
    
    w = 10;
    h = 8;
    set(gcf,'Units','inches','PaperSize',[w h],'PaperPositionMode','auto', 'Position', [3 4 w h])

            
    set(gcf,'defaulttextinterpreter','latex')

    % fontsize = 11;

    set(gcf,'defaulttextinterpreter','latex')
    set(gcf,'DefaultAxesFontSize',fontsize);
    % set(gca, 'TickLabelInterpreter','latex')



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
    

tx = q_o(1,1); % x pos of the table center 
ty = q_o(2,1); % y pos of the table top 
w = 0.2; h = 0.03;
tp = [-w/2+tx ty; -w/2+tx ty-h; w/2+tx ty-h; w/2+tx ty; -w/2+tx ty];
        

 bb = 0.02;
    Xmin = min([min(min(p_a(1,1,:))) min(q_o(1,1)-0.1) min(tp(:,1))])-bb; 
    Xmax = max([max(max(p_a(1,1,:))) max(q_o(1,1)+0.1) max(tp(:,1))])+bb;
    Ymin = min([min(min(p_a(2,1,:))) min(tp(:,2))])-bb; 
    Ymax = max([max(max(p_a(2,1,:))) max(tp(:,2)) q_o(2,1)+max(obj_pts_plot(:,2))])+bb;

    
    hold on; axis equal; box on;
    set(gca, 'TickLabelInterpreter','latex');
    
    title('Initial State', 'FontSize', fontsize, 'FontWeight', 'bold');
    
    
    i=1;

    
    % draw the table
    fill(tp(:,1), tp(:,2),[1 0.7 0], 'edgecolor', [0.7 0.5 0.3], 'linewidth',2);

    % draw the object
    line(q_o(1,i)+obj_pts_plot(1,:), q_o(2,i)+obj_pts_plot(2,:), 'color','k','linewidth',1);
        
      
    % draw fingers
    
    Rh = CalR2d(q_h(3,i));
    thetao = q_o(3,i);
    Ro = CalR2d(thetao);
    
    % draw hand frame
    l_frame = 0.02;

    xh = q_h(1:2,i) + Rh*[l_frame; 0]; 
    yh = q_h(1:2,i) + Rh*[0; l_frame];
    line([q_h(1,i) xh(1)], [q_h(2,i) xh(2)], 'color', 'r', 'linewidth',1.5);
    line([q_h(1,i) yh(1)], [q_h(2,i) yh(2)], 'color', 'g', 'linewidth',1.5);

    
    pf_init = pfH_init;
    
    for j = 1:param.n_f
        % draw goal locations
        pf_goal = q_o(1:2,i) + Ro*o.pfB_goal(:,j);
        circle(pf_goal(1),pf_goal(2), 0.005, 'k', '-', 'k')

        % draw spring (simulation)
        smid = q_h(1:2,i) + Rh*[p_a_H(1,j); p_f_H(2,i,j)];
        line([p_a(1,i,j) smid(1) p_f(1,i,j)],[p_a(2,i,j) smid(2) p_f(2,i,j)],...
            'color',colors(j,:), 'linewidth',2)

        % draw anchor point
        scatter(p_a(1,i,j),p_a(2,i,j),50,colors(j,:),...
                's','filled','MarkerEdgeColor','k')

%         ss = 500; %norm(f_c(:,i,j))*20;

         % draw initial contact point
%             p_f_init = q_o(1:2,i) + Ro*p_f_B(:,1,j);
%             scatter(p_f_init(1),p_f_init(2),70, colors(j,:), 'x','linewidth',2.5);

        % draw contact point, red when sliding
        cr = 0.008; % radius of the contact point
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

        if ~isempty(pfH_init)
            % draw finger endpos from feedback
            pf_init(:,j) = q_h(1:2,i) + Rh*pfH_init(:,j);
            circle(pf_init(1,j),pf_init(2,j), cr*0.7 , colors(j,:),'--','w')
        end
    end
    
    axis([Xmin Xmax Ymin Ymax])
        
    


end