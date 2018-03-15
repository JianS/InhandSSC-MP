% function animation(q_o, q_f,q_f_B, p_a, Ifslide, f_c, f_N, f_t, param)
function animationMPI(o, MUL, fontsize, ss, mode, Ifrecord, videoname)
% ss -- scalar for scale the plotted forces

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

% addpath('include/');

    figure(10);
    
    w = 12;
    h = 8;
    os = 2;
    set(gcf,'Units','inches','PaperSize',[w h], 'OuterPosition', [os os w h]);
    set(gcf,'defaulttextinterpreter','latex')
    
    lwb = 1; % line width base
    
    cr = 0.006; % radius of the contact point
    fcAlpha = 0.2;
    
    s_w = 0.008; % spring width
    s_ratio = 0.4; % ratio of spring curved section
    
    if Ifrecord
        F(round(param.N/MUL)+1) = struct('cdata',[],'colormap',[]); 
        k = 1;
        MUL = 20;
        
        w = 1920;
        h = 1080;
        os = 100;
        set(gcf,'Units','pixels','PaperSize',[w h], 'OuterPosition', [os os w h]);
        lwb = 2;
        cr = 0.004; % radius of the contact point
        fcAlpha = 0.4;
    end
    
%     fontsize = 9;
    
    axis equal
    
    
    
    w = 0.15; h = 0.03; % table size
    tx = q_o(1,1); % x pos of the table center 
    ty = q_o(2,1); % y pos of the table top 
    tp = [-w/2+tx ty; -w/2+tx ty-h; w/2+tx ty-h; w/2+tx ty; -w/2+tx ty];
    
    bb = 0.02;
    Xmin = min([min(min(p_a(1,:,:))) min(q_o(1,:)-0.) min(tp(:,1))])-bb; 
    Xmax = max([max(max(p_a(1,:,:))) max(q_o(1,:)+0.) max(tp(:,1))])+bb;
    Ymin = min([min(min(p_a(2,:,:))) min(tp(:,2))])-bb; 
    Ymax = max([max(max(p_a(2,:,:))) max(tp(:,2)) q_o(2,1)+max(obj_pts_plot(:,2))])+bb;

%     tx = max(q_o(1,:))+0.1;
    
    hold on
    
%     fig2 = figure(2);
%     set(gcf,'Units','inches','PaperSize',[w/2 h/2], 'OuterPosition', [os+w os+h/2 w/2 h/2]);
%     
%     fig3 = figure(3);
%     set(gcf,'Units','inches','PaperSize',[w/2 h/2],'OuterPosition', [os+w os w/2 h/2]);
    
    colors = colormap(lines(param.n_f));
    
%     MUL = 2;
    
    
    
    if strcmp(mode, 'sticking')
        n = param.N1;
        IfplotY = 0;
    elseif strcmp(mode, 'twoplots')
        n = param.N;
        IfplotY = 1;
    else
        n = param.N;
        IfplotY = 0;
    end
    
    % define size of subplots
    wL = 11;
    wR = 12;
    space = 4; wT = wL+wR+space;
   
    
    for i = [1:MUL:n n]
%         figure(fig1);
        clf
        
        if (IfplotY)
            subplot(1,wT,1:wL); 
        end
        
        hold on
        set(gca, 'TickLabelInterpreter','latex','FontSize',fontsize);
    
        % draw the table
        fill(tp(:,1), tp(:,2),[1 0.7 0], 'edgecolor', [0.7 0.5 0.3],...
            'linewidth',2*lwb);

        % draw the object
        line(q_o(1,i)+obj_pts_plot(1,:), q_o(2,i)+obj_pts_plot(2,:),...
            'color','k','linewidth',1.5*lwb);
        
        
        Rh = CalR2d(q_h(3,i));
        thetao = q_o(3,i);
        Ro = CalR2d(thetao);
        
        for j = 1:param.n_f
            % draw goal locations
            pf_goal = q_o(1:2,i) + Ro*o.pfB_goal(:,j);
            circle(pf_goal(1),pf_goal(2), cr*0.8, 'k', '-', [0,1,127/255],lwb^2)
            
            % draw spring (simulation)
            smid = q_h(1:2,i) + Rh*[p_a_H(1,j); p_f_H(2,i,j)];
%             line([p_a(1,i,j) smid(1) p_f(1,i,j)],[p_a(2,i,j) smid(2) p_f(2,i,j)],...
%                 'color',colors(j,:), 'linewidth',2.5*lwb)
            DrawSpring2D(p_a(:,i,j),smid,s_w, s_ratio, 3, colors(j,:),1.5*lwb);
            DrawSpring2D(smid, p_f(:,i,j), s_w, s_ratio, 3, colors(j,:),1.5*lwb);
            
            % draw anchor traj history
            plot(p_a(1,1:i,j),p_a(2,1:i,j),'color',colors(j,:), 'linewidth',lwb)
            
            % draw anchor point
            scatter(p_a(1,i,j),p_a(2,i,j),70*lwb^2.5,colors(j,:),...
                    's','filled','MarkerEdgeColor','k', 'linewidth', lwb)
            
%             ss = 500; %norm(f_c(:,i,j))*20;
            
             % draw initial contact point
%             p_f_init = q_o(1:2,i) + Ro*p_f_B(:,1,j);
%             scatter(p_f_init(1),p_f_init(2),70, colors(j,:), 'x','linewidth',2.5);

            % draw contact point, red when sliding
            
%             circle(p_f_exp(1,i,j),p_f_exp(2,i,j), cr , colors(j,:),'--','w')

            if ~isempty(Ifslide)
                if Ifslide(i,j)==1
%                 if i > o.param.N1
    %                 scatter(q_f(1,i,j),q_f(2,i,j),500, '.r')
                    circle(p_f(1,i,j),p_f(2,i,j), cr , colors(j,:),'-','r',lwb)
                else
                    circle(p_f(1,i,j),p_f(2,i,j), cr , colors(j,:),'-','b',lwb)
                end

                % draw friction cone
                ff_l = 0.03;
                tt = [f_N(2,i,j); -f_N(1,i,j)];
                ff1 = p_f(1:2,i,j) + ff_l*f_N(:,i,j)/norm(f_N(:,i,j)) + param.mu*ff_l*tt/norm(tt);
                ff2 = p_f(1:2,i,j) + ff_l*f_N(:,i,j)/norm(f_N(:,i,j)) - param.mu*ff_l*tt/norm(tt);
                fcp(:,1) = ff1; fcp(:,2) = p_f(1:2,i,j); fcp(:,3) = ff2;
                fill(fcp(1,:),fcp(2,:),[0 .8 0],'facealpha', fcAlpha, 'edgecolor','none')
    %             plot([ff1(1) q_f(1,i,j)],[ff1(2) q_f(2,i,j)],'--k','linewidth',.5)
    %             plot([ff2(1) q_f(1,i,j)],[ff2(2) q_f(2,i,j)],'--k','linewidth',.5)
                plot([ff1(1) p_f(1,i,j)],[ff1(2) p_f(2,i,j)],'color', [0 .8 0],...
                    'linewidth',.5*lwb)
                plot([ff2(1) p_f(1,i,j)],[ff2(2) p_f(2,i,j)],'color', [0 .8 0],...
                    'linewidth',.5*lwb)

                % draw contact force
                fNN = f_N(:,i,j)/ss;
                quiver(p_f(1,i,j),p_f(2,i,j), fNN(1), fNN(2),1, 'color','r',...
                    'linewidth',1.5*lwb)
                ftN = f_t(:,i,j)/ss;
                quiver(p_f(1,i,j),p_f(2,i,j), ftN(1), ftN(2),1, 'color',[0 0.8 0.2],...
                    'linewidth',2*lwb)
                fcN = f_c(:,i,j)/ss;
                quiver(p_f(1,i,j),p_f(2,i,j), fcN(1), fcN(2),1, 'color','b',...
                    'linewidth',1.5*lwb)
%             
            else
                circle(p_f(1,i,j),p_f(2,i,j), cr , colors(j,:),'-','b',lwb)
            end
            
        end
        
        title(['\bf time: ' num2str(i*param.t_int,'%.2f') ' s'], 'fontsize',fontsize)
        
%         axis([Xmin Xmax Ymin Ymax])
        
        axis equal
        box on
        
%         hold off
    	axis([Xmin Xmax Ymin Ymax])
        
        
%         fig2 = subplot(1,3,2,'replace');
%         fig3 = subplot(1,3,3,'replace');
%         plot_sticking_region(q_f_B(:,i,:), p_a(:,i,:), q_o(:,i), param, fig2,0)
%         plot_sticking_region(q_f_B(:,i,:), p_a(:,i,:), q_o(:,i), param, fig3,1)
%       

        if (IfplotY)
            subplot(1,wT,wL+space:wT); 
            hold on
            gg = gcf;
            o.PlotPath(fontsize,gg.Number, lwb)
%             circle(o.p_f_B(2,i,1),o.p_f_B(2,i,2),  0.0025, 'k','-','r')
            scatter(o.p_f_B(2,i,1),o.p_f_B(2,i,2), 120*lwb^1.5, 'o', 'MarkerEdgeColor','k',...
              'MarkerFaceColor','r', 'LineWidth',1*lwb)
%             axis([0 o.planResult.S(1) 0 o.planResult.S(2)])
            set(gca, 'TickLabelInterpreter','latex','FontSize',fontsize);
    
        end
        drawnow
        
        
        if i==1 
            title('Press anykey to start playback.','color','r','FontSize',fontsize*1.5);
            pause();
            
        
        end
        
        if Ifrecord
            F(k) = getframe(gcf); k=k+1;
        end
            
    end
    
    %% write .avi file
if Ifrecord
    vid = VideoWriter([videoname '.avi']);
    vid.Quality = 100;
    vid.FrameRate = 25;
    open(vid);
%     for k = 1:size(F,2)
        writeVideo(vid, F);
%     end
    close(vid);
end

end