% function animation(q_o, q_f,q_f_B, p_a, Ifslide, f_c, f_N, f_t, param)
function animation_MP(q_o, q_h, p_f, p_f_H, p_a, p_a_H, f_c, f_N, f_t, Ifslide, pfB_goal, param, obj_pts_plot, Ifrecord, videoname)

addpath('include/');

    figure;
    
    w = 7;
    h = 7;
    os = 2;
    set(gcf,'Units','inches','PaperSize',[w h], 'OuterPosition', [os os w h]);
    set(gcf,'defaulttextinterpreter','latex')
    
    fontsize = 9;
    
    axis equal
    
    bb = 0.02;
    Xmin = min([min(min(p_a(1,:,:))) min(q_o(1,:)-0.1)])-bb; 
    Xmax = max([max(max(p_a(1,:,:))) max(q_o(1,:)+0.1)])+bb;
    Ymin = min([min(min(p_a(2,:,:))), -0.05])-bb; Ymax = max([max(max(p_a(2,:,:))), 0.3])+bb;

    tx = max(q_o(1,:))+0.1;
    
    hold on
    
%     fig2 = figure(2);
%     set(gcf,'Units','inches','PaperSize',[w/2 h/2], 'OuterPosition', [os+w os+h/2 w/2 h/2]);
%     
%     fig3 = figure(3);
%     set(gcf,'Units','inches','PaperSize',[w/2 h/2],'OuterPosition', [os+w os w/2 h/2]);
    
    colors = colormap(lines(param.n_f));
    
    MUL = 5;
    
    if Ifrecord
        F(round(param.N/MUL)+1) = struct('cdata',[],'colormap',[]); 
        k = 1;
    end
    
    
    for i = [1:MUL:param.N param.N]
%         figure(fig1);
        clf
        
        hold on
        
        % draw the table
        ty = 0; % y pos of the table top 
        w = 0.2; h = 0.05;
        tp = [-w/2 ty; -w/2 ty-h; w/2 ty-h; w/2 ty; -w/2 ty];
        fill(tp(:,1), tp(:,2),[1 0.7 0], 'edgecolor', [0.7 0.5 0.3], 'linewidth',2);

        % draw the object
%         aa = 0:-pi/30:-pi*2;
% 
%         xc1 = param.o1(1) - sin(aa)*param.r;
%         yc1 = param.o1(2) + cos(aa)*param.r;
% 
%         xc2 = param.o2(1) - sin(aa)*param.r;
%         yc2 = param.o2(2) + cos(aa)*param.r;
% 
%         % line(xc1,yc1)
%         % line(xc2,yc2)
%         [xo, yo] = polybool('&', xc1,yc1, xc2,yc2);
%         [tpx, tpy] = poly2cw(tp(:,1), tp(:,2));
%         [xo, yo] = polybool('-', xo,yo, tpx, tpy);
%         line(xo,yo, 'color','k','linewidth',1.5)
        line(q_o(1,i)+obj_pts_plot(1,:), q_o(2,i)+obj_pts_plot(2,:), 'color','k','linewidth',1.5);
        
        
        Rh = CalR2d(q_h(3,i));
        thetao = q_o(3,i);
        Ro = CalR2d(thetao);
        
        for j = 1:param.n_f
            % draw goal locations
            pf_goal = q_o(1:2,i) + Ro*pfB_goal(:,j);
            circle(pf_goal(1),pf_goal(2), 0.005, 'k', '-', 'k')
            
            
            % draw spring (simulation)
            smid = q_h(1:2,i) + Rh*[p_a_H(1,i,j); p_f_H(2,i,j)];
            line([p_a(1,i,j) smid(1) p_f(1,i,j)],[p_a(2,i,j) smid(2) p_f(2,i,j)],...
                'color',colors(j,:), 'linewidth',2.5)
            
            % draw anchor point
            scatter(p_a(1,i,j),p_a(2,i,j),70,colors(j,:),...
                    's','filled','MarkerEdgeColor','k')
            
            ss = 500; %norm(f_c(:,i,j))*20;
            
             % draw initial contact point
%             p_f_init = q_o(1:2,i) + Ro*p_f_B(:,1,j);
%             scatter(p_f_init(1),p_f_init(2),70, colors(j,:), 'x','linewidth',2.5);

            % draw contact point, red when sliding
            cr = 0.004; % radius of the contact point
%             circle(p_f_exp(1,i,j),p_f_exp(2,i,j), cr , colors(j,:),'--','w')
            if Ifslide(i,j)==1
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
            fill(fcp(1,:),fcp(2,:),[0 .8 0],'facealpha',0.2, 'edgecolor','none')
%             plot([ff1(1) q_f(1,i,j)],[ff1(2) q_f(2,i,j)],'--k','linewidth',.5)
%             plot([ff2(1) q_f(1,i,j)],[ff2(2) q_f(2,i,j)],'--k','linewidth',.5)
            plot([ff1(1) p_f(1,i,j)],[ff1(2) p_f(2,i,j)],'color', [0 .8 0],'linewidth',.5)
            plot([ff2(1) p_f(1,i,j)],[ff2(2) p_f(2,i,j)],'color', [0 .8 0],'linewidth',.5)
            
            % draw contact force
            fNN = f_N(:,i,j)/ss;
            quiver(p_f(1,i,j),p_f(2,i,j), fNN(1), fNN(2),1, 'color','r','linewidth',1.5)
            ftN = f_t(:,i,j)/ss;
            quiver(p_f(1,i,j),p_f(2,i,j), ftN(1), ftN(2),1, 'color',[0 0.8 0.2],'linewidth',2)
            fcN = f_c(:,i,j)/ss;
            quiver(p_f(1,i,j),p_f(2,i,j), fcN(1), fcN(2),1, 'color','b','linewidth',1.5)
%             
            
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
        drawnow
        
        if i==1 
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