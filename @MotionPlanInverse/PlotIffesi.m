function PlotIffesi(o, fontsize)
%     figure
    
%     fontsize = 12;
    set(gcf,'defaulttextinterpreter','latex');
    set(gcf,'DefaultAxesFontSize',fontsize);

    set(gca, 'TickLabelInterpreter','latex');
    
    xlabel('$y^\mathcal{B}_{f1}$ (m)');
    ylabel('$y^\mathcal{B}_{f2}$ (m)');
    
%     title(['$\mu_e = ' num2str(o.param.mu_e) ', G = ' num2str(o.param.G(2)) '$\,N.']);

    hold on
    axis equal
    axis tight
    
    fcolor = [0.85 1 0.85];% color for feasible region

%     [n1,n2] = size(o.Iffeasi);
%     for i1 = 1:n1
%        for i2 = 1:n2
%            if o.Iffeasi(i1,i2)
% %                scatter(o.y1(i1), o.y2(i2), 30, 'r','filled')
% %                circle(o.y1(i1), o.y2(i2), r, 'r', '-', 'r')
%                 o.DrawSquare(o.y1(i1), o.y2(i2), r, fcolor, '-', fcolor)
%            else
% %                scatter(o.y1(i1), o.y2(i2), 30, [.7 .7 .7],'filled')
%                o.DrawSquare(o.y1(i1), o.y2(i2), r, [.7 .7 .7], '-', [.7 .7 .7])
%            end
%        end
%     end

%     plot(o.feasiBoundary(1,:),o.feasiBoundary(2,:),'b', 'linewidth',2 )
    bx = [0 0 o.p_f_B(2,1,1) o.p_f_B(2,1,1) 0];
    by = [0 o.p_f_B(2,1,2) o.p_f_B(2,1,2) 0 0];
    fill(bx, by, [.7 .7 .7]);
    
    % plot boundary of the feasible region
%     fill(o.feasiBoundary(1,:),o.feasiBoundary(2,:), fcolor,...
%         'edgecolor','b', 'linewidth',2 )
    fill(o.feasiBoundary(1,:),o.feasiBoundary(2,:), fcolor,'edgecolor','none')
        
          

end