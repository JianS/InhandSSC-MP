function o = PlanSlidingPath(o)
    if ~inpolygon(o.pfB_goal(2,1), o.pfB_goal(2,2),...
            o.feasiBoundary(1,:), o.feasiBoundary(2,:))
        disp('Goal positions cannot be achieved!');
        return
    end

    o.yfBpath = [o.p_f_B(2,1,1); o.p_f_B(2,1,2)];


    G = o.pfB_goal(2,:);


    sucflag = 0;
    for kk = 1:20 % 100 is the max iteration time
        A = o.yfBpath(:,end);
        B = G;
        x = [A(1) B(1)]; y = [A(2) B(2)];
        [xx,~] = polyxpoly(x,y, o.feasiBoundary(1,:), o.feasiBoundary(2,:));

        if length(xx) == 1 % when start can direct connect to goal
            o.yfBpath = [o.yfBpath B'];
            sucflag = 1;
            disp('Feasible path found!')
            break
        else % if not, do search

            nb = length(o.feasiBoundary);
            mindis = inf;
            for i = 1:nb
                B = o.feasiBoundary(:,i);
                x = [A(1) B(1)]; y = [A(2) B(2)];
                [xx,yy] = polyxpoly(x,y, o.feasiBoundary(1,:), o.feasiBoundary(2,:));
                
                mid1 = A + (B-A)/3;
                ret1 = inpolygon(mid1(1),mid1(2),o.feasiBoundary(1,:), o.feasiBoundary(2,:));
                mid2 = A + (B-A)/3*2;
                ret2 = inpolygon(mid2(1),mid2(2),o.feasiBoundary(1,:), o.feasiBoundary(2,:));
                if length(xx) == 2 && ret1 && ret2% can direct connect
                    dis = norm(A-B) + norm(B-G');
                    if dis < mindis
                        mindis = dis;
                        buff = B;
                    end
                end

%                 ifcan = 1; % if can direct connect
%                 nx = length(xx);
%                 if nx > 1
%                     for ii = 1:nx-1
%                         mid1x = xx(ii) + (xx(ii+1)-xx(ii))/3;
%                         mid1y = yy(ii) + (yy(ii+1)-yy(ii))/3;
%                         ret1 = inpolygon(mid1x,mid1y,o.feasiBoundary(1,:), o.feasiBoundary(2,:));
%                         mid2x = xx(ii) + (xx(ii+1)-xx(ii))/3*2;
%                         mid2y = yy(ii) + (yy(ii+1)-yy(ii))/3*2;
%                         ret2 = inpolygon(mid2x,mid2y,o.feasiBoundary(1,:), o.feasiBoundary(2,:));
%                         if ~(ret1 && ret2)
%                             ifcan = 0;
%                         end
%                     end
%                 end
%                 
%                 if ifcan
%                     dis = norm(A-B) + norm(B-G');
%                     if dis < mindis
%                         mindis = dis;
%                         buff = B;
%                     end
%                 end
            end
            o.yfBpath = [o.yfBpath buff];

        end
    end
    
%     nn = length(o.yfBpath);
%     ytemp = o.yfBpath(:,1);
%     for i = 2:nn
%         A = o.yfBpath(:,1);
%         B = o.yfBpath(:,i);
% %         C = o.yfBpath(:,i+2);
%         x = [A(1) B(1)]; y = [A(2) B(2)];
%         [xx,~] = polyxpoly(x,y, o.feasiBoundary(1,:), o.feasiBoundary(2,:));
%         mid1 = A + (B-A)/3;
%         ret1 = inpolygon(mid1(1),mid1(2),o.feasiBoundary(1,:), o.feasiBoundary(2,:));
%         mid2 = A + (B-A)/3*2;
%         ret2 = inpolygon(mid2(1),mid2(2),o.feasiBoundary(1,:), o.feasiBoundary(2,:));
%         if ~(length(xx) == 2 && ret1 && ret2)% cannot direct connect
%             ytemp = [ytemp o.yfBpath(:,i-1:nn)];
%             break
%         end
%     end
    
%     o.yfBpath = ytemp;

    if ~sucflag
        disp('Cannot find feasible path.');
    end
end