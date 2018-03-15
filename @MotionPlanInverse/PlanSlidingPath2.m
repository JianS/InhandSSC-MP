function o = PlanSlidingPath2(o)
% this planning algorithm is first goes to the line y1 = y2, then goes to
% the goal. Since the y1=y2 line is mostly stable because no horizontal
% table contact forces occur.

    if ~inpolygon(o.pfB_goal(2,1), o.pfB_goal(2,2),...
            o.feasiBoundary(1,:), o.feasiBoundary(2,:))
        disp('Goal positions cannot be achieved!');
        return
    end
    
    % start
    S = [o.p_f_B(2,1,1); o.p_f_B(2,1,2)];
    % goal
    G = o.pfB_goal(2,:)';

    o.yfBpath = S;

    if S(1) ~= S(2)
        t = zeros(2,3);
        t(:,1) = [S(1); S(1)];
        t(:,2) = [S(2); S(2)];
        t(:,3) = [(S(1)+S(2))/2; (S(1)+S(2))/2];

        next = [inf; inf];
        [~, ifS_Onedge] = inpolygon(S(1),S(2),o.feasiBoundary(1,:), o.feasiBoundary(2,:));
        for i = 1:3
            [feasi1, ison] = inpolygon(t(1,i),t(2,i),o.feasiBoundary(1,:), o.feasiBoundary(2,:));
            x = [S(1) t(1,i)]; y = [S(2) t(2,i)];
            [xx,~] = polyxpoly(x,y, o.feasiBoundary(1,:), o.feasiBoundary(2,:));
            feasi2 = length(xx)==ifS_Onedge + ison;
            if norm(t(:,i)-G) < norm(next-G) && feasi1 && feasi2
                next = t(:,i);
            end
        end
        o.yfBpath = [o.yfBpath next];
    else
        next = S;
    end
%     if norm(t1-G) < norm(t2-G)
%         o.yfBpath = [o.yfBpath t1];
%     else 
%         o.yfBpath = [o.yfBpath t2];
%     end
    
 
    if G(1) ~= G(2)
        t(:,1) = [G(1); G(1)];
        t(:,2) = [G(2); G(2)];
        t(:,3) = [(G(1)+G(2))/2; (G(1)+G(2))/2];

        next2 = [inf; inf];
        [~, ifG_Onedge] = inpolygon(G(1),G(2),o.feasiBoundary(1,:), o.feasiBoundary(2,:));
        for i = 1:3
            [feasi1, ison] = inpolygon(t(1,i),t(2,i),o.feasiBoundary(1,:), o.feasiBoundary(2,:));
            x = [G(1) t(1,i)]; y = [G(2) t(2,i)];
            [xx,~] = polyxpoly(x,y, o.feasiBoundary(1,:), o.feasiBoundary(2,:));
            feasi2 = (length(xx)==ifG_Onedge + ison);
            if norm(t(:,i)-next) < norm(next2-next) && feasi1 && feasi2
                next2 = t(:,i);
            end
        end
        o.yfBpath = [o.yfBpath next2];
    end
%     if norm(t1-G) < norm(t2-G)
%         o.yfBpath = [o.yfBpath t1];
%     else 
%         o.yfBpath = [o.yfBpath t2];
%     end
    
  
    o.yfBpath = [o.yfBpath G];
end