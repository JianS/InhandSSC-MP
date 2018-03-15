function [S1, G1, n2min, n2max] = FindS1G1(o, S, G, rs, rg)

    %% find S1 first
%     tem = S + (G-S)*rs;
% %     S1 = [mean(tem), mean(tem)]; % point at y1=y2 when t=T1
%     % find nearest point on MRC to tem
%     n1 = length(o.y1);
%     min_d = inf;
%     for i = 1:n1
%         tem_d = norm(o.MRC(:,i) - tem);
%         if tem_d < min_d;
%             min_d = tem_d;
%             min_i = i;
%         end
%     end
%     
%     tem1 = [inf; inf];
%     if min_i > 1
%         A = o.MRC(:,min_i-1); B = o.MRC(:,min_i);        
%         k = -((A-tem)'*(B-A))/((B-A)'*(B-A));
%         k = min([1,k]); k = max([0,k]);
%         tem1 = A + k*(B-A);
%     end
%     if sum(isnan(tem1)) > 0
%         tem1 = [inf; inf];
%     end
%     
%     tem2 = [inf;inf];
%     if min_i < n1
%         A = o.MRC(:,min_i); B = o.MRC(:,min_i+1);      
%         k = -((A-tem)'*(B-A))/((B-A)'*(B-A));
%         k = min([1,k]); k = max([0,k]);
%         tem2 = A + k*(B-A);
%     end
%     if sum(isnan(tem2)) > 0
%         tem2 = [inf; inf];
%     end
%     
%     if (norm(tem-tem1) < norm(tem-tem2))
%         S1 = tem1;
%         n2max = min_i-1;
%     else
%         S1 = tem2;
%         n2max = min_i;
%     end
    l_goal = rs*o.l_MRC;
    ladd = 0;
    i = 1;
    while ladd < l_goal
        ladd = ladd + norm(o.MRC(:,i+1) - o.MRC(:,i));
        i = i+1;
    end
    dl = ladd - l_goal;
    if(i<=1)
        disp('');
    end
    vec = o.MRC(:,i) - o.MRC(:,i-1);
    S1 = o.MRC(:,i) - dl*((vec)/norm(vec));
    n2max = i-1;
    
    %% find G1 now
    l_goal = rg*o.l_MRC;
    ladd = 0;
    i = 1;
    while ladd <= l_goal
        ladd = ladd + norm(o.MRC(:,i+1) - o.MRC(:,i));
        i = i+1;
    end
    dl = ladd - l_goal;
    vec = o.MRC(:,i) - o.MRC(:,i-1);
    G1 = o.MRC(:,i) - dl*((vec)/norm(vec));
    n2min = i;
    
    
%     tem = G + (S-G)*rg;
% %     G1 = [mean(tem), mean(tem)]; % point at y1=y2 when t=T1+T2
%     % find nearest point on MRC to tem
%     min_d = inf;
%     for i = 1:n1
%         tem_d = norm(o.MRC(:,i) - tem);
%         if tem_d < min_d;
%             min_d = tem_d;
%             min_i = i;
%         end
%     end
%     
%     tem1 = [inf; inf];
%     if min_i > 1
%         A = o.MRC(:,min_i-1); B = o.MRC(:,min_i);        
%         k = -((A-tem)'*(B-A))/((B-A)'*(B-A));
%         k = min([1,k]); k = max([0,k]);
%         tem1 = A + k*(B-A);
%     end
%     
%     tem2 = [inf;inf];
%     if min_i < n1
%         A = o.MRC(:,min_i); B = o.MRC(:,min_i+1);      
%         k = -((A-tem)'*(B-A))/((B-A)'*(B-A));
%         k = min([1,k]); k = max([0,k]);
%         tem2 = A + k*(B-A);
%     end
%     
%     if (norm(tem-tem1) < norm(tem-tem2))
%         G1 = tem1;
%         n2min = min_i;
%     else
%         G1 = tem2;
%         n2min = min_i+1;
%     end

end