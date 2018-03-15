function [c, ceq] = constraints(x)

    c = 0;
    ceq = [0 0];
global opti_input

    S = opti_input.S;
    G = opti_input.G;
    o = opti_input.o;
    
    n_f = size(S,1);
    
    dir = sign(G-S)';
    
    T1 = x(1)*opti_input.Tslide;
    T2 = x(2)*opti_input.Tslide;
    rs = x(3);
    rg = x(4);
    
    T3 = opti_input.Tslide - T1 - T2;
    
%     tem = S + (G-S)*rs;
%     S1 = [mean(tem), mean(tem)]; % point at y1=y2 when t=T1 
%     tem = G + (S-G)*rg;
%     G1 = [mean(tem), mean(tem)]; % point at y1=y2 when t=T1+T2
    [S1, G1, n2min, n2max] = o.FindS1G1(S, G, rs, rg);
%     S1 = S1'; G1 = G1';

    if S1 == o.MRC(:,n2max) % when S1 is at a node of MRC
        n2max = n2max - 1; 
    end
    if G1 == o.MRC(:,n2min) % when G1 is at a node of MRC
        n2min = n2min + 1; 
    end
    
    % calculate length of T2 traj -- l2
%     nn = 2+(n2max-n2min);
    ls1 = norm(S1-o.MRC(:,n2max));
    lg1 = norm(G1-o.MRC(:,n2min));
    l2 = ls1 + lg1;
    if n2max > n2min
        for i = n2min:n2max-1
            l2 = l2 + norm(o.MRC(:,i+1)-o.MRC(:,i));
        end
    elseif n2max < n2min
        l2 = norm(S1 - G1);        
        disp('n2max is smaller than n2min');
    end
    
    vnorm = l2/T2; % the speed magnitude during T2
    v1 = vnorm*((o.MRC(:,n2max)-S1)/ls1);
    v3 = vnorm*((G1-o.MRC(:,n2min))/lg1);
    
    
    % calculate
%     v = (G1-S1)/T2;
    M1 = o.CalCubicPolyMatrix(T1);
    M3 = o.CalCubicPolyMatrix(T3);
%     a11 = M1\[S(1); 0; S1(1); v(1)]; % finger 1 trej for phase 1
    
%     a13 = M3\[G1(1); v(1); G(1); 0]; % finger 1 trej for phase 3
%     
%     a21 = M1\[S(2); 0; S1(2); v(2)]; % finger 2 trej for phase 1
%     a23 = M3\[G1(2); v(2); G(2); 0]; % finger 2 trej for phase 3
    
%     a1 = zeros(4, n_f); a3 = zeros(4,n_f);
    a1 = M1\[S'; zeros(1,n_f); S1'; v1'];  % phase 1 trej
    a3 = M3\[G1'; v3'; G'; zeros(1,n_f)];  % phase 3 trej
    
    if sum(isnan(a1)) > 0
        disp('!!');
    end
    
    if sum(isnan(a1)) == inf
        disp('!!');
    end
    
    if sum(isnan(a3)) > 0
        disp('!!');
    end
    
    if sum(isnan(a3)) == inf
        disp('!!');
    end
    
    for i = 1:n_f
        % phase 1 the velocity direction should always towards goal
        rts1 = roots([3*a1(4,i), 2*a1(3,i), a1(2,i)]);
%         pts1 = sort([rts1; T1]);
        pts1 = 0;
        for j = 1:size(rts1,1)
            if rts1(j) > 0 && rts1(j) < T1
                pts1 = [pts1 rts1(j)];
            end
        end
        pts1 = [pts1 T1];
        pts1 = sort(pts1);
%         t1 = [mean(pts1(1:2)); mean(pts1(2:3))];
        t1 = zeros(length(pts1)-1, 1);
        for j = 1:length(t1)
            t1(j) = mean(pts1(j:j+1));
        end
        tvecV1 = [zeros(length(t1),1) ones(length(t1),1) 2.*t1 3.*t1.^2];
        res = tvecV1*a1(:,i);
        for j = 1:length(t1)
            if sign(res(j)) ~= dir(i)
                ceq(1) = ceq(1)+diff(pts1(j:j+1));
            else
%                 c = c - diff(pts1(j:j+1));
            end
        end
        
        % phase 3 the velocity direction should always towards goal
        rts3 = roots([3*a3(4,i), 2*a3(3,i), a3(2,i)]);
        pts3 = 0;
        for j = 1:size(rts3,1)
            if rts3(j) > 0 && rts3(j) < T3
                pts3 = [pts3 rts3(j)];
            end
        end
        pts3 = [pts3 T3];
        pts3 = sort(pts3);
%         pts3 = sort([rts3; 0]);
%         t3 = [mean(pts3(1:2)); mean(pts3(2:3))];
        t3 = zeros(length(pts3)-1, 1);
        for j = 1:length(t3)
            t3(j) = mean(pts3(j:j+1));
        end
        tvecV3 = [zeros(length(t3),1) ones(length(t3),1) 2.*t3 3.*t3.^2];
        res = tvecV3*a3(:,i);
%         if sign(res(1)) ~= dir(i)
%             c = c+diff(pts3(1:2));
%         elseif sign(res(2)) ~= dir(i)
%             c = c+diff(pts3(2:3));
%         end
        for j = 1:length(t3)
            if sign(res(j)) ~= dir(i)
                ceq(2) = ceq(2)+diff(pts3(j:j+1));
            else
%                 c = c - diff(pts3(j:j+1));
            end
        end
    end
    
%     ceq(3) = -T3;
    ceq = ceq*1e20;
    c = -T3;
%     t1 = [T1/99; T1/2; T1*0.99];
%     tvecV1 = [zeros(3,1) ones(3,1) 2.*t1 3.*t1.^2];
%     res = tvecV1*[a11 a21];
%     for i = 1:3
%         if sum(sign(res(i,:)) == dir) == 2 % when all the direction along with wanted
%             c = c-1;
%         else
%             c = c+10;
%         end
%     end
%     
%     t3 = [T3/99; T3/2; T3*0.99];
%     tvecV3 = [zeros(3,1) ones(3,1) 2.*t3 3.*t3.^2];
%     res = tvecV3*[a13 a23];
%     for i = 1:3
%         if sum(sign(res(i,:)) == dir) == 2 % when all the direction along with wanted
%             c = c-1;
%         else
%             c = c+10;
%         end
%     end
%     
% %     c
% %     ceq = (c<=0)-1;
% % ceq = 0;
%     if c > 0
%         ceq = c;
%     else
%         ceq = 0;
%     end

end