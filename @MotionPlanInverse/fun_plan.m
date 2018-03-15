function ret = fun_plan(x)

global opti_input

    S = opti_input.S;
    G = opti_input.G;
    o = opti_input.o;
    n_f = size(S,1);
    
    T1 = x(1)*opti_input.Tslide;
    T2 = x(2)*opti_input.Tslide;
    rs = x(3);
    rg = x(4);
    
    T3 = opti_input.Tslide - T1 - T2;

    %% find S1 and G1
    [S1, G1, n2min, n2max] = o.FindS1G1(S, G, rs, rg);
    
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
    
%     ret = -l2;
    
    vnorm = l2/T2; % the speed magnitude during T2
    v1 = vnorm*((o.MRC(:,n2max)-S1)/ls1);
    v3 = vnorm*((G1-o.MRC(:,n2min))/lg1);
    
    %%
    % calculate
%     v = (G1-S1)/T2;
    M1 = o.CalCubicPolyMatrix(T1);
    M3 = o.CalCubicPolyMatrix(T3);
    
%     a1 = zeros(4, n_f); a3 = zeros(4,n_f);
    a1 = M1\[S'; zeros(1,n_f); S1'; v1'];  % phase 1 trej
    a3 = M3\[G1'; v3'; G'; zeros(1,n_f)];  % phase 3 trej
    
    res1 = []; res3 = [];
    for i = 1:n_f % find the highest vel
        % phase 1 
        rts1 = roots([6*a1(4,i), 2*a1(3,i)]);
%         pts1 = sort([rts1; T1]);
        pts1 = 0;
        for j = 1:size(rts1,1)
            if rts1(j) > 0 && rts1(j) < T1
                pts1 = [pts1; rts1(j)];
            end
        end
        pts1 = [pts1; T1];

        tvecV1 = [zeros(length(pts1),1) ones(length(pts1),1) 2.*pts1 3.*pts1.^2];
        res1 = [res1; tvecV1*a1(:,i)];
        
        
        % phase 3 
        rts3 = roots([6*a3(4,i), 2*a3(3,i)]);
        pts3 = 0;
        for j = 1:size(rts3,1)
            if rts3(j) > 0 && rts3(j) < T3
                pts3 = [pts3; rts3(j)];
            end
        end
        pts3 = [pts3; T3];
        tvecV3 = [zeros(length(pts3),1) ones(length(pts3),1) 2.*pts3 3.*pts3.^2];
        res3 = [res3; tvecV3*a3(:,i)];

    end
%     maxV = max([max(abs(res1)) max(abs(res3))]);
    maxV = max(abs(res1))^2 + max(abs(res3))^2;
    
    ret = -l2 + maxV*0.5;
    
end