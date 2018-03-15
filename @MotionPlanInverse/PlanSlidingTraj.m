function o = PlanSlidingTraj(o)
% this planning algorithm tries to reach y1=y2 from Start(S) and then goes
% to the Goal(G) with cubic curves. the y1=y2 line is mostly stable because
% no horizontal table contact forces occur.

% addpath('@MotionPlanInverse/')
    % start
    S = [o.p_f_B(2,1,1); o.p_f_B(2,1,2)];
    % goal
    G = o.pfB_goal(2,:)';
    
    if ~inpolygon(G(1), G(2), o.feasiBoundary(1,:), o.feasiBoundary(2,:))
        disp('Goal positions cannot be achieved!');
        return
    end

    o.yfBpath = S;
    
    Tslide = o.param.T2;
    
    % setting initial values
    T1 = 0.25*Tslide; % phase 1 when goes to y1=y2 from S
    T2 = 0.5*Tslide; % phase 2 goes along y1=y2
     
    rs = 0.63;0.9; % ratio which determines where S1 is on MRC rs=0 at origin 
    rg = 0.35;0.3; % ratio which determines where G1 is on MRC
    
    
    %% optimazation for the choice of T1,T2,rs and rg
    % set initial conditions
    x0 = zeros(4,1);
    x0(1) = T1/Tslide; % normalize T1
    x0(2) = T2/Tslide;
    x0(3) = rs;
    x0(4) = rg;

    global opti_input
    opti_input.Tslide = Tslide;
    opti_input.S = S;
    opti_input.G = G;
    opti_input.o = o;
    
    % optimize option
options = optimoptions(@fmincon,'Display','iter-detailed', 'Algorithm', 'sqp',...
            'MaxIter', 50,'MaxFunEvals',1000, 'TolX', 1e-10);
if(1)
    % set the range for x elements: T > 0 and 0 < r < 0.5
    lb = [0.01; 0.01; 0.01; 0.01];
    ub = [0.98; 0.98; 1; 1];
    A = [1 1 0 0]; b = 0.99;
    [x,fval,exitflag,foutput] = fmincon(@o.fun_plan, x0 ,A,b,...
        [],[], lb, ub, @o.constraints, options);
    exitflag
    x0
    x
else
    x = x0;
end

    %%
    
    % Build the entire sliding phase
    t_int = o.param.t_int;
    
    T1 = x(1)*Tslide; % phase 1 when goes to y1=y2 from S
    T2 = x(2)*Tslide; % phase 2 goes along y1=y2
    T3 = Tslide - T1 -T2; % phase 3 when goes to G from y1=y2
    
    rs = x(3); % ratio which determines where S1 is in between of S and G 
%     tem = S + (G-S)*rs;
%     S1 = [mean(tem), mean(tem)]; % point at y1=y2 when t=T1
    rg = x(4); % ratio which determines where G1 is in between of S and G
%     tem = G + (S-G)*rg;
%     G1 = [mean(tem), mean(tem)]; % point at y1=y2 when t=T1+T2
    [S1, G1, n2min, n2max] = o.FindS1G1(S, G, rs, rg);
%     S1 = S1'; G1 = G1'; 
    
    o.planResult.x0 = x0;
    o.planResult.x = x;
    o.planResult.T1 = T1;
    o.planResult.T2 = T2;
    o.planResult.T3 = T3;
    
    o.planResult.S = S;
    o.planResult.S1 = S1;
    o.planResult.G = G;
    o.planResult.G1 = G1;
    
    % calculate
%     v = (G1-S1)/T2;

    if S1 == o.MRC(:,n2max) % when S1 is at a node of MRC
        n2max = n2max - 1; 
    end
    if G1 == o.MRC(:,n2min) % when G1 is at a node of MRC
        n2min = n2min + 1; 
    end
    
    % calculate length of T2 traj -- l2
    ls1 = norm(S1-o.MRC(:,n2max));
    lg1 = norm(G1-o.MRC(:,n2min));
    l2 = ls1 + lg1;
    if n2max > n2min
        for i = n2min:n2max-1
            l2 = l2 + norm(o.MRC(:,i+1)-o.MRC(:,i));
        end
    elseif n2max < n2min
        l2 = norm(S1 - G1);        
    end
   
    
    vnorm = l2/T2; % the speed magnitude during T2
    v1 = vnorm*((o.MRC(:,n2max)-S1)/ls1);
    v3 = vnorm*((G1-o.MRC(:,n2min))/lg1);
    
    
    M1 = o.CalCubicPolyMatrix(T1);
    a11 = M1\[S(1); 0; S1(1); v1(1)]; % finger 1 trej for phase 1
    M3 = o.CalCubicPolyMatrix(T3);
    a13 = M3\[G1(1); v3(1); G(1); 0]; % finger 1 trej for phase 3
    
    a21 = M1\[S(2); 0; S1(2); v1(2)]; % finger 2 trej for phase 1
    a23 = M3\[G1(2); v3(2); G(2); 0]; % finger 2 trej for phase 3
    
    
    n1 = round(T1/t_int);
    n2 = round(T2/t_int);
    n3 = o.param.N2 - n1 - n2;
    
    o.yfBpath = S;
    
    % build sliding traj of T1 in Tslide
    istart = o.param.N1;
    for i = istart+1 : istart+n1
        t1 = (i-istart)*t_int;
        tvec = [1 t1 t1^2 t1^3];
        y = tvec*[a11 a21];
        o = Calculate(o, y, i);
        o.yfBpath = [o.yfBpath y'];
    end
    
    % build sliding traj of T2 in Tslide
    istart = i;
    v = l2/n2;
    ypre = S1;
    iMRC = n2max;
    A = o.MRC(:, iMRC);
    B = S1;
    vdir = (A-B)/norm(A-B);
    for i = istart+1 : istart+n2
%         v = (G1-S1)/n2;
        % A and B are the end points of the current MRC segment
        
        if norm(ypre-A) < v % when goint out of the current segment
            vleft = v - norm(ypre-A);
            iMRC = iMRC-1;
            A = o.MRC(:, iMRC);
            B = o.MRC(:, iMRC+1);
            vdir = (A-B)/norm(A-B);
            y = B + vleft*vdir;
        else
            y = ypre + v*vdir;
        end
        
%         y = S1 + v*(i-istart);
        if i>o.n
            disp('i > total n!!');
            pause();
        end
        o = Calculate(o, y, i);
        ypre = y;
        o.yfBpath = [o.yfBpath y];
    end
    
    
    % build sliding traj of T3 in Tslide
    istart = i;
    for i = istart+1 : istart+n3
        t3 = (i-istart)*t_int;
        tvec = [1 t3 t3^2 t3^3];
        y = tvec*[a13 a23];
        o = Calculate(o, y, i);
        o.yfBpath = [o.yfBpath y'];
    end
end