function o = FindMostRobustCurve(o)
% find the curve that fxc1 = -fxc2 where there is no external contact force
% the curve can only be solved numerically since there is no close form for
% the shape of the object.

% use discretization of y1 as base points
n1 = length(o.y1);

% Most Robust Curve
MRC = zeros(2, n1);

% temp storage for finger contact pos in B
pfB = zeros(2, o.n_f);
% temp storage for the friction cone edge direction
fedge_ = zeros(2, o.n_f);

o.l_MRC = 0;

for i = 1:n1
    MRC(1,i) = o.y1(i);
    
    [pfB(:,1), fN, ~] = GetPtOnObj(o.y1(i),1, o.obj_data);
        fN = -fN;
    tt = [-sign(fN(1))*fN(2); sign(fN(1))*fN(1)];

    fedge_(:,1) = fN + o.dy(1)*o.param.mu*tt;
    
    global opti_input_MRC
    opti_input_MRC.o = o;
    opti_input_MRC.pfB = pfB;
    opti_input_MRC.fedge_ = fedge_;
%     opti_input_MRC.param = o.param;
    
if i == 1
    x0 = o.y1(i);
else
    x0 = MRC(2,i-1);
end
    
    options = optimoptions(@fmincon,'Display','none', 'Algorithm', 'sqp',...
            'MaxIter', 200,'MaxFunEvals',1000, 'TolX', 1e-10);
        
%     if abs(x0 - 0.2) < 1e-16
%         options = optimoptions(@fmincon,'Display','iter-detailed', 'Algorithm', 'sqp',...
%             'MaxIter', 200,'MaxFunEvals',1000, 'TolX', 1e-10);
%     end
    if i==n1
        disp('');
    end
    [x,fval,exitflag,foutput] = fmincon(@o.fun_MRC, x0 ,[],[],...
        [],[], x0-0.05, x0+0.05, [], options);
    
%     if (x-o.p_f_B(2,1,2))*o.dy(2) > 0
        MRC(2,i) = x;
%     else
%         MRC(:,i) = [NaN; NaN];
%     end
    if i > 1
        o.l_MRC = o.l_MRC + norm(MRC(:,i)-MRC(:,i-1));
    end
end

o.MRC = MRC;


end