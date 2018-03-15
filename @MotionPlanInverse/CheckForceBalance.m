function ret = CheckForceBalance(o, i)
    wcB = zeros(3,1);
    Ro = CalR2d(o.q_o(3,i));
    RoI = inv(Ro);
    for j = 1:o.n_f
        fcB = RoI*o.f_c(:,i,j);
        tau = cross([o.p_f_B(:,i,j);0], [fcB;0]);
        w = [fcB; tau(3)];
        wcB = wcB + w;
    end
    tau = cross([o.param.p_g_B; 0], [RoI*o.param.G;0]);
    wgB = [RoI*o.param.G; tau(3)];
%     wgB = [RoI*o.param.G; 0];
    weB = -wcB - wgB;

    m = 2*o.param.n_e;
    f = ones(m,1);
    A = -eye(m);
    b = zeros(m,1);
    opt = optimoptions(@linprog, 'algorithm', 'dual-simplex','display','off');
    [~, ~, ret, ~ ] = linprog(f,A,b, o.WeB, weB,[],[],[], opt);
%     
%     if ~(ret==1)
% %         Ro
% %         RoI
%         ret
%         wgB
%         weB
%     end
%         f'*x

end