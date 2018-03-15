function ret = fun_MRC(x)
% define cost function for find the other contact position that gives fxc1
% = -fxc2

    global opti_input_MRC;

    o = opti_input_MRC.o;
    pfB = opti_input_MRC.pfB;
    pf = pfB;
    fedge_ = opti_input_MRC.fedge_;
%     param = opti_input_MRC.param;

    y2 = x;
    
    
    [pfB(:,2), fN, ~] = GetPtOnObj(y2, 2, o.obj_data);
    fN = -fN;
    tt = [-sign(fN(1))*fN(2); sign(fN(1))*fN(1)];

    fedge_(:,2) = fN + o.dy(2)*o.param.mu*tt;

    A = zeros(2);
    B = zeros(2,1);
    for j = 1:o.n_f
       tt = [fedge_(2,j); -fedge_(1,j)];
       A(j,:) = tt'*o.param.K_H(:,:,j);
       pf(:,j) = o.q_o(1:2,1) + pfB(:,j);
       c = pf(:,j) - o.p_a_H(:,j) - o.param.d0(:,j);
       B(j) = tt'*o.param.K_H(:,:,j)*c;
    end

    %             if i1==n1 && i2==n2
    %                 aa=1;
    %             end

    ph = A\B; % calculate the hand pos 

% %     o.q_h(1:2,i) = ph;
%     Ro = CalR2d(o.q_o(3,i));
%     Rh = CalR2d(o.q_h(3,i));
    Ro = CalR2d(0);
    Rh = CalR2d(0);
    
%     p_f = zeros(2, o.n_f);
    p_f_H = zeros(2, o.n_f);
    
    f_c = zeros(2, o.n_f);
    f_c_H = zeros(2, o.n_f);
    
    i = o.param.N1+1; 
    for j = 1:o.n_f
%         p_f(:,j) = o.q_o(1:2,i) + Ro*pfB(:,j);
        p_f_H(:,j) = Rh'*(o.q_o(1:2,i)-ph) + Rh'*Ro*pfB(:,j);

        d_H = p_f_H(:,j) - o.p_a_H(:,j) - o.param.d0(:,j);

        f_c_H(:,j) = -o.param.K_H(:,:,j)*d_H;
        f_c(:,j) = Rh*f_c_H(:,j);
    end

    wcB = zeros(3,1);
%     Ro = CalR2d(o.q_o(3,i));
%     RoI = inv(Ro);
    for j = 1:o.n_f
        fcB = Ro\f_c(:,j);
        tau = cross([pfB(:,j);0], [fcB;0]);
        w = [fcB; tau(3)];
        wcB = wcB + w;
    end
    tau = cross([o.param.p_g_B; 0], [Ro\o.param.G;0]);
    wgB = [Ro\o.param.G; tau(3)];
%     wgB = [RoI*o.param.G; 0];
    weB = -wcB - wgB;
    
    Wbar = o.WeB_perp;
    for i = 1:o.param.n_e*2
        Wbar(:,i) =  o.WeB_perp(:,i)/norm(o.WeB_perp(:,i));
    end
    
    weBbar = weB/norm(weB);

    d = min(Wbar'*weBbar);
    
%     ret = abs(sum(f_c(1,:)));
    ret = -d;
end