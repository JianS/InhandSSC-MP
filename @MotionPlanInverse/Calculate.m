function o = Calculate(o, y, i)
    Ro = CalR2d(o.q_o(3,i));
    Rh = CalR2d(o.q_h(3,i));

%             ret = 1;

    fedge_ = zeros(2, o.n_f);

    A = zeros(2);
    B = zeros(2,1);
    for j = 1:o.n_f
        [o.p_f_B(:,i,j), fN, ~] = GetPtOnObj(y(j), j, o.obj_data);

        dy = o.yfB_goal(j) - o.p_f_B(2,1,j);

        fN = -fN;
        tt = [-sign(fN(1))*fN(2); sign(fN(1))*fN(1)];

        fedge_(:,j) = fN + sign(dy)*o.param.mu*tt;

        tt = [fedge_(2,j); -fedge_(1,j)];
        A(j,:) = tt'*o.param.K_H(:,:,j);
        c = o.p_f_B(:,i,j) - o.p_a_H(:,j) - o.param.d0(:,j);
        B(j) = tt'*o.param.K_H(:,:,j)*c;
    end

    o.q_h(1:2,i) = A\B + o.q_o(1:2,i);% this is assuming object do not rotate

    for j = 1:o.n_f
        [o.p_f_B(:,i,j), N_B, ~] = GetPtOnObj(y(j), j, o.obj_data);
        if i > 1
            o.dp_f_B(:,i,j) = (o.p_f_B(:,i,j) - o.p_f_B(:,i-1,j))/o.param.t_int;
            if norm(o.dp_f_B(:,i,j)) == 0
                o.Ifslide(i,j) = 0;
            else
                o.Ifslide(i,j) = 1;
            end
        end

        o.p_f(:,i,j) = o.q_o(1:2,i) + Ro*o.p_f_B(1:2,i,j);
        o.p_f_H(:,i,j) = Rh'*(o.q_o(1:2,i)-o.q_h(1:2,i)) + Rh'*Ro*o.p_f_B(:,i,j);

        d_H = o.p_f_H(:,i,j) - o.p_a_H(:,j) - o.param.d0(:,j);
%                 d = o.q_h(1:2,i) + Rh*d_H;

        o.f_c_H(:,i,j) = -o.param.K_H(:,:,j)*d_H;
        o.f_c(:,i,j) = Rh*o.f_c_H(:,i,j);

        Nhat_B = N_B/norm(N_B);
        Nhat = Ro*Nhat_B; %N = Ro*N_B;
        Nhat_H = Rh'*Nhat;

        if o.f_c(:,i,j)'*Nhat > 0
            disp(['i=' num2str(i) ', finger ' num2str(j) ' breaking contact!']);
        end
        o.f_N_H(:,i,j) = (o.f_c_H(:,i,j)'*Nhat_H)*Nhat_H;
        o.f_t_H(:,i,j) = o.f_c_H(:,i,j) - o.f_N_H(:,i,j);
        o.f_N(:,i,j) = Rh*o.f_N_H(:,i,j);
        o.f_t(:,i,j) = Rh*o.f_t_H(:,i,j);

%                 if norm(o.f_t(:,i,j)) >= o.param.mu*norm(o.f_N(:,i,j))
%                     disp(['finger ' num2str(j) ' initially not sticking!']);
% %                     ret = 0;
%                 end

        o.p_a(:,i,j) = o.q_h(1:2,i) + Rh*o.p_a_H(:,j);

    end
end