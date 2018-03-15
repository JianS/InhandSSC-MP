function o = BuildFeasiFingerPosMap(o, stepsize)
    ymin = zeros(o.n_f,1);
    ymax = zeros(o.n_f,1);
    dy = zeros(o.n_f,1);
    for j = 1:o.n_f
        dy(j) = sign(o.yfB_goal(j) - o.p_f_B(2,1,j) );
%         if dy(j) > 0
%             ymin(j) = o.p_f_B(2,1,j);
%             ymax(j) = max(o.obj_pts(2,:));
%         else
%             ymin(j) = min(o.obj_pts(2,:));
%             ymax(j) = o.p_f_B(2,1,j);
%         end

%         ymin(j) = min(o.obj_pts(2,:));
%         ymax(j) = max(o.obj_pts(2,:));
          ymin(j) = 0;
          ymax(j) = o.obj_data.h;
    end
    o.dy = dy;

    o.y1 = [ymin(1):stepsize:ymax(1) ymax(1)];
    o.y2 = [ymin(2):stepsize:ymax(2) ymax(2)];
    n1 = size(o.y1,2);
    n2 = size(o.y2,2);

    o.Iffeasi = zeros(n1,n2);
    o.Ystepsize = stepsize;

    i = o.param.N1+1; % use i=N1+1 as a temp storage spot
    fedge_ = zeros(2, o.n_f);
    for i1 = 1:n1
        [o.p_f_B(:,i,1), fN, ~] = GetPtOnObj(o.y1(i1),...
                1, o.obj_data);
            fN = -fN;
        tt = [-sign(fN(1))*fN(2); sign(fN(1))*fN(1)];

        fedge_(:,1) = fN + dy(1)*o.param.mu*tt;

        for i2 = 1:n2
            [o.p_f_B(:,i,2), fN, ~] = GetPtOnObj(o.y2(i2),...
                2, o.obj_data);
            fN = -fN;
            tt = [-sign(fN(1))*fN(2); sign(fN(1))*fN(1)];

            fedge_(:,2) = fN + dy(2)*o.param.mu*tt;

            A = zeros(2);
            B = zeros(2,1);
            for j = 1:o.n_f
               tt = [fedge_(2,j); -fedge_(1,j)];
               A(j,:) = tt'*o.param.K_H(:,:,j);
%                c = o.p_f_B(:,i,j) - o.p_a_H(:,j) - o.param.d0(:,j);
               o.p_f(:,i,j) = o.q_o(1:2,i)+o.p_f_B(:,i,j); %assuming Ro = I
               c = o.p_f(:,i,j) - o.p_a_H(:,j) - o.param.d0(:,j);
               B(j) = tt'*o.param.K_H(:,:,j)*c;
            end
            
%             if i1==n1 && i2==n2
%                 aa=1;
%             end

            ph = A\B; % calculate the hand pos 

            o.q_h(1:2,i) = ph;
            Ro = CalR2d(o.q_o(3,i));
            Rh = CalR2d(o.q_h(3,i));
            for j = 1:o.n_f
%                 o.p_f(:,i,j) = o.p_f_B(:,i,j);
                o.p_f(:,i,j) = o.q_o(1:2,i) + Ro*o.p_f_B(:,i,j);
                o.p_f_H(:,i,j) = Rh'*(o.q_o(1:2,i)-o.q_h(1:2,i))...
                   + Rh'*Ro*o.p_f_B(:,i,j);

                d_H = o.p_f_H(:,i,j) - o.p_a_H(:,j) - o.param.d0(:,j);

                o.f_c_H(:,i,j) = -o.param.K_H(:,:,j)*d_H;
                o.f_c(:,i,j) = Rh*o.f_c_H(:,i,j);
            end

            o.Iffeasi(i1,i2) = (CheckForceBalance(o,i) == 1);


        end
    end

    B = bwboundaries(o.Iffeasi);

    if length(B) > 1
        disp('There are two feasible regions!');
    end
    
    b = B{1};
    nfb = length(b);
%     o.feasiBoundary = zeros(2, nfb);
    fB = [];
    for i = 1:nfb
%         o.feasiBoundary(1,i) = o.y1(b(i,1));
%         o.feasiBoundary(2,i) = o.y2(b(i,2));
        
        tt = [o.y1(b(i,1)); o.y2(b(i,2))];
        if i < 3
            fB = [fB tt];
        else
            d = tt - fB(:,end);
            slope_now = d(2)/d(1);
            
            dl = fB(:,end) - fB(:,end-1);
            slope_last = dl(2)/dl(1);
            if slope_now == slope_last
                fB(:,end) = tt;
            else
                fB = [fB tt];
            end
        end
    end

    o.feasiBoundary = fB;
end

