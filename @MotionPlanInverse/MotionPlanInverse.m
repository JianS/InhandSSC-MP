classdef MotionPlanInverse

    properties
        % input
        q_o
        dq_o
%         q_o_init
%         q_h_init
%         yfB_init % init contact y positions
        yfB_goal
        pfB_goal
        p_a_H % anchor position in the Hand frame H. In this case, 
                %the anchor pos respect to the hand are fixed.
      
%         obj_pts % obj contour pts in B frame
        obj_pts_plot % obj contour pts for plot
        obj_data % struct store all obj geo info
        
        % output
        q_h
        dq_h
        p_a
        dp_a
        p_f
        p_f_B
        p_f_H
        Ifslide
        f_c
        f_N
        f_t
        
%         f_e % external contact force
%         f_e_B
        WeB % W matrix that contains friction cone wrenches in B frame
        WeB_perp % each row is a vector perpendicular to a face of wrench cone
        
        % used in simulations
        dp_f
        dp_f_B
        dp_f_H
        
        f_c_H
        f_N_H
        f_t_H
        
        
        n
        n_f
        param

        fedge % the vector of friction cone edge on which finger f_c lies 
        
        Iffeasi % for build the feasible finger contact pos map
        Ystepsize % the stepsize of y when build the feasiable map
        y1
        y2
        dy % desired y sliding direction
        feasiBoundary % coordinates of the boundary of the feasiable region
        
        MRC % most robust curve in the y1-y2 space; when fxc1 = -fxc2
        l_MRC % length of the MRC
        
        yfBpath % planned path of yfBs
        planResult % including the planing result info, for the algorithm of 
               % two cubic poly, including S, G, S1, G1, T1, T2, T3, details
               % can be found in PlanSlidingTraj.m
    end
    
    methods (Static)
        DrawSquare(x,y,r,linecolor, linestyle, fillcolor)
    end
    
    methods
        function o = MotionPlanInverse(param, qo, dqo, qh_init, y_init,...
                y_goal, obj_data_in, FFPmapPath)
            % if pre-calculated, load feasi region data
            if exist(FFPmapPath,'file')
                data = load(FFPmapPath);
                % tell if the spring properties are the same
                if all((param.K_H == data.param.K_H) == 1) ==0
                    disp(['The K_H in "' FFPmapPath '" do not match. Regenerate the map.']);
                    delete(FFPmapPath);
                elseif all(((param.d0+param.AnchorPos_H) ==...
                        (data.param.d0+data.param.AnchorPos_H)) == 1) ==0
                    disp(['The d0 in "' FFPmapPath '" do not match. Regenerate the map.']);
                    delete(FFPmapPath);
                else
                    o.Iffeasi = data.Iffeasi;
                    o.Ystepsize = data.Ystepsize;
                    o.y1 = data.y1;
                    o.y2 = data.y2;
                    o.dy = data.dy;
                    o.feasiBoundary = data.feasiBoundary;

                    o.MRC = data.MRC;
                    o.l_MRC = data.l_MRC;
                end
                
            end
            
            o.n = param.N;
            o.n_f = param.n_f;
            o.param = param;
            
            o.yfB_goal = y_goal;
            o.pfB_goal = zeros(2, o.n_f);
            
            o.q_o = qo;
            o.dq_o = dqo;
%             o.q_o_init = qo_init;
%             o.q_h_init = qh_init;
%             o.yfB_init = y_init;
            
            o.p_a_H = zeros(2, o.n_f);
            o.p_a_H(:,1) = param.AnchorPos_H(:,1);
            o.p_a_H(:,2) = param.AnchorPos_H(:,2);

%             o.obj_pts = objpts_input;
%             o.obj_pts_plot = obj_pts_plot;
            o.obj_data = obj_data_in;
            o.obj_pts_plot = obj_data_in.obj_pts_plot;
            
            o.q_h = zeros(3, o.n); % hand pos
            o.q_h(:,1) = qh_init;
            o.dq_h = zeros(3, o.n); % hand vel

            o.p_a = zeros(2, o.n, o.n_f);
            o.dp_a = zeros(2, o.n, o.n_f);
           
            o.p_f = zeros(2, o.n, o.n_f);
            o.p_f_B = zeros(2, o.n, o.n_f);
            o.p_f_H = zeros(2, o.n, o.n_f);
            o.Ifslide = zeros(o.n, o.n_f);
            
            o.p_f_B(2,1,:) = y_init; % set initial contact points
%             for j = 1:o.n_f
%                 [o.p_f_B(:,1,j), ~, ~] = GetPtOnObj(o.p_f_B(2,1,j), j, o.obj_pts);
%             end

            for j = 1:o.n_f
                % set goal finger contact pts
                [o.pfB_goal(:,j),~,~] = GetPtOnObj(o.yfB_goal(j), j, o.obj_data);
            end
            
            % calculate external wrenches on the friction cone
            o.WeB = zeros(3,2*param.n_e);
            for j = 1:param.n_e
                nn = param.nhat_e_B(:,j);
                tt = [-nn(2); nn(1)];
                fL = nn + param.mu_e*tt;
                fR = nn - param.mu_e*tt;
                
                peB = param.p_e_B(:,j);
                
                w = cross([peB;0],[fL;0]);
                o.WeB(:,2*(j-1)+1) = [fL; w(3)];
                w = cross([peB;0],[fR;0]);
                o.WeB(:,2*(j-1)+2) = [fR; w(3)];
                
            end
            
            o.WeB(:,[3 4]) = o.WeB(:,[4 3]);
            for j = 1:2*param.n_e
                w1 = o.WeB(:, j);
                if j < 2*param.n_e
                    w2 = o.WeB(:, j+1);
                else
                    w2 = o.WeB(:, 1);
                end
                
                o.WeB_perp(:,j) = cross(w2, w1);
            end
            
            o.f_c = zeros(2, o.n, o.n_f);
            o.f_N = zeros(2, o.n, o.n_f);
            o.f_t = zeros(2, o.n, o.n_f);
            o.f_c_H = zeros(2, o.n, o.n_f);
            o.f_N_H = zeros(2, o.n, o.n_f);
            o.f_t_H = zeros(2, o.n, o.n_f);
            o.dp_f = zeros(2, o.n, o.n_f);
            o.dp_f_B = zeros(2, o.n, o.n_f);
            o.dp_f_H = zeros(2, o.n, o.n_f);
           

        end
        
        ret = CheckForceBalance(o, i)
        
        [ret,o] = CheckInit(o)
        
        o = BuildSticking(o)
       
        animationMPI(o, speed, mode, fontsize,  ss, Ifrecord, videoname)
        
        o = BuildFeasiFingerPosMap(o, stepsize)
        
        o = FindMostRobustCurve(o)
        
        o = PlanSlidingPath(o)
        
        o = PlanSlidingPath2(o)
        
        o = PlanSlidingTraj(o)
        
        PlotInit(o, pfH_init, fontsize, ss, figureNum)
            
        PlotIffesi(o, fontsize, r)
       
        function PlotPath(o, fs, figureNum, linewidthMUL)
            if ~isempty(figureNum)
                figure(figureNum)
            else 
                figure
            end
            
            if nargin == 3
                linewidthMUL = 1;
            end
            
            PlotIffesi(o, fs)
            
                   
            % initial point
            scatter(o.p_f_B(2,1,1), o.p_f_B(2,1,2), 120*linewidthMUL^2, 's', 'MarkerEdgeColor','k',...
                      'MarkerFaceColor','b', 'LineWidth',1*linewidthMUL)
            % goal point
            scatter(o.pfB_goal(2,1), o.pfB_goal(2,2), 500*linewidthMUL^2, 'p', 'MarkerEdgeColor','k',...
                      'MarkerFaceColor','r', 'LineWidth',1*linewidthMUL)

            
            plot(o.MRC(1,:), o.MRC(2,:), 'r', 'linewidth',3*linewidthMUL)
            
            plot(o.yfBpath(1,:), o.yfBpath(2,:), 'k', 'linewidth',2.5*linewidthMUL)
            
            
            
            % plot S1
            scatter(o.planResult.S1(1), o.planResult.S1(2), 60*linewidthMUL^2, 'o', 'MarkerEdgeColor','k',...
              'MarkerFaceColor','k', 'LineWidth',1*linewidthMUL)
            % plot G1
            scatter(o.planResult.G1(1), o.planResult.G1(2), 60*linewidthMUL^2, 'o', 'MarkerEdgeColor','k',...
              'MarkerFaceColor','k', 'LineWidth',1*linewidthMUL)
          
            p1 = [o.p_f_B(2,1,1); o.p_f_B(2,1,2)];
            p2 = p1 + inf*o.dy;
            axis([min([p1(1) p2(1)]) max([p1(1) p2(1)])...
                min([p1(2) p2(2)]) max([p1(2) p2(2)])])
        end
            
        function o = BuildSliding(o)
            % build trejs based on the result from PlanSlidingPath1 or 2
            np = max([1, length(o.yfBpath)-1]);
            
            piecelength = zeros(np,1);
            nn = zeros(np,1);
            for i = 1:np
                piecelength(i) = norm(o.yfBpath(:,i+1)-o.yfBpath(:,i));
            end
            totallength = sum(piecelength);
            
            s=o.param.N1;
            for i = 1:np
                if i < np
                    nn(i) = round(piecelength(i)/totallength*o.param.N2);
                else
                    nn(i) = o.param.N2 - sum(nn);
                end
                
                vy = (o.yfBpath(:,i+1)-o.yfBpath(:,i))/nn(i);
                
                
                for ii = (s+1):(s+nn(i))
%                     if ii == o.param.N
%                         1;
%                     end
                    y = o.yfBpath(:,i) + (ii-s)*vy;
            o = Calculate(o, y,ii);
                end
                s = ii;
            end
        end
        
        o = Calculate(o, y, i)
        
        [S1, G1, n2min, n2max] = FindS1G1(o, S, G, rs, rg)

    end
    
    methods (Static)
        M = CalCubicPolyMatrix(T) % Matrix for cal cubic poly
        
        ret = fun_plan(x) % optimal function
        [c, ceq] = constraints(x) % constraints for optimization
        
        ret = fun_MRC(x) % optimal function for MRC
    end
    
    
end