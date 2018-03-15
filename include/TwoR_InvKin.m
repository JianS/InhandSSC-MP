
function [ theta1, theta2 ] = TwoR_InvKin( Cx, Cy, param)
% Calculating the inverse kinematic of the 2-R robot arm;
% parameters are stated in the parameters.m
% A - base point of robot arm, (xc, yc, zc) = (0,0,0)
% B - elbow point
% C - end-point, (Cx, Cy)
% The zero of theta1 is set to be y positive direction
% The zero of theta2 is set to be along with link 1
%%
% l = 0.45; Cx = 0.0; Cy = 0.5;
% 
%     l1 = l+0.1; l2 = l-0.1;
%     l1 = l+0.; l2 = l-0.;

l1 = param.robot_l1;
l2 = param.robot_l2;

    r = norm([Cx; Cy]);

    originX=0;
    
    
    %% old algorithm--when l1=l2=l     
%     if Cx - originX < 0
%         phi = -atan(Cy/Cx);
%     else
%         phi = pi - atan(Cy/Cx);
%     end
%     
%     alpha = acos(r/(2*l));
%     
%     theta1 = pi/2 + alpha - phi;
%     
%     theta2 = asin(r/(2*l)) - phi;
%     theta2 = theta2 - theta1;
%     
%     theta1/pi*180
%     theta2/pi*180
%% new algorithm 
%  check: http://www.slideshare.net/DamianGordon1/forward-kinematics

    cos2 = (r^2-l1^2-l2^2)/(2*l1*l2);% cos(theta2)
    sin2 = -(1-cos2^2)^0.5;
    
%     theta2 = acos(cos2);atan(sin2/cos2);
    
%     theta2 = 2*atan(( ((l1^2+l2^2)^2-(r^2)) / ((r^2)-(l1^2-l2^2)^2) )^0.5)
    
    theta2 = -2*atan( ((1-cos2)/(1+cos2))^0.5 );
    
    
    if Cx - originX >= 0
        phi = atan(Cy/Cx);
    else
        phi = pi + atan(Cy/Cx);
    end
    theta1 = phi - atan(l2*sin2/(l1+l2*cos2)) - pi/2;
    
%     theta1/pi*180
%     theta2/pi*180
%     [param.robot_l1*cos(theta1+pi/2)+param.robot_l2*cos(theta1+theta2+pi/2),...
%         param.robot_l1*sin(theta1+pi/2)+param.robot_l2*sin(theta1+theta2+pi/2)]
    
end

