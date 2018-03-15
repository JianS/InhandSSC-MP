function pos = GetPtPosOnCircle(s, n, param)
% get the position of a point on the circle in Body frame
% 's' -- the 1dof param define where the point is on the circle
%        0 means at the top of the arc;
%        1 means at the bottom of the arc.
% 'theta' -- angle of the point to the center from the vertical in
% counterclock direction;
% 'n' -- tell which finger:
%        1 for the left finger;
%        2 for the right finger.

if n == 1
    theta = param.thetaStart1 + s*(param.thetaEnd1 - param.thetaStart1);
    o = param.o1;
elseif n == 2
    theta = param.thetaStart2 + s*(param.thetaEnd2 - param.thetaStart2);
    o = param.o2;
end

x = -param.r*sin(theta) + o(1);
y = param.r*cos(theta) + o(2);

pos = [x;y];

end

