function dR = CaldR2d( theta, dtheta )
% Calculate rotation matrix
dR = [-sin(theta) -cos(theta); cos(theta) -sin(theta)]*dtheta;


end

