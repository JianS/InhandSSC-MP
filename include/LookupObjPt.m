function [ i, iUP, iDOWN ] = LookupObjPt( y, obj_pts )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
min_i = 1;
max_i = size(obj_pts,2);

while( (max_i-min_i) > 1 )
    mid = (min_i + max_i)/2;
    mid = round(mid);
    if (y < obj_pts(2,mid) )
        min_i = mid;
    else
        max_i = mid;
    end
end

iUP = max_i;
iDOWN = min_i;

if ( abs(y - obj_pts(2,min_i)) > abs(y-obj_pts(2,max_i)) )
    i = max_i;
else
    i = min_i;
end

end

