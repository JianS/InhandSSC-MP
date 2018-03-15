function ret = AverageFilter( data, step )
% average filter

n = size(data,1);

ret = data;

for i = 1:n
    ret(i,:) = smooth(data(i,:), step);
end


end

