function SimplePlot3( X, Y , titletext)
%SIMPLEPLOT3 plot 3 row matrix Y with X in 3 subplots
if nargin < 3
    titletext = '';
end

figure

subplot(3,1,1)
plot(X, Y(1,:))
title(titletext);

subplot(3,1,2)
plot(X, Y(2,:))

if (size(Y,1)>2)
subplot(3,1,3)
plot(X, Y(3,:))
end

end

