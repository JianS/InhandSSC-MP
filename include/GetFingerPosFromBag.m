function [t, p_f] = GetFingerPosFromBag(filepath)

% filepath = fullfile('/home/nxr/allegro_icra_method3/bagfiles','test.bag');
% filepath = 'test.bag';

% for the coke-bottle pinching exp, we only use the index and thumb of the
% allegro hand, which are finger 0 and 3
bag = rosbag(filepath);

bagselect0=select(bag,'Topic','/space_pose0');
% bagselect2=select(bag,'Topic','/space_pose2');
bagselect3=select(bag,'Topic','/space_pose3');

ts0 = timeseries(bagselect0, 'X', 'Y', 'Z');
% ts2 = timeseries(bagselect2, 'X', 'Y', 'Z');
ts3 = timeseries(bagselect3, 'X', 'Y', 'Z');

n = bagselect0.NumMessages;

% p_f = zeros(3,n,3);
p_f = zeros(3,n,2);

tt = ts0.Time;

t = tt-tt(1);

p_f(:,:,1) = ts0.Data';
% p_f(:,:,2) = ts2.Data';
p_f(:,:,2) = ts3.Data';

end