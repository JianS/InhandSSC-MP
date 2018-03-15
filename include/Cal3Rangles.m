function JPos = Cal3Rangles(q_f, hand_end, param)
% Calculate the joint angles of the 3R robot

    n = size(hand_end,1);

    offset2 = atan(param.wamCO/0.55); % offset angles due to the joint 4 of WAM is not aline with the center
    offset4 = atan(param.wamCO/0.3);


    JPos = zeros(n,3); % joint angles of the 3R robot

    for i = 1:n
        [t1, t2] = TwoR_InvKin(hand_end(i,1), hand_end(i,2), param);
        JPos(i, 1) = t1 + offset2 - pi/2;
        JPos(i, 2) = -t2 + offset4 + offset2;
        JPos(i, 3) = -q_f(3,1,i) + t1 + t2 - offset4;
    end


end