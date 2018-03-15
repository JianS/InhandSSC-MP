function hand_end = Calhandend(q_f , param)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
n = size(q_f,3);

hand_end = zeros(n,2);
for i = 1:n
    x_f = q_f(1,1,i); y_f = q_f(2,1,i);
    
    rm = [cos(q_f(3,1,i)) -sin(q_f(3,1,i)); sin(q_f(3,1,i)) cos(q_f(3,1,i))];

    hand_end_b = rm * [0; -param.l_hand];
    hand_end(i,:) = [x_f; y_f] + hand_end_b; 
end

end

