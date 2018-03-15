function f = fun( mu )
% describe optimization cost function
global opti_input

[ ~, ~, ~, s, ~, ~, cost] = Simulate(opti_input.s_init, opti_input.q_o, opti_input.dq_o,...
    opti_input.q_h, opti_input.dq_h, opti_input.p_e_H, opti_input.dp_a_H, opti_input.n_cam,...
    mu, opti_input.param);
% w = 1000;
% G = [w 0 0; 0 w 0; 0 0 w];

% f = norm( G*( s(opti_input.n_cam,:)' - opti_input.s_exp_end') );

f = mean(abs(s-opti_input.s_exp),1)*[1;1;1] + cost;

end

