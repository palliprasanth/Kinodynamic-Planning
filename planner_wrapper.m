function[plan, plan_length] = planner_wrapper(envmap, start_state, goal_state)

%call the planner in C
tic;
[plan, plan_length] = planner(envmap, start_state, goal_state);
toc;

end
