function [] = runtest(mapfile, robotstart, targetstart)

envmap = load(mapfile);

close all;

%draw the environment
image(envmap'*255);

%current positions of the target and robot
robotpos = robotstart;
targetpos = targetstart;

[plan, plan_length] = planner_wrapper(envmap, robotpos, targetpos);
fprintf(1, 'plan of length %d was found\n', plan_length);

%draw the plan
end