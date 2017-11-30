function [] = runtest(mapfile, robotstart, targetstart)

envmap = load(mapfile);

close all;

RES = 0.1;

% Parameters
r = 0.25;
t_delta = 0.1;

%draw the environment
image(envmap'*255);

%current positions of the target and robot
robotpos = robotstart;
targetpos = targetstart;

[plan, plan_length] = planner_wrapper(envmap, robotpos, targetpos);
fprintf(1, 'plan of length %d was found\n', plan_length);

hold on 

%draw the plan
for i=1:(plan_length-1)
%     x = [plan(i,1)/RES plan(i+1,1)/RES];
%     y = [plan(i,2)/RES plan(i+1,2)/RES];
%     plot(x,y, 'c-');
%     pause(0.1);
 plot_trajectory_di(plan(i,1),plan(i,2),plan(i,3),plan(i,4),plan(i+1,1),plan(i+1,2),plan(i+1,3),plan(i+1,4),plan(i+1,5),t_delta,r,RES);
end

end