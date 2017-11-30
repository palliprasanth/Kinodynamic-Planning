function [] = plot_trajectory_di(x1,x2,x3,x4,y1,y2,y3,y4,t_star,t_delta,r,RES)

t = t_delta;
scaling = 3;

while (t <= t_star)
    x = y1 + y3*(t - t_star) - (((4*r*(x3 - y3))/t_star - (6*r*(x1 - y1 + t_star*x3))/t_star^2)*(t - t_star)^2)/(2*r) - (((6*r*(x3 - y3))/t_star^2 - (12*r*(x1 - y1 + t_star*x3))/t_star^3)*(t - t_star)^3)/(6*r);
    y =  y2 + y4*(t - t_star) - (((4*r*(x4 - y4))/t_star - (6*r*(x2 - y2 + t_star*x4))/t_star^2)*(t - t_star)^2)/(2*r) - (((6*r*(x4 - y4))/t_star^2 - (12*r*(x2 - y2 + t_star*x4))/t_star^3)*(t - t_star)^3)/(6*r);
    
    x = (x/RES);
    y = (y/RES);
    
    t = t + t_delta;
    scatter(x,y,'MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0 .7 .7]);
    pause(t_delta/scaling);
end


end

