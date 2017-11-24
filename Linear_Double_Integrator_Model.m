clc
clear all
close all

%% Initial and Final States
X_initial = [0;0;10;10];
X_final = [20;15;0;0];

%% Linear Double Integrator System
A = [zeros(2) eye(2);zeros(2) zeros(2)];
B = [zeros(2);eye(2)];
c = zeros(4,1);
r = 0.25;
R = r*eye(2);
R_inverse = eye(2)/R;
time_out = 20;

%% Linear Double Integrator System
syms t expA(t) expAt(t)
syms G(t) x_bar(t) td
syms tau d(tau) dCost(tau) Cost(tau)
syms y(t) u(t) expAdash(t) traj(t)

expA(t) = eye(4) + A*t; % Exponential of At
expAt(t) = eye(4) + A'*t; % Exponential of A't

%% Computing Weighted Controllability Grammian
% Equations 6 and 8 from the Kinodynamic RRT* paper

G(t) = int(expA(t-td)*B*R_inverse*B'*expAt(t-td),td,0,t);
x_bar_term = int(expA(t-td)*c,td,0,t);
x_bar(t) = expA(t)*X_initial + x_bar_term;

%% Computing Optimal Arrival Time of the Trajectory

% Cost(tau) = tau + (X_final-x_bar(tau))'*inv(G(tau))*(X_final-x_bar(tau));

d(tau) = eye(4)/G(tau)*(X_final-x_bar(tau));
dCost(tau) = 1 - 2*(A*X_final + c)'*d(tau) - d(tau)'*B*R_inverse*B'*d(tau);

Tau_roots = double(vpasolve(dCost == 0,[0 time_out]));
t_star = Tau_roots(Tau_roots>0 & (abs(imag(Tau_roots))<1e-9))

% if (isempty(t_star))
%    t_star = double(vpasolve(dCost == 0));
% end

% figure
% time = 2:0.02:20;
% plot(time,Cost(time));

%% Computing the Optimal Control Input and Trajectory
y(t) = expAt(t_star-t)*d(t_star);
u(t) = R_inverse*B'*y(t);

% figure
% hold on
% time = 0:0.1:t_star;
% for i = 1:length(time)
%     u_temp = double(u(time(i)));
%     scatter(time(i),u_temp(1),'k*');
%     scatter(time(i),u_temp(2),'b*');
% end

A_dash = [A B*R_inverse*B';zeros(4) -A'];
c_dash = [c;zeros(4,1)];

expAdash(t) = eye(8) + A_dash*t + A_dash^2*t^2/2 + A_dash^3*t^3/6;
traj(t) = expAdash(t-t_star)*[X_final;d(t_star)]; % + integral = 0 for double integrator model

figure
grid on
axis([-5 25 -5 25])
hold on
time = 0:0.05:t_star;
for i = 1:length(time)
    traj_temp = double(traj(time(i)));
    scatter(traj_temp(1),traj_temp(2),'b.');
    pause(0.01)
end