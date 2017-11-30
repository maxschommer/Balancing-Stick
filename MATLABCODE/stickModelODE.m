function stickModelODE
clear
m = 1;
theta0 = .1; %Radians
rodLength = .3;
thetadot0 = 0;
g = 9.81;
integral=0;

zeros = [];
poles = [-1, -1];
K = .1;
sys = zpk(zeros, poles, K);
disp(sys)
[C_pid,info] = pidtune(sys,'PID');
P = C_pid.kp;
I = C_pid.ki;
D = C_pid.kd;
disp(P)

dt = .001;

tEnd = 10;
t=0:dt:tEnd;   % time scale

% options = odeset('Events', @springEvents);

[t1, x1] = ode45(@phase1, t, [theta0 thetadot0]);

hold on;
%Plot phase 1
plot(t1,x1(:,1)); %Plot mass

figure
posx = rodLength*sin(x1(:,1));
posy = rodLength*cos(x1(:,1));

axisLength =rodLength*1.25;

axis([-axisLength, axisLength, -axisLength, axisLength])
pbaspect([1,1,1])
% axis(ax, 'square')
% comet(posx, posy)

h = animatedline('Marker', 'o');
b = animatedline;


for k = 1:10:length(posx)
    addpoints(h,posx(k),posy(k));
    addpoints(b, 0,0);
    addpoints(b, posx(k),posy(k));
    drawnow
    clearpoints(h);
    clearpoints(b);
end



function dValues=phase1(t,M)
    theta = M(1);
    thetadot = M(2);

    error =theta;
    integral = integral*.95+theta*dt;
    a = g*sin(theta) -( P*error + D*thetadot + I*integral);

    dValues=[thetadot; a];
end


end