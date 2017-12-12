function stickModelODE
close all
dt = .001;
tEnd = 80;
t=0:dt:tEnd;   % time scale

m = 1;
theta0 = .1; %Radians
rodLength = .3;
thetadot0 = 0;
C0 = 0;
g = 9.81;
integral=0;
randOffset = 0; %Radians of offset that the center of mass is.

sys = tf([rodLength], [0 0 -rodLength 0 g]);

disp(sys)
C_pid = pidtune(sys,'PID');
K = 1;
P = K*C_pid.kp;
D = K*C_pid.kd;
I_w = -.00002;
P_c = -.00004;
offset = 0;
C = 0;
w = 0;
error=0;
wTracker = zeros(1,1+tEnd/dt);

x1 = ode1(@phase1, t, [theta0 thetadot0]);

plot(t,x1(:,1)); %Plot position
xlabel('Time');
ylabel('Angle');

figure
plot(t, x1(:,2));
xlabel('Time');
ylabel('ThetaDot');

figure
dy=diff(transpose(x1(:,2)))./diff(t);
plot(t(2:end),dy)

figure
posx = rodLength*sin(x1(:,1));
posy = rodLength*cos(x1(:,1));

axisLength =rodLength*1.25;

axis([-axisLength, axisLength, -axisLength, axisLength])
pbaspect([1,1,1])

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
   
    offset = offset + I_w*w + P_c*C;
    w = w + C*dt;
    wTracker(round(t/dt)+1) = w;
    error = theta+offset;
    integral = integral+(error)*dt;

    C = P*(theta+offset) + D*thetadot;
    TC = 5;
    if C > TC
        C = TC;
    end
    if C < -TC
        C = -TC;
    end
    
    sys = g*sin(theta+randOffset);
    a = sys + C/m;
    dValues=[thetadot; a];
end

figure
plot(t, wTracker);
xlabel('Time');
ylabel('Velocity of Reaction Wheel');

end