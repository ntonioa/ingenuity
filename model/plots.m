% syms t psi(t) theta(t) phi(t) real 
% 
% R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
%          sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
%          -sin(theta),       cos(theta)*sin(phi),                     cos(theta)*cos(phi)];
% 
% diff(R,t)

%% 3D plot
figure(1)
plot3(out.P_des.Data(1,:),out.P_des.Data(2,:),out.P_des.Data(3,:))
hold on
plot3(out.P.Data(:,1),out.P.Data(:,2),out.P.Data(:,3))
legend("desired trajectory","actual trajectory")
%% desired and real trajectory comparison
figure(2)
t=out.P.Time;
subplot(3,1,1)
plot(t,out.P_des.Data(1,:))
hold on
plot(t,out.P.Data(:,1))
subplot(3,1,2)
plot(t,out.P_des.Data(2,:))
hold on
plot(t,out.P.Data(:,2))
subplot(3,1,3)
plot(t,out.P_des.Data(3,:))
hold on
plot(t,out.P.Data(:,3))

figure(3)
subplot(3,1,1)
plot(t,out.P_dot_des.Data(1,:))
hold on
plot(t,out.P_dot.Data(1,:))
subplot(3,1,2)
plot(t,out.P_dot_des.Data(2,:))
hold on
plot(t,out.P_dot.Data(2,:))
subplot(3,1,3)
plot(t,out.P_dot_des.Data(3,:))
hold on
plot(t,out.P_dot.Data(3,:))

%% attitude
figure(4)
t=out.P.Time;
subplot(3,1,1)
plot(t,out.attitude.Data(:,1))
legend("roll")
subplot(3,1,2)
plot(t,out.attitude.Data(:,2))
legend("pitch")
subplot(3,1,3)
plot(t,out.attitude.Data(:,3))
legend("yaw")

%% control input
figure(5)
t=out.P.Time;
subplot(3,1,1)
plot(t,out.u.Data(1,:))
subplot(3,1,2)
plot(t,out.u.Data(2,:))
subplot(3,1,3)
plot(t,out.u.Data(3,:))


