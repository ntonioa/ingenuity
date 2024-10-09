% syms t psi(t) theta(t) phi(t) real 
% 
% R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
%          sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
%          -sin(theta),       cos(theta)*sin(phi),                     cos(theta)*cos(phi)];
% 
% diff(R,t)

end_index=find(out.P.Data(:,1)>50 | out.P.Data(:,1)<-50);
t=out.P.Time;

plot3(out.P.Data(:,1),out.P.Data(:,2),out.P.Data(:,3))
hold on
% plot3(10*cos(t/50),10*sin(t/50),1+t/50)
grid on
