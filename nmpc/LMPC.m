%================无人机模型预测控制================%
clear all;clc;close all;
%% 无人机参数设定--采用运动学模型进行轨迹跟踪
x0 = 3; y0 = 3;
vx0 = 0; vy0 = 0;
x(1) = x0; y(1) = y0;vx(1) = vx0;vy(1) = vy0;
%% 领航者参数设定
inter = 0.05;  % 采样周期
time = 60;  % 总时长
R = 2;
omega = 2;
t = 0:inter:time;
for i = 1:1:length(t)
   if (mod(floor(omega*t(i)/(2*pi)),2) == 0)
    Xr(i) = R*cos(omega*t(i))-R;
    Yr(i) = R*sin(omega*t(i));
    Vxr(i) = -R*sin(omega*t(i))*omega;
    Vyr(i) = R*cos(omega*t(i))*omega;
    Uxr(i) = -R*cos(omega*t(i))*omega^2;
    Uyr(i) = -R*sin(omega*t(i))*omega^2;
   else
    Xr(i) = -R*cos(omega*t(i))+R;
    Yr(i) = R*sin(omega*t(i));   
    Vxr(i) = R*sin(omega*t(i))*omega;
    Vyr(i) = R*cos(omega*t(i))*omega;
    Uxr(i) = R*cos(omega*t(i))*omega^2;
    Uyr(i) = -R*sin(omega*t(i))*omega^2;
   end

end
% Xr = -R*cos(t);
% Yr = R*sin(t);
% Vxr = R*sin(t);
% Vyr = R*cos(t);
% Uxr = R*cos(t);
% Uyr = -R*sin(t);
EX(:,1) = [x0 - Xr(1);vx0 - Vxr(1);y0 - Yr(1);vy0 - Vyr(1)];
X(:,1) = [x0;vx0;y0;vy0];
% figure
% grid minor
% l1 = [];
% axis([-7 7 -7 7]);
% axis equal
% for i = 2:1:length(t)
%   hold on
%   plot([Xr(i) Xr(i-1)],[Yr(i) Yr(i-1)],'b');
%   hold on
%   delete(l1);
%   l1 =  plot(Xr(i),Yr(i),'r.','MarkerSize',20);
%   pause(0.1);
%   
% end
%% 模型预测控制参数设定
Np = 30;     % 预测步长
Nc = 5;      % 控制步长
A = [0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];  B = [0 0;1 0;0 0;0 1]; 
lena = size(A);
lenb = size(B);
R = eye(Nc*lenb(2));
Ak = A*inter + eye(lena(1));
Bk = B*inter;
F = cell(Np,1);
PHI = cell(Np,Nc);
for i = 1:1:Np         % 计算预测方程矩阵
  F{i,1} = Ak^i;
end
F = cell2mat(F);

for i = 1:1:Np
   for j = 1:1:Nc
       if (j<=i)
           PHI{i,j} = Ak^(i-j)*Bk;
       else
           PHI{i,j} = zeros(lena(1),lenb(2));
       end
   end
end
PHI = cell2mat(PHI);
k1 =2;k2 =2;
%% 迭代计算
for i = 1:1:length(t)-1
 
  U = -inv(PHI'*PHI + R)*PHI'*F*EX(:,i);
  u = U(1:2,1) + [Uxr(i);Uyr(i)];
  EX(:,i+1) = Ak*EX(:,i) + Bk*U(1:2,1);
  X(:,i+1) = [Xr(i+1);Vxr(i+1);Yr(i+1);Vyr(i+1)]+EX(:,i+1);

end
x = (X(1,:))';
vx = (X(2,:))';
y = (X(3,:))';
vy = (X(4,:))';
% VV = vecnorm([Vxr;Vyr]);
% VX = vecnorm([vx;vy]);
% plot(t,VV,'r')
% hold on
% plot(t,VX(1:length(t)),'b')


l1 = [];
l2 = [];
figure
 grid minor
 axis([-5 5 -5 5])
axis equal
Tag1 = animatedline('Color','r');
for i = 1:1:length(Xr)-1
    
    hold on
    delete(l1);
   delete(l2);

    plot([x(i) x(i+1)],[y(i) y(i+1)],'r');
   hold on
   plot([Xr(i) Xr(i+1)],[Yr(i) Yr(i+1)],'b');
   hold on
   l1 = plot(x(i+1),y(i+1),'r.','MarkerSize',20);
   hold on
   l2 = plot(Xr(i+1),Yr(i+1),'b.','MarkerSize',20);
%    pause(0.1);
%    addpoints(Tag1,t(i),x(i));
   drawnow;
end

