 %% 轨迹生成器
 
function [KP,gap,KPP]=KpCirecleQ1(step)    %% 从正方形中间开始
a=150; % 半径a=200单位
KPP=zeros(step+1,3);
KpCircle=zeros(step+1,3);
for i=0:1:step/2
   KpCircle(i+1,:) = [0,a*sin(pi/2*i/(step/2)),a-a*cos(pi/2*i/(step/2))];%296---400
   KPP(i+1,:) = [0,1,0];
end

% 生成1/4圆弧轨迹（0到90度）
for i=step/2+1:1:step
   KpCircle(i+1,:) = [0,2*a-a*sin(pi/2*i/(step/2)),a-a*cos(pi/2*i/(step/2))];
   KPP(i+1,:) = [0,1,0];
end


KP=KpCircle;
% 401~601

for d1=i+1:i+25
    KP(d1+1,:) = [KP(d1,1)-3 KP(d1,2) KP(d1,3)];
    KPP(d1+1,:) = [0,1,0];
end
for a=d1+1:d1+50
    KP(a+1,:) = [KP(a,1) KP(a,2) KP(a,3)-3];
    KPP(a+1,:) = [0,1,0];
end
for b=a+1:a+50
    KP(b+1,:) = [KP(b,1)+3 KP(b,2) KP(b,3)];
    KPP(b+1,:) = [0,1,0];
end
for c=b+1:b+50
    KP(c+1,:) = [KP(c,1) KP(c,2) KP(c,3)+3];
    KPP(c+1,:) = [0,1,0];
end
for d=c+1:c+25
    KP(d+1,:) = [KP(d,1)-3 KP(d,2) KP(d,3)];
    KPP(d+1,:) = [0,1,0];
end

KP(i+201:i+201+step,:) = flipud(KpCircle);
KPP(i+201:i+201+step,:) = KPP(1:step+1,:);
TargetNum = size(KP,1);
for i=2:TargetNum-1
  tar3(i-1)=norm( KP(i,:) - KP(i-1,:) )^2;
end
for i=1:TargetNum-1
  gap(i)=norm( KP(i+1,:) - KP(i,:) )^2;
end