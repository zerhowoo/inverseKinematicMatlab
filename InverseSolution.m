global L;
global kp;
b=320;
polt_flag=0;
MaxAng=30;
iteAng=0.1;
L=b*[1 1 1 1 1 1] ;
[kp,gap,kpp]=KpCirecleQ1(400);  
TargetNum=size(kp,1);
LinkNum=size(L,2);

a=zeros(LinkNum+1,3,TargetNum);  % 初始化所有关节的三维坐标数组 ()
ini_a=zeros(1,LinkNum+1);        % 设置机械臂初始姿态 (初始也可以不是拉直的)

ini_a(LinkNum+1)=0;         
a(LinkNum+1,2,1)= 0;
for i=LinkNum:-1:1
      ini_a(i)=ini_a(i+1)-L(i);
      a(i,2,1)= ini_a(i);
end

%% 逆运动学求解 
% 逐点求解机械臂配置，基于前一时刻配置求解下一时刻  237开始超 30°
tic
for i=1:TargetNum-1
a(:,:,i+1)=SloveInverseArray(a(:,:,i),kp(i+1,:),kpp(i+1,:),LinkNum,MaxAng,iteAng);
end
runtime=toc;

%% 每个关节的坐标 转化成 角度
% CurAng=zeros(LinkNum+1,2,300);
% for i=1:TargetNum
%   CurAng(:,:,i)=SloveAngles(a(:,:,i),LinkNum);
% 
% 
%  if polt_flag
%   a_solved= LinkCorPlot(CurAng(:,:,i));
%   pause(0.005)
%  end
% end