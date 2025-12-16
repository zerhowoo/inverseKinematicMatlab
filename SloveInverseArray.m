function a =SloveInverseArray(FormerArray,NextPoint,kpp,LinkNum,MaxAng,iteAng)
global L;
 a=zeros(LinkNum+1,3);
 a(LinkNum,:)= NextPoint;  % 第6关节(腕关节)位置，由外部控制
 a(LinkNum+1,:)= NextPoint+L(LinkNum)*kpp;  % 第7点(末端执行器)，腕关节+方向向量×杆长

 Index_compared=3;    
 uFormerArray=flipud(FormerArray);  % 翻转数组：从末端到基座的顺序
 uFormerArray=[uFormerArray;[0,uFormerArray(LinkNum+1,2)-L(1)],0]; % 添加虚拟基座点

flag=1;
for n=LinkNum:-1:2  % 从第6关节向第1关节反向求解
% 从当前关节位置 a(n,:) 出发，沿着前一时刻的关节轨迹（翻转后的 uFormerArray）寻找距离恰好为连杆长度 L(n-1) 的点，作为下一个关节的候选位置。
% 第一次调用：找候选点
[a(n-1,:),Index_compared]= FindPoint_PA(a(n,:),Index_compared,L(n-1),uFormerArray);
     % 含义：计算第 n 个关节处两根连杆的夹角。如果夹角 > MaxAng（最大允许角度，如30°），则需要修正。
    if acosd(dot(a(n+1,:)-a(n,:),a(n,:)-a(n-1,:))/(L(n)*L(n-1)))>MaxAng
        % 超限时的两种处理策略（由 flag 控制）
        % 策略A（flag = 1 刚开始超限或上一个关节未超限）
        % 做法：将下一关节位置强制旋转到恰好满足最大角度约束的位置。使用 Rodrigues 公式绕法向量 k 旋转 180-MaxAng 度。
        if flag 
            k=cross(a(n+1,:)-a(n,:),a(n-1,:)-a(n,:)); % 计算两杆的法向量(旋转轴)
            a(n-1,:)=Rodrigues(a(n+1,:)-a(n,:),k,180-MaxAng)+a(n,:);  % 用罗德里格斯公式旋转
            flag=1;
        % 策略B: flag = 0（连续超限，需要更激进调整）
        % 做法： 1. 从 a(n+1,:) 出发，寻找距离为 2*L(n)*cos((MaxAng-iteAng)/2) 的点作为 a(n-1,:)
        %        2. 然后用几何方法（等腰三角形）重新计算中间关节 a(n,:) 的位置
        %        3. 这种方法同时调整两个关节，更激进
        else
            % 重新寻找：跳过当前关节，直接找更远的点
            [a(n-1,:),Index_compared]=FindPoint_PA(a(n+1,:),Index_compared,L(n)*cosd((MaxAng-iteAng)/2)*2,uFormerArray);
            % 计算新的中间关节位置
            slope = cross(cross(a(n,:)-a(n+1,:),a(n-1,:)-a(n+1,:)),a(n-1,:)-a(n+1,:));
            a(n,:) = (a(n+1,:)+a(n-1,:))/2 - slope/norm(slope)*L(n)*sind((MaxAng-iteAng)/2);  
            flag=0;  
        end
    % 未超限时的处理
    else
        if n<LinkNum 
            % 含义：如果当前关节角度正常，保持 FindPoint_PA 找到的位置不变。但设置 flag=0，
            %       表示后续如果再遇到超限，应该使用策略B（因为链条中间已经有正常关节了）。
            flag=0; % 标记：后续再遇到超限时使用 策略B
        end
    end

end