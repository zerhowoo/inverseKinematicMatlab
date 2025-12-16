%% von Vorner nach hinten ging
function [result,compared]=FindPoint_PA(point,compared,length,Array)

dis_b=[];
dis_a=norm(point-Array(compared,:));

%% 条件1：初始点已满足距离要求
% 含义：如果当前参考点距离恰好等于连杆长度（误差<0.02），直接返回该点。
if  abs(dis_a-length)<0.02  
    result=Array(compared,:);  
    return 
end

%% 条件2：沿轨迹搜索
% 含义：沿着参考轨迹逐点搜索，直到找到距离 ≥ length 的点。循环结束时，
% 目标点位于 Array(compared-1,:) 和 Array(compared,:) 之间。
while(dis_a<length)  % 当前距离小于目标长度，继续向前搜索
    if  abs(dis_a-length)<0.02
        result=Array(compared,:);
        return 
    end
    
    compared=compared+1; % 移动到下一个点

    dis_b=dis_a; 
    dis_a=norm(point-Array(compared,:));
    
    if  abs(dis_a-length)<0.02
        result=Array(compared,:);
        return 
    end
end

if  abs(dis_a-length)<0.02
    result=Array(compared,:);
    return 
end

%% 条件3：插值精确计算
dis_c = norm(Array(compared,:)-Array(compared-1,:));  % 两个参考点之间的距离
cosb = dot(point-Array(compared,:),Array(compared-1,:)-Array(compared,:))/(dis_a*dis_c);  
res = dis_a*cosb-sqrt((dis_a*cosb)^2-dis_a^2+length^2);  % 余弦定理求解
result = (Array(compared-1,:)-Array(compared,:))/dis_c*res+Array(compared,:);  

end