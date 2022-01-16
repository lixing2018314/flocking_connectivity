% 函数功能:获取beta智能体(beta智能体是抽象出来的用于障碍物避碰的虚拟智能体)
% 输入变量:智能体位置,智能体速度,障碍集合
% 输出变量:虚拟智能体位置,虚拟智能体速度
% 重要说明:障碍仅限于圆形障碍物
function [Pos,Vel]=VirtualAgentSet(Pa,Va,Obs_set)
Obs_pos=Obs_set(1:2,:);                                                      % 针对圆柱形障碍
Obs_Rad=Obs_set(3,:);
% 变量初始化
n=length(Obs_set(1,:));
Pos=zeros(2,n);
Vel=zeros(2,n);
a=zeros(2,n);
k=zeros(1,n);
L=zeros(1,n);
for i=1:n
    % 计算法向单位向量
    a(:,i)=Pa-Obs_pos(:,i);
    L(i)=norm(a(:,i));
    if L(i)<0.0001
        L(i)=0.0001
    end
    a(:,i)=a(:,i)/L(i);
    % 计算虚拟智能体位置
    k(i)=Obs_Rad(i)./L(i);
    Pos(1:2,i)=k(i).*Pa+(1-k(i)).*Obs_pos(1:2,i);
    % 计算虚拟智能体速度    
    Vel(1:2,i)=(Obs_Rad(i)/L(i)).*(eye(2)-a(:,i)*a(:,i)')*Va;
end 
end  