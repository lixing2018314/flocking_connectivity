% 函数功能:求解虚拟邻接矩阵和虚拟距离矩阵
% 输入变量:各智能体位置,各障碍智能体位置,虚拟作用距离
% 输出变量:虚拟邻居集,虚拟邻接距离
function [VsNeighSet,VsDistSet,VsAdstSet]=VirtualNeigh(Pos,Obs,Rad,e)
Na=length(Pos(1,:));                                                       % 获取智能体个数
No=length(Obs(1,:));                                                       % 获取障碍个数
VsDistSet=zeros(Na,No);                                                    % 初始化虚拟距离矩阵
VsNeighSet=zeros(Na,No);                                                   % 初始化虚拟邻接矩阵
VsAdstSet=zeros(Na,No);                                                    % 初始化虚拟邻接矩阵
for i=1:Na
    for j=1:No
        z=Obs(1:2,j)-Pos(:,i);
        VsDistSet(i,j)=sqrt(z'*z);
        if VsDistSet(i,j)<=Rad
            VsNeighSet(i,j)=1;
        end
        VsAdstSet(i,j)=-bump(norm(VsDistSet(i,j)),e,6);
    end
end
end

%% 图论函数库

% 函数功能:alpha范数
% 输入变量:计算变量,可调参数
% 输出变量:计算结果
function y=alpha_norm(z,e)
y=(sqrt(1+e*(z'*z))-1)/e;
end

% 函数功能:凸函数
% 输入变量:计算变量,可调参数
% 输出变量:计算结果
function y=bump(z,h,R_com)
if z>=h&&z<=R_com
    y=0.5*(1+cos(pi*(z-h)/(R_com-h)));
end
if z>=0 && z<h
    y=1;
end
if z>R_com
    y=0;
end
end