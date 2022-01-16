% 函数功能:计算避障控制力
% 输入变量:本体位置,本体速度,beta智能体位置集,beta智能体速度位置集,
%          beta智能体邻接矩阵,参数数组
% 输出变量:避障控制力
function V_avoid=AvoidV(Pa_set,Po_set,Ao_set)
% 参数设定
% r=param(1);                                                                % 邻居作用半径
% e=param(2);                                                                % alpha范数调节参数
% h=param(3);                                                                % 凸函数调节参数
% d=param(4);                                                                % 虚拟作用距离
% c1=param(5);                                                               % 梯度项调节参数c1
% c2=param(6);                                                               % 协调项调节参数c2
% 变量初始化
% m=length(Pa_set(1,:));                                                     % 智能体个数
% n=length(Po_set(1,:));                                                     % 邻居智能体个数
% F_avoid=zeros(3,m);                                                        % 避障控制力
% avod=zeros(3,m);                                                           % 障碍规避项
% cons=zeros(3,m);                                                           % 速度协调项
% 计算避障控制力
% for i=1:m
%     for j=1:n
%         计算障碍规避项
%         temp=Po_set(:,j)-Pa_set(:,i);
%         dist=sqrt(temp'*temp);
%         z=alpha_norm(dist,e);
%         field=avoidance(z,r,e,h,d)*gradnorm(Po_set(:,j)-Pa_set(:,i),e);
%         avod(:,i)=avod(:,i)+field;
%         计算速度协调项
%         cons(:,i)=cons(:,i)+(Vo_set(:,j)-Va_set(:,i))*Ao_set(i,j);    
%     end
%     F_avoid(:,i)=c1*avod(:,i)+c2*cons(:,i);
% end
    V_avoid = zeros(2,1);
    len = size(Po_set,2);
    for i=1:len
        V_avoid = V_avoid + Ao_set(1,i)*(Po_set(:,i)-Pa_set);
    end


end

%% 避障函数库

% 函数功能:alpha范数
% 输入变量:计算变量,可调参数
% 输出变量:计算结果
function y=alpha_norm(z,e)
y=(sqrt(1+e.*(z'*z))-1)./e;
end

% 函数功能:alpha范数梯度
% 输入变量:计算变量,可调参数
% 输出变量:计算结果
function y=gradnorm(z,e)
y=z./sqrt(1+e.*(z'*z));
end

% 函数功能:凸函数
% 输入变量:计算变量,可调参数
% 输出变量:计算结果
function y=bump(z,h)
y=0.5*(1+cos(pi*(z-h)/(1-h)));
if z>=0 && z<h
    y=1;
end
if z<0 || z>1
    y=0;
end
end

% 函数功能:规避函数
% 输入变量:计算变量,可调参数e,可调参数h,虚拟作用范围d
% 输出变量:计算结果
function y=avoidance(z,r,e,h,d)
y1=bump(z/alpha_norm(r,e),h);
y2=gradnorm(z-alpha_norm(d,e),1)-1;
y=y1*y2;
end

