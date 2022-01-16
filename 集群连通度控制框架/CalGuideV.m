% 函数功能:计算导引控制力
% 输入变量:本体位置,本体速度,目标位置,目标速度,参数集
% 输出变量:导引控制力
function V_guide=CalGuideV(Pa,Pt)
% 参数设定
e=1;       % alpha范数调节参数
% 变量初始化
n=length(Pa(1,:));
V_guide=zeros(2,n);
% 计算导引控制力
for i=1:n
    z=Pa(:,i)-Pt;
    V_guide(:,i)=-1.*z;
end
end

%% 函数库
% 函数功能:alpha范数梯度
% 输入变量:计算变量,可调参数
% 输出变量:计算结果
function y=gradnorm(z,e)
y=z./sqrt(1+e*(z'*z));
end