%%    %%%%%%%%%%
% 函数功能： 考虑到跟随与被跟随智能体，限制智能体的最大输入速度
% 函数输入： cons_agent为限制该智能体速度大小的智能体,d_s为通信距离,p为智能体位置,u为计算的运动方向
% 函数输出： 速度最大限制
function   mag =  calVectorMag(p,cons_agent,d_s,u)
    
     temp = norm(u);
    count = size(cons_agent,2);
    if count ~=0
        p = kron(ones(1,count),p);
        s_ji = cons_agent - p;
        p = s_ji;
        s_ji = s_ji'*u;
        s_ji(s_ji<=0) = 0;                                                     %元素为0表示运动输入u远离该智能体
        s_ji(s_ji>0) = 1;                                                      %元素为1表示运动输入u接近智能体

        for i=1:count
            if (s_ji(i)>0)
                m = u'*p(:,i)/norm(u);
                temp = min(temp,m);
            else
             m = p(1,i).^2+p(2,i).^2;
             m = 0.5*(d_s-sqrt(m));
             temp = min(temp,m);
            end
        end
    end
    mag = temp;
end