%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数功能:群中心估计
% 输入变量:T-控制周期,a_mtr-邻接矩阵,pos_set-位置集合,vel_set-速度集合,
%          cen_set-群中心集合
% 输出变量:cens_set-群中心估计集合
function [ps_cen,vs_cen]=EstimateGroupCenter(T,flock_size,a_mtr,pos_cen,vel_cen,v_set,p_set)
    % 比例参数设定
    k_1=0.1;
    k_2=2.5;
    % 重塑邻接矩阵
    a_mtr(a_mtr>0)=1;
    % 计算位置估计误差量
    px_mtr=kron(pos_cen(1,:),ones(flock_size,1));
    py_mtr=kron(pos_cen(2,:),ones(flock_size,1));
    px_rel=(px_mtr'-px_mtr).*a_mtr;
    py_rel=(py_mtr'-py_mtr).*a_mtr;
    px_sum=sum(px_rel,2)';
    py_sum=sum(py_rel,2)';    
    % 计算速度估计误差量
    vx_mtr=kron(vel_cen(1,:),ones(flock_size,1));
    vy_mtr=kron(vel_cen(2,:),ones(flock_size,1));
    vx_rel=(vx_mtr'-vx_mtr).*a_mtr;
    vy_rel=(vy_mtr'-vy_mtr).*a_mtr;
    vx_sum=sum(vx_rel,2)';
    vy_sum=sum(vy_rel,2)';
    % 计算群中心估计状态    
    v= vel_cen*a_mtr';
    count = sum(a_mtr,2);
    count = [count';count'];
    v =v./count;
    vs_cen = -k_1.*[px_sum;py_sum]+v_set;
    ps_cen = pos_cen+vs_cen;
   % ps_cen(:,flock_size) = p_set(:,flock_size);
end