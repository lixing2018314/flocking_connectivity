%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数功能:解耦式的群中心控制，估计群中心跟随参考轨迹运动
% 输入变量:T-控制周期,a_mtr-邻接矩阵,pos_set-位置集合,vel_set-速度集合,
%          cen_set-群中心集合
% 输出变量:cens_set-群中心估计集合
function [ps_cen,vs_cen]=GroupCenterControl(flock_size,a_mtr,pos_cen,v_set,p_ref,v_ref,informed_robots)
    % 比例参数设定
    k_1=0.05;
    k_3 = 0.05;
    k_4 = 0.1;
    % 重塑邻接矩阵
    a_mtr(a_mtr>0)=1;
    % 计算位置估计误差量
    px_mtr=kron(pos_cen(1,:),ones(flock_size,1));
    py_mtr=kron(pos_cen(2,:),ones(flock_size,1));
    px_rel=(px_mtr'-px_mtr).*a_mtr;
    py_rel=(py_mtr'-py_mtr).*a_mtr;    
    px_sum=sum(px_rel,1);
    py_sum=sum(py_rel,1);    
    % 计算速度估计误差量
%     vx_mtr=kron(vel_cen(1,:),ones(flock_size,1));
%     vy_mtr=kron(vel_cen(2,:),ones(flock_size,1));
%     vx_rel=(vx_mtr'-vx_mtr).*a_mtr;
%     vy_rel=(vy_mtr'-vy_mtr).*a_mtr;
%     vx_sum=sum(vx_rel,1);
%     vy_sum=sum(vy_rel,1);
    % 计算群中心估计状态    
%     us_cen=cmd_set-(k_1.*[px_sum;py_sum]+k_2.*[vx_sum;vy_sum]);
%     ps_cen=pos_cen+vel_cen.*T+us_cen.*(T*T/2); 
%     vs_cen=vel_cen+us_cen.*T; 
%        v= vel_cen*a_mtr';
%        count = sum(a_mtr,2);
%        count = [count';count'];
%  
%        v =v.count;
      vs_cen = k_1.*[px_sum;py_sum]+v_set;
      
     %informed robots  add navigation
%       for i =1:flock_size
%           if informed_robots(i) == 1
%               vs_cen(:,i) =vs_cen(:,i)+k_3*(p_ref(:,i)-pos_cen(:,i))+(v_ref(:,i));
%           end
%       end
      ps_cen = pos_cen+vs_cen;

end