

%%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数功能：计算参考轨迹的参考点以及参考速度
% 输入变量：起始点pos_init，目标点pos_init,参考点数量n
% 输出变量：参考轨迹点位置以及参考轨迹点速度

function [p_ref,v_ref] = CreateReferTraj(pos_init,pos_des,n)

   tra(1,:) = linspace(pos_init(1),pos_des(1),n);
   tra(2,:) = linspace(pos_init(2),pos_des(2),n);
   v_ref = tra;
   p_ref = tra;
   v_ref(:,:)=4;
end