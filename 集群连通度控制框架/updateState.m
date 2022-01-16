%% %%%%%%%%%%%%%%%%%%%
% 函数功能：智能体状态更新
% 函数输入：T-控制周期,a_mtr-邻接矩阵,pos_set-位置集合,vel_set-速度集合,
% 函数输出：位置矩阵以及速度矩阵


function [u,P_set] = updateState(pos_set,follow_agent,other_agent,target_pos,obstacle,a_mtr,u,leader_set,e_cen,p_ref_period,v_ref_peroid,informed_robots)

    flock_size = size(pos_set,2);
    V_set = zeros(2,flock_size);                                           %没用的变量
    %计算每个智能体的运动方向
    direction =zeros(2,flock_size);
    distance_config;                                                       %距离参数配置
    %计算跳数为0的智能体的运动方向
    [Pos_v,~]=VirtualAgentSet(pos_set(:,leader_set(end)),V_set(:,leader_set(end)),obstacle);% 获取虚拟智能体
    direction(:,leader_set(end)) = calLeaderMotionVector(pos_set(:,leader_set(end)),target_pos,a_mtr,pos_set,flock_size,d_c,Pos_v,d_obs,d_n,d_obsd);
    obs_symbol = zeros(1,flock_size);                                      %标记是否能够监测到障碍
    temp =zeros(2,flock_size);
    %计算其他智能体的运动方向
    for i=1:flock_size
        if i~=leader_set
        [Pos_v,~]=VirtualAgentSet(pos_set(:,i),V_set(:,i),obstacle);% 获取虚拟智能体
        follower_symbol = find(follow_agent(i,:)>0);
        [direction(:,i),obs_symbol(i),temp(:,i)] = calFollowerMotionVector(pos_set(:,i),pos_set(:,follower_symbol),a_mtr,pos_set,i,d_c,Pos_v,d_obs,d_n,d_conn,target_pos,u,d_obsd,e_cen(:,i)); 
        end
    end
    % 根据跟随智能体与被跟随智能体限制智能体的输入大小
%     
%     for i =1:flock_size
%         if informed_robots(i) == 1
%             direction(:,i) =direction(:,i)+0.1*(p_ref_period(:,i)-e_cen(:,i))+(v_ref_peroid(:,i));
%         end
%     end
    mag = zeros(1,flock_size);
    for i=1:flock_size
%         if obs_symbol(i)==1||i==leader_set(end)                            
         s = find(follow_agent(i,:));
         s1 = find(other_agent(i,:));
         s2 = union(s,s1);
         cons_agent = zeros(2,size(s2,2));
         for j = 1:size(s2,2)
             cons_agent(:,j) = pos_set(:,s2(j));
         end
%         else
%           cons_agent = [];
%         end
         mag(i) =  calVectorMag(pos_set(:,i),cons_agent,d_s,direction(:,i));
    end
%     mag(mag>0.1) = 0.1;
    for i =1:flock_size
        if informed_robots(i) == 1
           if(mag(i)>0.07)
              mag(i)=0.07;
           end
        else
            if mag(i)>0.1
                mag(i)=0.1;
            end
        end
        
            
    end
    dis = zeros(1,flock_size);
    dis(1,:) = direction(1,:).^2+direction(2,:).^2;
    dis = sqrt(dis);
    norm_direction = zeros(2,flock_size);
    norm_direction(1,:) = direction(1,:)./dis;
    norm_direction(2,:) = direction(2,:)./dis;
    u =zeros(2,flock_size);
    u(1,:) = norm_direction(1,:).*mag;
    u(2,:) = norm_direction(2,:).*mag;
    u(isnan(u))= 0;
    P_set = pos_set+u;
end