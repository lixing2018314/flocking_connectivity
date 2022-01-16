
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数功能：计算每个智能体跳数，修改后的跳数更新函数
% 函数输入：a_mtr为邻接矩阵，estimate_pcen为估计的群中心矩阵,Flock_hop为智能体跳数矩阵,R_com为通信半径,Flock_pos为智能体当前位置矩阵

% 函数输出：更新的智能体跳数矩阵HopMtr

% 
function [HopMtr] = UpdateHop(a_mtr,estimate_pcen,Flock_hop,R_com,Flock_pos,zero_pos)

    [n,~]= size(a_mtr);
    Flock_size = n;
    HopMtr = Flock_hop;
    %导航机器人无需更新跳数，其跳数为0
    for i =1:Flock_size
        min = Flock_hop(i);
        for j=1:Flock_size
            if a_mtr(i,j)>0
                if min>=Flock_hop(j)
                    min = Flock_hop(j);
                end
                
            end
        end
  
        if  Flock_hop(i)>=min&&min~=2000
            HopMtr(i) = min+1;
        end
    end
    HopMtr(zero_pos) = 0;
end



% 最初的跳数更新函数，对应中心法
% function [HopMtr] = UpdateHop(a_mtr,estimate_pcen,Flock_hop,R_com,Flock_pos)
% 
%     [n,~]= size(a_mtr);
%     Flock_size = n;
%     HopMtr = Flock_hop;
%     for i =1:Flock_size
%         if norm(Flock_pos(:,i)-estimate_pcen(:,i))<R_com
%             HopMtr(i) = 1;
%         else
%             min = Flock_hop(i);
%             for j=1:Flock_size
%                 if a_mtr(i,j)>0
%                     
%                     if min>=Flock_hop(j)
%                         min = Flock_hop(j);
%                     end
%                     
%                 end
%             end
%             if  Flock_hop(i)>=min
%                 HopMtr(i) = min+1;
%             end
%         end
%     end
% end