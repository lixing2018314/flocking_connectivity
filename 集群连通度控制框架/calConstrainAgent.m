%%    %%%%%%%%%
%函数功能：计算跟随智能体以及其他跟随的智能体
%函数输入：领导者位置，目标点位置，运动障碍位置，邻接矩阵,目标点，d_c为期望的智能体距离，d_obs为期望的智能体与障碍的距离
%函数输出：跟随智能体以及其他跟随的智能体

function  [follow_agent,other_agent] = calConstrainAgent(hop_mtr,p_set,a_mtr,zero_pos)
    flock_size = size(a_mtr,2);
    follow_agent =zeros(flock_size,flock_size);
    other_agent = zeros(flock_size,flock_size);
    
    for i = 1:flock_size
        other_agent_single = [];
        dis = 1000;
        dis1 = 1000;
        dis2 = 1000;
        hop_copy = hop_mtr;
        hop_copy = hop_copy.*a_mtr(i,:);
        hop_copy(hop_copy==0) = 2100000;
        if(a_mtr(i,zero_pos)==1)
            hop_copy(zero_pos) =0;
        end
        s = find(hop_copy<hop_mtr(i));
        sp = find(hop_copy>hop_mtr(i));
        sp1 = find(hop_copy~=2100000);
        sp = intersect(sp,sp1);
        se = find(hop_copy==hop_mtr(i));
        sn = find(hop_copy<=hop_mtr(i));
        s1 = find(a_mtr(i,:)>0);
        if size(s,2) > 0
            %计算要跟随的智能体
            for j=1:size(s,2)
                if dis>norm(p_set(:,i)-p_set(:,s(1,j)))
                    near_agent_num = s(1,j);
                end
            end
            dis2 = 1000;
            for j=1:size(sn,2)
                if norm(p_set(:,i)-p_set(:,near_agent_num))>norm(p_set(:,sn(1,j))-p_set(:,near_agent_num))
                    if dis2>norm(p_set(:,i)-p_set(:,sn(1,j)))
                        dis2 = norm(p_set(:,i)-p_set(:,sn(1,j)));
                        follow_agent_num = sn(1,j);
                    end
                end
            end
            
            %计算其他智能体跟随自身的
             for j=1:size(s1,2)
                 dis1 = 1000;
                 if hop_mtr(s1(1,j))<hop_mtr(i)
                     continue;
                 end
                 ss = find(a_mtr(s1(1,j),:)>0);
                 s2 =intersect(s1,ss);
                 s2 = [s2,i];        %在当前智能体下的拓扑结构其他智能体的邻居
                 %寻找跳数小的，如果有的话
                 s3 = find(hop_mtr<hop_mtr(s1(1,j)));
                 s4 = intersect(s2,s3);
                 if size(s4,2)>0
                   for l = 1:size(s4,2)
                       if dis1>norm(p_set(:,s1(1,j))-p_set(:,s4(1,l)))
                           dis1=norm(p_set(:,s1(1,j))-p_set(:,s4(1,l)));
                           near_agent_num = s4(1,l);
                       end
                   end
                    dis2 = 1000;
                   for m=1:size(s2,2)
                       if norm(p_set(:,s1(1,j))-p_set(:,near_agent_num))>norm(p_set(:,s2(1,m))-p_set(:,near_agent_num))
                           if dis2>norm(p_set(:,s1(1,j))-p_set(:,s2(1,m)))
                               dis2 = norm(p_set(:,s1(1,j))-p_set(:,s2(1,m)));
                               other_agent_num = s2(1,m);
                           end
                       end
                   end
                   if other_agent_num == i
                       other_agent_single = [other_agent_single,s1(1,j)]; 
                   end
                 else
                     dis4 =1000;
                     for q = 1:size(s2,2)
                         if dis4>norm(p_set(:,s1(1,j))-p_set(:,s2(1,q)))&&hop_mtr(s1(1,j))==hop_mtr(s2(1,q))
                            dis4=norm(p_set(:,s1(1,j))-p_set(:,s2(1,q)));
                            other_agent_num = s2(1,q);
                         end
                     end
                     if other_agent_num == i
                         other_agent_single = [other_agent_single,s1(1,j)];
                     end   
                 end
                 
             end

            
        else
            %计算要跟随的智能体
            dis1 = 1000;
            for k = 1:size(sn,2)
                if dis1 > norm(p_set(:,i)-p_set(:,sn(1,k)))&&hop_mtr(i)==hop_mtr(sn(1,k))
                    dis1 =  norm(p_set(:,i)-p_set(:,sn(1,k)));
                     follow_agent_num = sn(1,k);
                end
            end
            
            %计算其他智能体要跟随自身的,跳数相等的全部加入其他智能体，跳数大于的则要根据规则加入
            for j = 1:size(se,2)
                if se(1,j)~=i
                 other_agent_single = [other_agent_single,se(1,j)];
                end
            end
           
            for j=1:size(sp,2)
                 dis1 = 1000;
                 dis2 =1000;
                 ss = find(a_mtr(sp(1,j),:)>0);
                 s2 =intersect(s1,ss);
                 s2 = [s2,i];
                 %寻找跳数小的，如果有的话
                 s3 = find(hop_mtr<hop_mtr(sp(1,j)));
                 s4 = intersect(s2,s3);
                   for l = 1:size(s4,2)
                       if dis1>norm(p_set(:,s1(1,j))-p_set(:,s4(1,l)))
                           dis1=norm(p_set(:,s1(1,j))-p_set(:,s4(1,l)));
                           near_agent_num = s4(1,l);
                       end
                   end
                   for m=1:size(s2,2)
                       if norm(p_set(:,sp(1,j))-p_set(:,near_agent_num))>norm(p_set(:,s2(1,m))-p_set(:,near_agent_num))
                           if dis2>norm(p_set(:,sp(1,j))-p_set(:,s2(1,m)))
                               dis2 = norm(p_set(:,sp(1,j))-p_set(:,s2(1,m)));
                               other_agent_num = s2(1,m);
                           end
                       end
                   end
                   if other_agent_num == i
                       other_agent_single = [other_agent_single,sp(1,j)]; 
                   end
                 
            end
            
            
        end
        if i~=zero_pos
           follow_agent(i,follow_agent_num) = 1;
       end
        for k=1:size(other_agent_single,2)
            other_agent(i,other_agent_single(1,k)) = 1;
        end
     
    end
    
    
end