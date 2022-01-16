%%    %%%%%%%%%
%函数功能：领导者运动方向计算
%函数输入：领导者位置，目标点位置，运动障碍位置，邻接矩阵,目标点，d_c为期望的智能体距离，d_obs为期望的智能体与障碍的距离
%函数输出：领导者运动方向

function   direction = calLeaderMotionVector(p,p_des,a_mtr,pos_set,leader_symbol,d_c,p_obstacle,d_obs,d_n,d_obsd)

    %可调参数
    k11 = 0.01;
    k12 = 0.0005;
    k2 = 3;
    k3 = 0.1;
    k4 =0;
    flock_size = size(a_mtr,2);
    delta = 0.1;
    % 目标导航势能计算及负梯度方向计算
    psi_nav = 0.5*norm((p-p_des))^2;                                       
    delta_psi_nav = -(p-p_des);                                            
    
    % 智能体碰撞势能计算及负梯度计算                                                          
    d_d = 3.5;
    px_mtr=kron(pos_set(1,:),ones(flock_size,1));
    py_mtr=kron(pos_set(2,:),ones(flock_size,1));
    px_rel=(px_mtr'-px_mtr);
    py_rel=(py_mtr'-py_mtr);   
    p_dis = sqrt(px_rel.^2+py_rel.^2);
    p_dis_x = p_dis-d_d;
    p_dis_x_1 = p_dis_x./(d_c-d_d)+delta;
    p_dis_x_des = ((1./(p_dis_x_1.^2)).*((1./p_dis_x_1)-1/(1+delta)))./p_dis;
    px_rel = px_rel.*p_dis_x_des;
    py_rel = py_rel.*p_dis_x_des;
    px_rel(p_dis_x_1>=1+delta) = 0;
    py_rel(p_dis_x_1>=1+delta) = 0;
%     p_dis = d_c-p_dis;
%     p_dis(p_dis<0) = 0;
    px_rel(logical(eye(size(px_rel)))) =0; 
    py_rel(logical(eye(size(py_rel)))) =0; 
    px_sum=sum(px_rel,2);
    py_sum=sum(py_rel,2);
    delta_psi_coll = [px_sum(leader_symbol);py_sum(leader_symbol)];
    
    
     %智能体凝聚势能计算及负梯度计算
     d_d = 3.5
     theta1 = asin(d_d/d_n);
     s = find(a_mtr(leader_symbol,:)>0);
     for i = 1:size(s,2)
         r = norm(pos_set(:,leader_symbol)-pos_set(:,s(i)));
         for j =i+1:size(s,2)
             if norm(pos_set(:,s(j))-pos_set(:,leader_symbol))<r
                 p1 = pos_set(:,s(j))-pos_set(:,leader_symbol);
                 p2 = pos_set(:,s(i))-pos_set(:,leader_symbol);
                 theta = acos(dot(p1,p2)/(norm(p1)*norm(p2)));
                 if theta>0&&theta<theta1
                     a_mtr(leader_symbol,s(i)) = 0;
                     break;
                 end
             end
         end
     end
    
    px_mtr_1=kron(pos_set(1,:),ones(flock_size,1));
    py_mtr_1=kron(pos_set(2,:),ones(flock_size,1));
    px_rel_1=(px_mtr_1'-px_mtr_1);
    py_rel_1=(py_mtr_1'-py_mtr_1);   
    p_dis_1 = sqrt(px_rel_1.^2+py_rel_1.^2);
    px_rel_1 = px_rel_1./p_dis_1;
    py_rel_1 = py_rel_1./p_dis_1;
    p_dis_1 = d_n-p_dis_1;
    p_dis_1(p_dis_1>0) = 0;
    p_dis_1 = -3.*(p_dis_1).^4;
    px_rel_1 = px_rel_1.*p_dis_1;
    py_rel_1 = py_rel_1.*p_dis_1;
    px_rel_1(logical(eye(size(px_rel_1)))) =0; 
    py_rel_1(logical(eye(size(py_rel_1)))) =0; 
    px_rel_1 = px_rel_1.*a_mtr;
    py_rel_1 = py_rel_1.*a_mtr;
    px_sum_1=sum(px_rel_1,2);
    py_sum_1=sum(py_rel_1,2);
    delta_psi_coll_1 = [px_sum_1(leader_symbol);py_sum_1(leader_symbol)];
    
    
    % 智能体障碍势能计算及负梯度计算
        temp = [0;0];
    for k = 1:size(p_obstacle,2)
        p_l_o = norm(p-p_obstacle(:,k));
        if(p_l_o>=d_obs)
            delta_psi_obs = 0;
            temp =temp+delta_psi_obs;
        else
            x =(p_l_o-d_obsd)/(d_obs-d_obsd)+delta;
            x_1 = (1/(x^2))*(1/x-1/(1+delta))/p_l_o;
            delta_psi_obs = x_1*(p-p_obstacle(:,k));    
            temp =temp+delta_psi_obs;
        end
    end
    if sum(temp)~=0
        k4 =0;
        
    end
    
    
    delta_psi_nav(1) = k11*delta_psi_nav(1);
    delta_psi_nav(2)  = k11*delta_psi_nav(2);
    gra_psi_leader =  1*delta_psi_nav+k2*delta_psi_coll+k3*temp+k4* delta_psi_coll_1;
    direction = gra_psi_leader;
end









