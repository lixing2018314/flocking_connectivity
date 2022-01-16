%%    %%%%%%%%%
%函数功能：跟随者运动方向计算
%函数输入：领导者位置，目标点位置，运动障碍位置，邻接矩阵,目标点，d_c为期望的智能体距离，d_obs为期望的智能体与障碍的距离,follow_symbol为本体的标号
%函数输出：跟随者运动方向

function   [direction,symbol,temp1] = calFollowerMotionVector(p,p_follow,a_mtr,pos_set,follower_symbol,d_c,p_obstacle,d_obs,d_n,d_conn,p_des,u,d_obsd,e_cen)

    flock_size = size(a_mtr,2);
    %可调参数
    k1 = 3;
    k2 = 0.5;
    k3 = 0.2;
   delta = 0.1;
    % 跟随智能体势能计算及负梯度方向计算
    psi_follow = 0.5*norm((p_follow-p))^2;  
    if norm(p_follow-p)>d_conn
        delta_psi_follow = (norm(p_follow-p)-d_conn)*(p_follow-p)/norm(p_follow-p);  
    else
        delta_psi_follow = [0;0];
    end
   
    %mean_shift计算,计算周围邻居的中心位置
    delta_mean_shift = e_cen-p;
    local_x_cen=a_mtr(follower_symbol,:)*pos_set(1,:)';
    local_y_cen= a_mtr(follower_symbol,:)*pos_set(2,:)';
    num=sum(a_mtr(follower_symbol,:));
    local_x_cen=(local_x_cen)/(num);
    local_y_cen=(local_y_cen)/(num);
    p_io=norm([local_x_cen;local_y_cen]-p);
    d = 4;
    delta_mean_shift = ((p_io-d)/p_io)*([local_x_cen;local_y_cen]-p);
    
    
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
    px_rel(logical(eye(size(px_rel)))) =0; 
    py_rel(logical(eye(size(py_rel)))) =0; 
    px_sum=sum(px_rel,2);
    py_sum=sum(py_rel,2);
    delta_psi_coll = [px_sum(follower_symbol);py_sum(follower_symbol)];
    %智能体凝聚势能计算及负梯度计算,这里需要考虑到如果智能体之间存在其他智能体，那么不考虑最外面的凝聚势能

    %度小于2的智能体围绕跳数小的智能体旋转，如果度增大，停止旋转,度等于2的智能体结合局部拓扑考虑
    degree = sum(a_mtr(follower_symbol,:));
    lock=[0;0];
    if  degree == 1&&follower_symbol~=size(a_mtr,2)
        line1 = pos_set(:,follower_symbol)-p_follow;
        kk =[ -line1(2);line1(1)];
        kk = kk/norm(kk);
        lock = 1/norm(line1)^2*kk;
   
    elseif degree == 2&&follower_symbol~=size(a_mtr,2)
        degree_set = find(a_mtr(follower_symbol,:)>0);
        line1 = pos_set(:,follower_symbol)-pos_set(:,degree_set(1));
        line2 = -pos_set(:,follower_symbol)+pos_set(:,degree_set(2));
        theta1 = acos(dot(line1,line2)/(norm(line1)*norm(line2)));
        if  theta1<0.5*pi/3&&theta1>=0
            line3 = pos_set(:,follower_symbol)-p_follow;
            kk =[ -line3(2);line3(1)];
            kk = kk/norm(kk);
            if  follower_symbol/2==0
                lock = (1.8*pi/3-theta1)*1/norm(line3)^2*kk;
            else
                lock = -(1.8*pi/3-theta1)*1/norm(line3)^2*kk;
            end
        end
    end
        %计算当前智能体的周围智能体无遮挡的智能体
    theta1 = asin(2/d_n);
    theta1 = 1;
    s = find(a_mtr(follower_symbol,:)>0);
    for i = 1:size(s,2)
        r = norm(pos_set(:,follower_symbol)-pos_set(:,s(i)));
        for j =i+1:size(s,2)
            if norm(pos_set(:,s(j))-pos_set(:,follower_symbol))<r
                p1 = pos_set(:,s(j))-pos_set(:,follower_symbol);
                p2 = pos_set(:,s(i))-pos_set(:,follower_symbol);
                theta = acos(dot(p1,p2)/(norm(p1)*norm(p2)));
                if theta>0&&theta<theta1
                    a_mtr(follower_symbol,s(i)) = 0;
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
    p_dis_1 = -3.*(p_dis_1).^2;
    px_rel_1 = px_rel_1.*p_dis_1;
    py_rel_1 = py_rel_1.*p_dis_1;
    px_rel_1(logical(eye(size(px_rel_1)))) =0; 
    py_rel_1(logical(eye(size(py_rel_1)))) =0; 
    px_rel_1 = px_rel_1.*a_mtr;
    py_rel_1 = py_rel_1.*a_mtr;
    px_sum_1=sum(px_rel_1,2);
    py_sum_1=sum(py_rel_1,2);
    delta_psi_coll_1 = [px_sum_1(follower_symbol);py_sum_1(follower_symbol)];
    
  
    
    
    % 一致性势能计算
    ux_mtr=kron(u(1,:),ones(flock_size,1));
    uy_mtr=kron(u(2,:),ones(flock_size,1));
    ux_rel=(ux_mtr'-ux_mtr).*a_mtr;
    uy_rel=(uy_mtr'-uy_mtr).*a_mtr;
    ux_sum=sum(ux_rel,2)';
    uy_sum=sum(uy_rel,2)';
    
    
    a_mtr(a_mtr>0) = 1;
    u_avr = u*a_mtr';
    degree = sum(a_mtr,2);
    degree1 = [degree';degree'];
    u_avr = u_avr./degree1;
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
    if sum(temp)==0
          symbol = 0;
    else
        symbol = 1;
    end    
    k4 =5*1/(1+100*exp(100*abs(sum(temp))));                                     %sigmoid函数
    k5 =5*1/(1+50*exp(1000*abs(sum(temp))));
    k6 =20*1/(1+50*exp(1000*abs(sum(temp))));    
    %delta_psi_follow为连通度保持项，delta_psi_coll为智能体间避撞，temp为智能体与障碍间的势能函数，delta_psi_coll_1为凝聚势能，lock为解除直线锁项，delta_mean_shift为集群形状自适应项
    gra_psi_follower = k1*delta_psi_follow+k2*delta_psi_coll+k3*temp+k4* delta_psi_coll_1+0*lock+k6*delta_mean_shift;
    direction = gra_psi_follower;
    temp1 = temp;
end


