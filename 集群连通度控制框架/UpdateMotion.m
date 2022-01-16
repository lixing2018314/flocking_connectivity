%% %%%%%%%%%%%%%%%%%%%
% 函数功能：智能体状态更新
% 函数输入：T-控制周期,a_mtr-邻接矩阵,pos_set-位置集合,vel_set-速度集合,
% 函数输出：位置矩阵以及速度矩阵

function [Pset,Vset] = UpdateMotion(T,a_mtr,pos_set,vel_set,hop_mtr,e_cen,AvoidV,guideV,p_ref_period,v_ref_period,informed_robots,v_track)
    
    h=4;
    h1 = 6;
    R_com =6;
    R_com1 = 8.4;

    v_max =2;
    [flock_size,~] = size(a_mtr);
    vel_set1 = vel_set;
    Vset = vel_set;
    %集群速度一致项
    vx_mtr=kron(vel_set(1,:),ones(flock_size,1));
    vy_mtr=kron(vel_set(2,:),ones(flock_size,1));
    vx_rel=(vx_mtr'-vx_mtr).*a_mtr;
    vy_rel=(vy_mtr'-vy_mtr).*a_mtr;    
    vx_sum=sum(vx_rel,2)';
    vy_sum=sum(vy_rel,2)';
    
    %集群斥力控制项
    RepVset =Rep(pos_set,a_mtr,h,R_com);
    
  
    %集群连通度控制项
    
    v_conn = ConnMaintain(hop_mtr,pos_set,a_mtr,h1,R_com1,e_cen);
    
    %mean_shift控制项
     v_mean_shift =  MeanShift(hop_mtr,pos_set,a_mtr,h1,R_com1,e_cen);
   
%    % 合并mean_shift与集群连通度
%     for i = 1:flock_size
%         if(AvoidV(1,i)==0&&AvoidV(1,i)==0)
%             v_conn(:,i) =v_mean_shift(:,i);
%         else
%             guideV = 0.8*guideV;
%   
%         end
%             
%     end
%     
    
    %智能体速度一致
    a_mtr(a_mtr>0) = 1;
    v_avr = vel_set*a_mtr';
    degree = sum(a_mtr,2);
    degree1 = [degree';degree'];
    v_avr = v_avr./degree1;
    
    
    %速度更新
    
   % Vset= -(k1.*[vx_sum;vy_sum]);%+10*RepVset+5*v_conn+5*AvoidV+guideV;
    
    k1 = 1;
    k2 =500;
    k3 =150;
    k4 =1;
    k5 =100;
    c2 = 5;
%    v_track = zeros(2,flock_size);
    
%     for j=1:flock_size
%         if (informed_robots(j) == 1)
%             v_track(:,j) = c2*(p_ref_period(:,j)-pos_set(:,j))+v_ref_period(:,j);
%         end
%     end
    
    
    
   % Vset=v_avr+k3*v_conn+k2*RepVset+k5*AvoidV+0.5*v_track;
    % Vset=k4*v_avr+v_track;
  %   Vset= RepVset+v_conn+3*AvoidV;
   % Vset= guideV+v_avr;
%     for index = 1:flock_size
%         if norm(guideV(:,index)) > 0
%            % guideV(:,index) = guideV(:,index)/norm(guideV(:,index));
% %             if norm(v_conn(:,index))>0.9
% %                  guideV(:,index) = 0;
% %                  v_conn(:,index) = 5*v_conn(:,index);
% %             end
% 
%         end
%         
%     end
%     for   index =1:flock_size
%         if norm(v_conn(:,flock_size))<0.6
%             %                  v_conn(:,index) = 5*v_conn(:,index);
%             %Vset(:,index)=v_avr(:,index) + 100*RepVset(:,index)+25*AvoidV(:,index)+2*guideV(:,index);
%             Vset=1.5*v_avr + 100*RepVset+1*AvoidV+guideV;
%         end
%         if norm(v_conn(:,flock_size))<0.9&&norm(v_conn(:,flock_size))>=0.6
%             % Vset(:,index)=1200*v_conn(:,index)+25*AvoidV(:,index);
%             Vset = vel_set;
%             %                  guideV(:,index) = 0.5* guideV(:,index);
%         end
% 
%         if norm(v_conn(:,flock_size))>0.9
%            % Vset(:,index)=1200*v_conn(:,index)+25*AvoidV(:,index);
%            Vset=1200*v_conn+25*AvoidV;
%             %                  guideV(:,index) = 0.5* guideV(:,index);
%         end
%     end
    
%     if norm(v_conn(:,flock_size))>1
%         Vset=10*v_conn+RepVset+0.1*guideV+5*v_avr+20*AvoidV;
%     else
%          Vset=10*v_conn+RepVset+0.1*guideV+v_avr+20*AvoidV;
%     end
%      Vset=10*RepVset+3*v_avr+0.1*guideV+AvoidV;%+0.3*guideV+v_avr;%+50*AvoidV;
 %     Vset=10*RepVset+0.1*guideV+v_avr;
 %     Vset=20*v_conn+v_avr + 100*RepVset+0.13*guideV+25*AvoidV;
   %  Vset=v_conn+RepVset;
    %一阶惯性环节
%     Vset=50*v_conn+v_avr + 30*RepVset+2*v_track+20*AvoidV;
%     Vset=50*v_conn+v_avr + 30*RepVset;
%     Vset(Vset>v_max) =v_max;
%      Vset(Vset<-v_max) =-v_max;
%     ratio = 0.1;
  %  Vset = ratio*vel_set1 +(1-ratio)*Vset;
   % Vset=50*v_conn;
   
   %v_track(:,1:flock_size-1) = 0;
  % v_conn(:,flock_size) = 0;
   %v_avr(:,flock_size) = 0;

    Vset=250*v_conn+v_avr + 5*RepVset+0.005*guideV+30*AvoidV;
    Pset=pos_set+Vset.*T; 
    Vset(Vset>v_max) =v_max;
    Vset(Vset<-v_max) =-v_max;
    ratio = 0.1;
    Vset = ratio*vel_set1 +(1-ratio)*Vset;
    
       
end





%% 函数库

% 函数功能： 智能体之间的斥力函数，这里为速度排斥项
% 函数输入：智能体位置矩阵P_set,邻接矩阵A_mtr,h为智能体安全半径，R_rep为排斥开始距离
% 函数输出：每个智能体所受斥力速度

function  RepVset =Rep(P_set,A_mtr,h,R_rep)
    [flock_size,~] = size(A_mtr);
    dis = zeros(flock_size,flock_size);
    px_mtr=kron(P_set(1,:),ones(flock_size,1));
    py_mtr=kron(P_set(2,:),ones(flock_size,1));
    px_rel=(px_mtr'-px_mtr);
    py_rel=(py_mtr'-py_mtr);
    px_dis = sqrt(px_rel.^2+py_rel.^2);
  
    
    for i=1:flock_size
        for j=1:flock_size
            dis(i,j) = bump(px_dis(i,j),h,R_rep); 
            
        end
    end
    px_dis(px_dis==0)=1;
    px_rel = px_rel./px_dis;
    py_rel = py_rel./px_dis;
    res_x = px_rel*dis;
    res_y = py_rel*dis;
    res_x =diag(res_x,0);
    res_y =diag(res_y,0);
    RepVset = [res_x';res_y'];

end


% 函数功能:凸函数
% 输入变量:计算变量,其中h为智能体的安全半径，z为智能体之间的距离,R_rep为排斥开始半径
% 输出变量:计算结果
function y=bump(z,h,R_rep)
if z>h&&z<=R_rep
    y=0.5*(1+cos(pi*(z-h)/(R_rep-h)));
%     if(z-h<=0.01)    
%         y = 100;
%     else
%         y = 1/(z-h)-1/(R_rep-h);
%     end
 end
if z>=0 && z<=h
    y=1;
end
if z>R_rep
    y=0;
end
end


% 函数功能：连通度保持计算函数,需要改变策略，寻找在智能体圈内最近的智能体进行跟随，至少要标号相同，更新后的连通度保持函数
% 输入变量：跳数矩阵
% 输出变量：每个智能体所受连通性保持的速度
% 
function v_conn = ConnMaintain(hop_mtr,P_set,A_mtr,h,R_com,e_cen)
    
    [flock_size,~] = size(A_mtr);
    oneHop =zeros(1,flock_size);
    v =zeros(2,flock_size);
    dis = zeros(flock_size,flock_size);
    px_mtr=kron(P_set(1,:),ones(flock_size,1));
    py_mtr=kron(P_set(2,:),ones(flock_size,1));
    px_rel=(px_mtr'-px_mtr);
    py_rel=(py_mtr'-py_mtr);
    px_dis = sqrt(px_rel.^2+py_rel.^2); 
    symbol = -5;
    for i = 1:flock_size
         symbol = -5;
        dis_min =0;
        min_hop = hop_mtr(i);
        dis1 =10000;
        leader_index = 0;
        for j=1:flock_size
            dis(i,j) = bump1(px_dis(i,j),h,R_com,hop_mtr(i)); 
            if A_mtr(i,j)>0
                %寻找跳数小1的跟随，跳数相同，选择距离中最近的跟随
                if hop_mtr(j)<=hop_mtr(i)
                    min_hop =hop_mtr(j);
                    %比较两者与中心的距离，如果距离进，则有可能会跟随，还要比较这两个智能体间的距离
                    if norm(P_set(:,i)-e_cen(:,i))>norm(P_set(:,j)-e_cen(:,i))
                      
                        if norm(P_set(:,i)-P_set(:,j))<dis1
                            dis1 = norm(P_set(:,i)-P_set(:,j));
                            symbol =j;
                        end
                    end              
                end
                
            end
            
        end
        
        if symbol==-5
            symbol =i;
        end
        for k=1:flock_size
            if k ~= symbol
                px_rel(i,k)=0;
                py_rel(i,k)=0;
            end
        end
    end
    px_dis(px_dis==0)=1;
    px_rel = px_rel./px_dis;
    py_rel = py_rel./px_dis;
    res_x = px_rel*dis';
    res_y = py_rel*dis';
    res_x =diag(res_x,0);
    res_y =diag(res_y,0);
    v_conn = [-res_x';-res_y'];
    v_conn(:,flock_size) = 0;
end


%函数功能：凸函数,conn与mean shift聚类算法共用
%输入变量:智能体距离z，可调变量h，通信半径R_com，智能体跳数hop
%输出变量：计算结果

function y=bump1(z,h,R_com,hop)
if z>=h&&z<=R_com
%     if(R_com-z<=0.01)    
%         y = 100;
%     else
%         y = 1/(R_com-z)-1/(R_com-h);
%     end
    y = 0.5*(1+cos(pi*(R_com-z)/(R_com-h)));
end
if z>=0 && z<h
    y=0;
end
if z>R_com
    y=0;
end
end



%函数功能：mean shift 聚类算法
%输入变量：
%输出变量：
function v_mean_shift = MeanShift(hop_mtr,P_set,A_mtr,h,R_com,e_cen)
   [flock_size,~] = size(A_mtr);
    oneHop =zeros(1,flock_size);
    v_center =zeros(2,flock_size);
    dis = zeros(flock_size,flock_size);
    px_mtr=kron(P_set(1,:),ones(flock_size,1));
    py_mtr=kron(P_set(2,:),ones(flock_size,1));
    px_rel=(px_mtr'-px_mtr);
    py_rel=(py_mtr'-py_mtr);
    px_dis = sqrt(px_rel.^2+py_rel.^2); 
    
    symbol_mtr = zeros(1,flock_size);
    mean_count = zeros(1,flock_size);
 
    
    
    for i = 1:flock_size
        dis_min =0;
        min_hop = hop_mtr(i);
        if hop_mtr(i) == 1
           oneHop(i) = 1;
           continue;
        end
        symbol_mtr(symbol_mtr>0) = 0;
        dis1 =norm(P_set(:,i)-e_cen(:,i));
        for j=1:flock_size
            dis(i,j) = bump1(px_dis(i,j),h,R_com,hop_mtr(i)); 
            
            if A_mtr(i,j)>0
                %寻找跳数小1的跟随，跳数相同，选择距离中最近的跟随
                if hop_mtr(j)<hop_mtr(i)
                    
                    min_hop =hop_mtr(j);
                    %比较两者与中心的距离，如果距离进，则有可能会跟随，还要比较这两个智能体间的距离
                    if norm(P_set(:,i)-e_cen(:,i))>norm(P_set(:,j)-e_cen(:,i))
                        if norm(P_set(:,i)-P_set(:,j))<dis1
                            dis1 = norm(P_set(:,i)-P_set(:,j));
                            symbol_mtr(j) = 1;
                        end
                    end 
                elseif  hop_mtr(j)==hop_mtr(i)
                    symbol_mtr(j) = 1;
     
                    
                end
            end
            
        end
        mean_count(i) = sum(symbol_mtr);
        for k=1:flock_size
            if symbol_mtr(k) ==0
                px_rel(i,k)=0;
                py_rel(i,k)=0;
            end
        end
    end
    px_dis(px_dis==0)=1;
    px_rel = px_rel./px_dis;
    py_rel = py_rel./px_dis;
    res_x = px_rel*dis';
    res_y = py_rel*dis';
    res_x =diag(res_x,0);
    res_y =diag(res_y,0);
    v_conn = [-res_x';-res_y'];
    mean_count_tran = [mean_count;mean_count];
    for m=1:flock_size
        %v_center(:,m) = bump1(R_com,h,R_com,hop_mtr(1))*(e_cen(:,m)-P_set(:,m))/norm((e_cen(:,m)-P_set(:,m)));
        v_center(:,m) = bump1(R_com,h,R_com,hop_mtr(1))*(e_cen(:,m)-P_set(:,m));
    end
    v_conn = v_conn+v_center/15;
    v_conn = v_conn./(mean_count_tran+1);    %求得mean_shift速度
    
    for  k =1:flock_size
         symbol_mtr( symbol_mtr>0) = 0;
        if oneHop(k) ==1
            dis1 = norm(P_set(:,k)-e_cen(:,k));
            for index = 1:flock_size
                if oneHop(index)==1
                   if  norm(P_set(:,k)-e_cen(:,k))>norm(P_set(:,index)-e_cen(:,k))
                        if norm(P_set(:,k)-P_set(:,index))<dis1
                                dis1 = norm(P_set(:,k)-P_set(:,index));
                                symbol_mtr(index) =1;
                        end
                   end
                end
              
            end 
            v_conn(:,k) = 0;
            for q=1:flock_size
                if symbol_mtr == 1
                    D =P_set(:,q)-P_set(:,k);
                    w = bump1(norm(D),h,R_com,1);
                    v1 =w*D/norm(D);
                    v_conn(:,k) = v_conn(:,k)+v1;
                end
            end
            
            D =e_cen(:,k)-P_set(:,k);
            w = bump1(norm(D),4,6,1);
            v1 =w*D/norm(D);
            v_conn(:,k) = v_conn(:,k)+20*v1;
            v_conn(:,k) = v_conn(:,k)./(sum(symbol_mtr)+1);
        end
       
       
    end
    v_mean_shift = v_conn;

end


% 更新前的连通度保持函数
% function v_conn = ConnMaintain(hop_mtr,P_set,A_mtr,h,R_com,e_cen)
%     
%     [flock_size,~] = size(A_mtr);
%     oneHop =zeros(1,flock_size);
%     v =zeros(2,flock_size);
%     dis = zeros(flock_size,flock_size);
%     px_mtr=kron(P_set(1,:),ones(flock_size,1));
%     py_mtr=kron(P_set(2,:),ones(flock_size,1));
%     px_rel=(px_mtr'-px_mtr);
%     py_rel=(py_mtr'-py_mtr);
%     px_dis = sqrt(px_rel.^2+py_rel.^2); 
%     for i = 1:flock_size
%         
%         dis_min =0;
%         min_hop = hop_mtr(i);
%         if hop_mtr(i) == 1
%            oneHop(i) = 1;
%            continue;
%         end
%         dis1 =norm(P_set(:,i)-e_cen(:,i));
%         for j=1:flock_size
%             dis(i,j) = bump1(px_dis(i,j),h,R_com,hop_mtr(i)); 
%             if A_mtr(i,j)>0
%                 %寻找跳数小1的跟随，跳数相同，选择距离中最近的跟随
%                 if hop_mtr(j)<=hop_mtr(i)
%                     min_hop =hop_mtr(j);
%                     %比较两者与中心的距离，如果距离进，则有可能会跟随，还要比较这两个智能体间的距离
%                     if norm(P_set(:,i)-e_cen(:,i))>norm(P_set(:,j)-e_cen(:,i))
%                         if norm(P_set(:,i)-P_set(:,j))<dis1
%                             dis1 = norm(P_set(:,i)-P_set(:,j));
%                             symbol =j;
%                         end
%                     end              
%                 end
%             end
%             
%         end
%         for k=1:flock_size
%             if k ~= symbol
%                 px_rel(i,k)=0;
%                 py_rel(i,k)=0;
%             end
%         end
%     end
%     px_dis(px_dis==0)=1;
%     px_rel = px_rel./px_dis;
%     py_rel = py_rel./px_dis;
%     res_x = px_rel*dis';
%     res_y = py_rel*dis';
%     res_x =diag(res_x,0);
%     res_y =diag(res_y,0);
%     v_conn = [-res_x';-res_y'];
%     
%     for  k =1:flock_size
%         if oneHop(k) ==1
%             dis1 = norm(P_set(:,k)-e_cen(:,k));
%             symbol=1000;
%             for index = 1:flock_size
%                 if oneHop(index)==1
%                    if  norm(P_set(:,k)-e_cen(:,k))>norm(P_set(:,index)-e_cen(:,k))
%                         if norm(P_set(:,k)-P_set(:,index))<dis1
%                                 dis1 = norm(P_set(:,k)-P_set(:,index));
%                                 symbol =index;
%                         end
%                    end
%                 end
%               
%             end
%             if symbol == 1000
%                 D =e_cen(:,k)-P_set(:,k);
%                 w = bump1(norm(D),h-3,R_com,1);
%                 v(:,k) =w*D/norm(D);
%                 v_conn(:,k) = v(:,k);
%                   v_conn(:,k) =  v(:,k);
%             else
%                 D =P_set(:,symbol)-P_set(:,k);
%                 w = bump1(norm(D),h,R_com,1);
%                 v(:,k) =w*D/norm(D);
%                 v_conn(:,k) = v(:,k);
%             end
%             
%         end
%     end
%     
% end
% 
% 
% 










