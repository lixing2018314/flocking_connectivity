%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数功能:获取邻居集合
% 输入变量:k-视场均分数,r-感知半径,s-安全半径,n-集群规模,pos_set-位置坐标,vel_set-速度状态
% 输出变量:va_mtr-视场邻接矩阵,vz_mtr-邻居个体所在视场分区,hs_mtr-智能体每个分区中心角,
%         a_mtr-邻接矩阵,d_mtr-距离矩阵,x_rel-邻居相对距离(x),y_rel-邻居相对距离(y)
% 函数说明:将智能体视场以运动方向为基准分为若干等份
function [va_mtr,vz_mtr,hs_mtr,as_mtr,d_mtr,x_rel,y_rel]=NeighbourSet(k,r,s,n,pos_set,vel_set)
    % 首先,计算感知范围内的邻居集合
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 计算距离矩阵
    x_mtr=(pos_set(1,:)'*ones(1,n));
    y_mtr=(pos_set(2,:)'*ones(1,n));
    x_rel=x_mtr'-x_mtr;
    y_rel=y_mtr'-y_mtr;
    d_mtr=sqrt((x_rel).^2+(y_rel).^2);
    % 计算邻接矩阵
    a_temp=d_mtr<=(ones(n,n).*r);
    a_mtr=a_temp-diag(ones(1,n));
    as_mtr=CalDecayFunction(d_mtr,r,s).*a_mtr;
    
    % 然后,将邻居个体按视角分为n个集合
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 获取智能体的航向角    
    head_set=atan2(vel_set(2,:),vel_set(1,:));    
    head_set(head_set<0)=head_set(head_set<0)+2*pi;    
    % 避免速度较小时航向,不断改变
    epsilon=0.25;
    vel_scale=vel_set(1,:).^2+vel_set(2,:).^2;
    head_set((abs(vel_scale)<=epsilon))=0;
    % 获取每个智能体的视角范围
    delta_vis=2*pi/k;
    delta_mtr=(ones(n,1)*(0:1:k-1)).*delta_vis-delta_vis/2;    
    vis_mtr=delta_mtr+head_set';
    vis_mtr(vis_mtr<0)=vis_mtr(vis_mtr<0)+2*pi;
    vis_mtr=rem(vis_mtr,2*pi);  
    % 计算智能体每个视角范围内的中线,依次从1到k排序
    hs_mtr=(ones(n,1)*(0:1:k-1)).*delta_vis+head_set';
    hs_mtr(hs_mtr<0)=hs_mtr(hs_mtr<0)+2*pi;
    hs_mtr=rem(hs_mtr,2*pi);      
    % 计算邻居智能体的相对视角
    nx_rel=x_rel.*a_mtr;
    ny_rel=y_rel.*a_mtr;
    nvis_mtr=atan2(ny_rel,nx_rel);
    nvis_mtr(nvis_mtr<0)=nvis_mtr(nvis_mtr<0)+2*pi;
    nvis_mtr=rem(nvis_mtr,2*pi);
    
    % 最后,获取每个集合中距离最小的智能体作为邻居个体
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vz_mtr=zeros(size(a_mtr));
    va_mtr=a_mtr;
    d_mtr=d_mtr.*va_mtr;
    nvis_mtr=nvis_mtr.*va_mtr-(~va_mtr);
    [vis_sort,index_mtr]=sort(vis_mtr,2);    
    for i=1:1:n
        % 处理前k-1个视角内的邻居个体
        for j=2:1:k
            % 找出属于当前视角范围内的邻居集合
            min_head=vis_sort(i,j-1);
            max_head=vis_sort(i,j);
            index_vis=find(nvis_mtr(i,:)>=min_head&nvis_mtr(i,:)<max_head);
            if isempty(index_vis)
                continue;
            end
            nvis_mtr(i,index_vis)=-1;
            % 找出集合内距本体距离最小的智能体
            dist_temp=d_mtr(i,index_vis);
            if length(index_vis)==1
                vz_mtr(i,index_vis)=index_mtr(i,j-1);
                continue;
            end
            [~,index_dis]=min(dist_temp);
            vz_mtr(i,index_vis(index_dis))=index_mtr(i,j-1);
            % 将集合中其他智能体对应的邻接矩阵置0
            index_vis(index_dis)=[];
            va_mtr(i,index_vis)=false;            
        end
        % 处理第k个视角内的邻居个体
        % 找出属于当前视角范围内的邻居集合
        index_vis=find(nvis_mtr(i,:)>=0);
        if isempty(index_vis)
            continue;
        end
        nvis_mtr(i,index_vis)=-1;
        % 找出集合内距本体距离最小的智能体
        dist_temp=d_mtr(i,index_vis);
        if length(index_vis)==1
            vz_mtr(i,index_vis)=index_mtr(i,k);
            continue;
        end
        [~,index_dis]=min(dist_temp);
        vz_mtr(i,index_vis(index_dis))=index_mtr(i,k);
        % 将集合中其他智能体对应的邻接矩阵置0
        index_vis(index_dis)=[];
        va_mtr(i,index_vis)=false;
    end   
    
    % 输出变量赋值
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%     d_mtr=d_mtr.*va_mtr;
%     x_rel=x_rel.*va_mtr;
%     y_rel=y_rel.*va_mtr;
    d_temp=d_mtr+r.*(~va_mtr);
    d_temp(d_temp>r)=r;
    d_temp(d_temp<2*s)=2*s;
    va_mtr=CalDecayFunction(d_temp,r,s).*va_mtr;
end

%% 辅助函数库%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数功能:计算衰减函数
function y=CalDecayFunction(x,r,s)
    y=(1+cos(pi.*(x-2*s)./(r-2*s)))/2;
    y(x<2*s)=1;
    y(x>r)=0;
end