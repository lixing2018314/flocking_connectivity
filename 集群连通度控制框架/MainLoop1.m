%% 搭建控制框架
%采用解耦的群中心控制法，主要目标:1保持一个圆形区域的集群2保持连通度


clear;
clc;
%% 仿真参数设置
% 定时器参数
T=0.01;                                                                    % 控制间隔ms
Time=150;                                                                  % 定时器运行时间      
maxcount=ceil(Time/T);                                                     % 定时器最大计数

%集群控制参数
R_body=1.5;
Vel_max=5;
Vis_num=6;
R_safe=2;                                                                  % 智能体安全半径
R_com=8;                                                                   % 智能体通信半径
Sim_range=[0,300,-100,100];                                                % 坐标轴范围

%集群初始状态
Flock_size =10;
pos_init = [75,15]';     
[P_set,V_set]=InitialState(Flock_size,1.2*R_com,4*R_safe,pos_init);
% P_set(2,:) = 10;
%  P_set(1,1) =10;
% for i=2:Flock_size
%     P_set(1,i) = P_set(1,1)+5*(i-1);
% end
Flock_hop = 2000*ones(1,Flock_size);                                       %集群跳数矩阵
e_cen =P_set;
v_set =zeros(2,Flock_size);

%设置多个领导者，并且其中一个领导者跳数置为0
leader_size = 1;
leader_set = zeros(1,leader_size);
leader_set(1,1) = Flock_size;
Flock_hop(leader_set(1,1)) =0;
informed_robots = zeros(1,Flock_size);
informed_robots(Flock_size-1:Flock_size) = 1;

%% 任务参数设置
% 定点静态任务
Pos_st=[300 45 ]';
Vel_st=[20 0 ]';

% 生成航路点的任务
pointCount = 1500;
[p_ref,v_ref] = CreateReferTraj(pos_init,Pos_st,pointCount);
v_ref_peroid = zeros(2,Flock_size);
p_ref_peroid = zeros(2,Flock_size);
plot(p_ref(1),p_ref(2));
for i=1:Flock_size
    v_ref_period(1,i) = 5;
    v_ref_period(2,i) = 0;
    p_ref_period(2,i) = 10;
end
pos_off = zeros(2,Flock_size);

% 仿真记录变量
FlockHop_rec =zeros(1,maxcount+1,Flock_size);                              %记录仿真跳数变化
Pset_rec = zeros(2,maxcount+1,Flock_size); Vset_rec = zeros(2,maxcount+1,Flock_size);
Ecen_set = P_set;  Ecev_set = V_set;
Ecen_rec(:,1,:)=Ecen_set; Ecev_rec(:,1,:)=Ecev_set;  
Rcen_rec(:,1)=sum(P_set,2)./Flock_size; Rcev_rec(:,1)=sum(V_set,2)./Flock_size;
lamda2_rec = zeros(1,maxcount+1);
u = zeros(2,Flock_size);

% 障碍设置
%连续圆形通道障碍,S型障碍
% r = 40;
% r1 = 20;
% r2 = 60;
% theta=pi/2:-pi/5000:0;
% x_1 =30*cos(theta)+110;
% y_1 = 30*sin(theta)-15;
% theta1=pi:pi/5000:pi*3/2;
% x_2 =50*cos(theta1)+190;
% y_2 = 50*sin(theta1)-15;
% theta=pi/2:-pi/100:0;
% x =r*cos(theta)+110;
% y = r*sin(theta)-15;
% x1 =r1*cos(theta)+110;
% y1 = r1*sin(theta)-15;
% theta1=pi:pi/100:pi*3/2;
% x2 =r*cos(theta1)+190;
% y2 = r*sin(theta1)-15;
% x3 =r2*cos(theta1)+190;
% y3 = r2*sin(theta1)-15;
% u = zeros(2,Flock_size);
% Obs_r = 5;
% Obs_set=[];
% %上轨线全体坐标
% x_obs_1 = [x,x2(2:end)];
% y_obs_1 = [y,y2(2:end)];
% %下轨线一段坐标
% x_obs_2 = x1;
% y_obs_2 = y1;
% %下轨线第二段坐标
% x_obs_3 = x3(2:end);
% y_obs_3 = y3(2:end);
% for i=1:15
%  obs = [x_obs_1((i-1)*7+1),y_obs_1((i-1)*7+1),Obs_r]';
%  Obs_set = [Obs_set,obs];
% end
% for i=1:4
%  obs = [x_obs_2((i-1)*14+1),y_obs_2((i-1)*14+1),Obs_r]';
%  Obs_set = [Obs_set,obs];
% end
% for i=1:10
%  obs = [x_obs_3((i-1)*5+1),y_obs_3((i-1)*5+1),Obs_r]';
%  Obs_set = [Obs_set,obs];
% end







%  Obs1=[130,30,15]';                                                       % 障碍1
%  Obs2=[130,-10,15]';                                                      % 障碍2
%  Obs3=[130,-50,15]';                                                      % 障碍3
%  Obs4=[130,-90,15]';                                                      % 障碍4
%  Obs_set=[Obs1,Obs2,Obs3,Obs4];
%正方形障碍设计
%设置起始坐标
x_ini = 100;
y_ini = 42;
Obs_set=zeros(3,1);
%计算n列障碍
x_ini_1=x_ini;
for i=1:3
    
    y_ini_1=y_ini;
    for j=1:4
        obs=[x_ini_1,y_ini_1,5]';
        y_ini_1 = y_ini_1-18;
        Obs_set=[Obs_set,obs];
    end
    x_ini_1 =x_ini_1+30;
end
%计算m列障碍
x_ini_2=x_ini+15;
for i=1:2
    y_ini_1=y_ini-5;
    for j=1:4
        obs=[x_ini_2,y_ini_1,5]';
        y_ini_1 = y_ini_1-18;
        Obs_set=[Obs_set,obs];
    end
    x_ini_2 =x_ini_2+30;
end




Obs_size=length(Obs_set(1,:));
obstacle_count = size(Obs_set,2);
pos_v_log= zeros(2,obstacle_count,Flock_size);
p_ad = zeros(obstacle_count,Flock_size);

%% 绘图初始化
% 绘图标志位设定:实时显示,群中心估计,目标跟踪,控制输入,运动状态,运动轨迹
draw_proc=1; draw_esti=0; draw_track=0; draw_input=0; draw_state=0; draw_traj=0;
% 绘图初始化
if draw_proc
    [Hop,Ha_pos,Ha_vel,He_pos,Hc_pos]=DrawSomProcess(R_body,Sim_range,P_set,V_set,Ecen_set,Rcen_rec(:,1),Flock_hop);
end
 [As_set,~,~,A_set,~,~,~]=NeighbourSet(Vis_num,R_com,R_safe,Flock_size,P_set,V_set);
 A_set(A_set>0)=1;
 Com=DrawComNetwork(Flock_size,P_set,A_set);
 DrawCircleObstacles(Obs_set)  %绘制圆形障碍
%  plot(x,y,'-','color','r');
%   plot(x1,y1,'-','color','r');
%  plot(x2,y2,'-','color','r');
%  plot(x3,y3,'-','color','r');
%  plot(x_1,y_1,'-','color','r');
%   plot(x_2,y_2,'-','color','r')
 

%% 主控制程序
for i=1:maxcount
    %求解邻居集
    [As_set,~,~,A_set,~,~,~]=NeighbourSet(Vis_num,R_com,R_safe,Flock_size,P_set,V_set);
    NeighborSet = A_set;
    NeighborSet(NeighborSet>0) = 1;
    UpdateComNetwork(Com,Flock_size,P_set,NeighborSet)
    K = sum(A_set,2);
    L = diag(K,0)-A_set;
    %计算集群的代数连通度
    [m,n] = eig(L);
    lamda2_rec(1,i) = n(2,2);
    %生成参考轨迹（对应轨迹跟踪控制）
%     if i<=4600
%         for f=1:Flock_size
%             p_ref_period(1,f) = 70+0.05*(i);
%             p_ref_period(2,f) = 15;
%             v_ref_period(1,f) = 0.05;
%         end
%     elseif i>4600&&i<=6300
%          for f=1:Flock_size
%             p_ref_period(1,f) = 300;
%             p_ref_period(2,f) = 15-0.05*(i-4600);
%             v_ref_period(1,f) = 0.05;
%          end
%     else
%          for f=1:Flock_size
%             p_ref_period(1,f) = 300-0.05*(i-6300);
%             p_ref_period(2,f) = -70;
%             v_ref_period(1,f) = 0.05;
%          end
%         
%     end
 
%     for f=1:Flock_size
%         p_ref_period(1,f) = x_1(i);
%         p_ref_period(2,f) = y_1(i);
%         v_ref_period(1,f) = 0.07;
%     end


%  S型轨迹时的点
%     if  i<=800
%         for f=1:Flock_size
%             p_ref_period(1,f) = 70+0.05*(i);
%             p_ref_period(2,f) = 15;
%             v_ref_period(1,f) = 0.05;
%         end
%     elseif i<=2501&&i>800
%         for f=1:Flock_size
%             p_ref_period(1,f) = x_1(i);
%             p_ref_period(2,f) = y_1(i);
%             v_ref_period(1,f) = -0.05*cos(atan(-(x_1(i)-110)/(y_1(i)+15.1)));
%             v_ref_period(2,f) = 0.05*sin(atan(-(x_1(i)-110)/(y_1(i)+15.1)));
%         end
%     elseif i<=5002
%         for f=1:Flock_size
%            p_ref_period(1,f) = x_2(i-2501);
%             p_ref_period(2,f) = y_2(i-2501);
%            v_ref_period(1,f) = -0.07*cos(atan(-(x_2(i-2501)-110)/(y_2(i-2501)+15.1)));
%             v_ref_period(2,f) = 0.07*sin(atan(-(x_2(i-2501)-110)/(y_2(i-2501)+15.1)));
%         end
%     elseif i<=8000
%          for f=1:Flock_size
%            p_ref_period(1,f) =190+0.03*(i);
%             p_ref_period(2,f) = -55;
%             v_ref_period(1,f) = 0.04;
%         end
%     end
% 点数设置 （100，9），（115，4），（130，9），（145，4），（160，9）
    if  i<=320
        for f=1:Flock_size
            p_ref_period(1,f) = 100+(0.15/(10^0.5))*i;
            p_ref_period(2,f) = 9-(0.05/(10^0.5))*i;
            v_ref_period(1,f) = (0.15/(10^0.5));
            v_ref_period(2,f) = -(0.05/(10^0.5));
        end
    elseif i<=640&&i>320
        for f=1:Flock_size
            p_ref_period(1,f) = 115+(0.15/(10^0.5))*(i-320);
            p_ref_period(2,f) = 4+(0.05/(10^0.5))*(i-320);
            v_ref_period(1,f) = (0.15/(10^0.5));
            v_ref_period(2,f) = (0.05/(10^0.5));
        end
    elseif i<=960&&i>640
        for f=1:Flock_size
            p_ref_period(1,f) = 135+(0.15/(10^0.5))*(i-640);
            p_ref_period(2,f) =  9-(0.05/(10^0.5))*(i-640);
            v_ref_period(1,f) = (0.15/(10^0.5));
            v_ref_period(2,f) = -(0.05/(10^0.5));
        end
    elseif  i<=1280&&i>960
         for f=1:Flock_size
           p_ref_period(1,f) = 115+(0.15/(10^0.5))*(i-960);
            p_ref_period(2,f) = 4+(0.05/(10^0.5))*(i-960);
            v_ref_period(1,f) = (0.15/(10^0.5));
            v_ref_period(2,f) = (0.05/(10^0.5));
         end
    elseif i<=1800&&i>1280
        for f=1:Flock_size
            p_ref_period(1,f) = 160+0.05*(i-1280);
            p_ref_period(2,f) = 9;
            v_ref_period(1,f) = 0.05;
        end
    else
         for f=1:Flock_size
            p_ref_period(1,f) = 160+0.05*(1800-1280);
            p_ref_period(2,f) = 9;
            v_ref_period(1,f) = 0.05;
        end
        
    end







   %更新智能体跳数
    Flock_hop = UpdateHop(A_set,Ecen_set,Flock_hop,R_com,P_set,leader_set(end));
    [follow_agent,other_agent] = calConstrainAgent(Flock_hop,P_set,NeighborSet,leader_set(end)) ;   %计算跟随智能体与其他约束智能体
    %估计群中心位置并控制
    %[e_cen,ev_cen]=GroupCenterEstimate(T,Flock_size,A_set,Ecen_set,Ecev_set,u,P_set);
    [e_cen,ev_cen]=GroupCenterControl(Flock_size,A_set,e_cen,v_set,p_ref_period,v_ref_peroid,informed_robots);
    Ecen_set =e_cen;
    %更新智能体运动,一阶系统,需要添加中心聚集力
    [u, P_set] = updateState(P_set,follow_agent,other_agent,Pos_st,Obs_set,NeighborSet,u,leader_set,e_cen,p_ref_period,v_ref_peroid,informed_robots);
    v_set = u;
    %记录仿真变量
    Pset_rec(:,i+1,:)=P_set; Vset_rec(:,i+1,:)=V_set;  
   % Gpos_rec(:,i+1)=Goal_pos; Gvel_rec(:,i+1)=Goal_vel; Gacc_rec(:,i+1)=Goal_acc;  
     Rcen_rec(:,i+1)=sum(P_set,2)./Flock_size; Rcev_rec(:,i+1)=sum(V_set,2)./Flock_size;     
    Ecen_rec(:,i+1,:)=Ecen_set; Ecev_rec(:,i+1,:)=Ecev_set;
    FlockHop_rec(:,i+1,:) = Flock_hop;
 
    %更新绘图状态
    if draw_proc
        UpdateSimProcess(Hop,Ha_pos,Ha_vel,He_pos,Hc_pos,R_body,P_set,V_set,Ecen_set,Rcen_rec(:,i+1),Flock_hop);
    end
    
end






%% 曲线绘制

% 绘制群中心状态估计值
if draw_esti
    DrawCenterEstimate(T,maxcount,Flock_size,Ecen_rec,Ecev_rec,Rcen_rec,Rcev_rec);
end
% 绘制智能体跟踪状态
if draw_track
    DrawTrackStates(T,maxcount,Flock_size,Pset_rec,Vset_rec,Fset_rec,Ecen_rec,Ecev_rec);
end
% 绘制智能体运动状态
if draw_state
    DrawMotionStates(T,maxcount,Flock_size,Pset_rec,Vset_rec,Fset_rec,Rcen_rec,Rcev_rec);
end
% 绘制智能体运动轨迹
if draw_traj
    DrawMotionTrajectory(Flock_size,Sim_range,Pset_rec,Rcen_rec);
end






















%% 函数库
% 函数功能：实时绘制系统仿真过程
function [hop,ha_pos,ha_vel,he_pos,hc_pos]=DrawSomProcess(r,sim_range,pos_set,vel_set,ecp_set,cen_pos,Flock_hop)
    figure(1);
    box on; hold on; grid on; axis equal;
    set(gcf,'units','normalized','position',[0.05,0.05,0.9,0.85]);
    % 绘制智能体群状态(位置,速度)
    [ha_pos,ha_vel,hop]=DrawAgentGroup(pos_set,vel_set,r,Flock_hop); 
    % 绘制智能体估计中心
    he_pos=plot(ecp_set(1,:),ecp_set(2,:),'+','color',[0.9290 0.6940 0.1250],'markersize',3);
    he_pos.Color(4)=0.1;
    % 绘制智能体真实中心
    hc_pos=plot(cen_pos(1,:),cen_pos(2,:),'rs','markersize',8);
    % 绘图属性
    xlabel('x axis (m)');
    ylabel('y axis (m)');
    set(gca,'FontSize',10,'Fontname','Times New Roman');
    axis(sim_range);
    set(gca,'xtick',sim_range(1):(sim_range(2)-sim_range(1))/4:sim_range(2));
    set(gca,'ytick',sim_range(3):(sim_range(4)-sim_range(3))/4:sim_range(4)); 
end

% 函数功能:绘制智能体群运动状态,添加了智能体跳数更新
function [ha_pos,ha_vel,hop]=DrawAgentGroup(pos_set,vel_set,r,Flock_hop)
    scale=0.2;
    num_agent=length(pos_set(1,:));
    for i=1:1:num_agent
        rect=[pos_set(1,i)-r,pos_set(2,i)-r,2*r,2*r];
        ha_pos(i)=rectangle('Position',rect,'Curvature',[1,1],'EdgeColor',[0 0.4470 0.7410]); 
        temp_x=[pos_set(1,i),pos_set(1,i)+scale.*vel_set(1,i)];
        temp_y=[pos_set(2,i),pos_set(2,i)+scale.*vel_set(2,i)];
        hop(i) = text;
        set(hop(i),'Position',[pos_set(1,i),pos_set(2,i)],'String',num2str(Flock_hop(i)),'FontSize',10);       
        ha_vel(i)=plot(temp_x,temp_y,'r','linewidth',1.0);
        
    end
end


% 函数功能:更新系统仿真过程
function UpdateSimProcess(hop,ha_pos,ha_vel,he_pos,hc_pos,r,pos_set,vel_set,ecp_set,cen_pos,Flock_hop)
    % 更新智能体群状态(位置,速度)
    UpdateAgentGroup(ha_pos,ha_vel,hop,pos_set,vel_set,r,Flock_hop);
    % 更新智能体估计中心
     set(he_pos,'XData',ecp_set(1,:),'YData',ecp_set(2,:));
%     % 更新智能体真实中心
     set(hc_pos,'XData',cen_pos(1,:),'YData',cen_pos(2,:));
    drawnow;
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% 函数功能:更新智能体群运动状态,添加了智能体跳数更新
function UpdateAgentGroup(ha_pos,ha_vel,hop,pos_set,vel_set,r,Flock_hop)
    scale=0.2;
    num_agent=length(pos_set(1,:));
    for i=1:1:num_agent
        rect=[pos_set(1,i)-r,pos_set(2,i)-r,2*r,2*r];
        if i == num_agent
          set(ha_pos(i),'EdgeColor','g');  
        end
        set(ha_pos(i),'Position',rect);
        temp_x=[pos_set(1,i),pos_set(1,i)+scale.*vel_set(1,i)];
        temp_y=[pos_set(2,i),pos_set(2,i)+scale.*vel_set(2,i)];
        set(hop(i),'Position',[pos_set(1,i),pos_set(2,i)],'String',num2str(Flock_hop(i)));  
        set(ha_vel(i),'XData',temp_x,'YData',temp_y);
    end    
end


% 函数功能:绘制集群通信网络
% 输入变量:集群规模,群体位置,邻居集
% 输出变量:集群通信网络的绘制句柄
function Com=DrawComNetwork(Flock_size,Pos,NeighborSet)
for i=1:Flock_size-1
    for j=i+1:Flock_size
        Com(i,j)=plot([Pos(1,i),Pos(1,j)],[Pos(2,i),Pos(2,j)],'c','LineWidth',0.5);
        set(Com(i,j),'Visible','off');
        if NeighborSet(i,j)==1
            set(Com(i,j),'Visible','on');
        end
    end
end
end

% 函数功能:更新集群通信网络
% 输入变量:集群规模,群体位置,邻居集
function UpdateComNetwork(Com,Flock_size,Pos,NeighborSet)
for i=1:Flock_size-1
    for j=i+1:Flock_size
                                                                     set(Com(i,j),'Visible','off');
        if NeighborSet(i,j)==1     
            set(Com(i,j),'Visible','on');
            set(Com(i,j),'XData',[Pos(1,i),Pos(1,j)],'YData',[Pos(2,i),Pos(2,j)]); 
        end
    end
end
end

% 函数功能:绘制圆形障碍
% 输入变量:威胁数组
function DrawCircleObstacles(Obs_set)
Num=length(Obs_set(1,:));
t=0:0.1:2*1.1*pi;
for i=1:Num
    x2=Obs_set(1,i)+Obs_set(3,i)*sin(t);
    y2=Obs_set(2,i)+Obs_set(3,i)*cos(t);
    plot(x2,y2,'color',[1,0.5,0],'LineWidth',0.5);
end
end




