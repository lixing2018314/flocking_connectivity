function [pos_set,vel_set] = InitialState(flock_size,r_sens,r_avoid,pos)
  % initialize state variables
    pos_set=zeros(2,flock_size);  
    vel_set=zeros(2,flock_size);
    % initialize variables
    mtr_size=ceil(flock_size/2+1);
    gen_mtr=zeros(mtr_size,mtr_size);
    gen_set=zeros(2,flock_size);
    neigh_mtr=ones(3,3).*[-1,0,1];
    % generate the first state
    cell_pos=[ceil(mtr_size/2),ceil(mtr_size/2)]';      
    gen_mtr(cell_pos(1),cell_pos(2))=1;
    gen_num=1;
    gen_set(:,gen_num)=cell_pos; 
    % generate other states
    while gen_num~=flock_size        
        % select one from the generated cells
        index=randi([1,gen_num],1,1); 
        cell_pos=gen_set(:,index);
        % find valid cells around the focus cell
        neigh_row=ones(3,3).*cell_pos(1)+neigh_mtr';    
        neigh_col=ones(3,3).*cell_pos(2)+neigh_mtr;  
        neigh_row=reshape(neigh_row,1,9);
        neigh_col=reshape(neigh_col,1,9);
        neigh_set=zeros(2,8);
        ungen_count=0;      
        for i=1:1:9
            if isequal([neigh_row(i),neigh_col(i)]',cell_pos)
                continue;
            end
            if neigh_row(i)<1||neigh_row(i)>mtr_size||...
               neigh_col(i)<1||neigh_col(i)>mtr_size
                continue;
            end
            if gen_mtr(neigh_row(i),neigh_col(i))==1
                continue;
            end
            ungen_count=ungen_count+1;
            neigh_set(:,ungen_count)=[neigh_row(i),neigh_col(i)]';
        end        
        % abandon the cell with many valid neighbors
        if ungen_count<3
            continue;
        end
        % choose a suitable cell
        suit_set=zeros(2,ungen_count);
        suit_count=0;
        for i=1:1:ungen_count
            cell_pos=neigh_set(:,i);   
            neigh_row=ones(3,3).*cell_pos(1)+neigh_mtr';    
            neigh_col=ones(3,3).*cell_pos(2)+neigh_mtr;
            neigh_row=reshape(neigh_row,1,9);
            neigh_col=reshape(neigh_col,1,9);
            gened_count=0;
            for j=1:1:9
                if neigh_row(j)<1||neigh_row(j)>mtr_size||...
                   neigh_col(j)<1||neigh_col(j)>mtr_size
                    continue;
                end
                if gen_mtr(neigh_row(j),neigh_col(j))==1
                    gened_count=gened_count+1;
                end
            end
            if gened_count>=3
                continue;
            end
            suit_count=suit_count+1;
            suit_set(:,suit_count)=cell_pos;
        end
        if suit_count==0
            continue;
        end        
        select=randi([1,suit_count],1,1);  
        cell_pos=suit_set(:,select);
        gen_mtr(suit_set(1),suit_set(2))=1;
        gen_num=gen_num+1;
        gen_set(:,gen_num)=cell_pos;    
    end    
    % generate initial positions
    grid_lgth=r_sens/1.5/sqrt(2);
    grid_valid=grid_lgth-r_avoid/2;
    count=0;
    for i=1:1:mtr_size    
        for j=1:1:mtr_size
            if gen_mtr(i,j)==0
                continue;
            end
            count=count+1;
            temp=[i,j]'-gen_set(:,1);
            temp_minx=temp(1)*grid_lgth-grid_valid/2; temp_maxx=temp(1)*grid_lgth+grid_valid/2;
            temp_miny=-temp(2)*grid_lgth-grid_valid/2; temp_maxy=-temp(2)*grid_lgth+grid_valid/2;
            pos_set(1,count)=temp_minx+(temp_maxx-temp_minx)*rand(1,1);
            pos_set(2,count)=temp_miny+(temp_maxy-temp_miny)*rand(1,1);
        end
    end
    pos_set=pos_set+pos;
    % generate initial velocities
%     vel_scale=rand(1,param.swarm_size).*param.vel_max;
%     heading=rand(1,param.swarm_size)*2*pi;
%     vel_set=[vel_scale.*cos(heading);vel_scale.*sin(heading)];
    % assignment output states



end