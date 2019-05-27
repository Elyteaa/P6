 %% Input data

data = [427.16 -199.89 138.03;     %Teach pendant values(x,y,z), 1st measurement
        428.57 -203.74 137.30];    %Camera frame values, 1st measurement
    
data(:,:,2)= [369.27 6.71 115.22;  %Teach pendant values, 2nd measurement
           362.69 8.30 111.91]; 
       
data(:,:,3)= [529.69 -223.28 213.37; % etc...
           532.33 -219.51 209.17]; % etc...

data(:,:,4)= [308.39 -178.98 -40.87;
           301.46 -180.24 -38.55];

data(:,:,5)= [391.54 277.12 220.35;
           388.89 278.4 220.47];

data(:,:,6)= [442.28 176.24 183.12; %
           442.08 177.92 184.53];
%% Data Crunch, euclidean distance
offset_raw = zeros([size(data,2),size(data,3)]);
offset_raw(:,:)=[];
for i=1:size(data,2) %Which x, y, or z
    for j=1:size(data,3) %Which measurement
            offset_raw(i,j) = data(2,i,j) - data(1,i,j);
    end
end

offset_eucl = zeros([size(data,3)]);
offset_eucl(:,:)=[];
for i=1:size(data,3)
    offset_eucl(i) = sqrt(offset_raw(1,i)^2 + offset_raw(2,i)^2 + offset_raw(3,i)^2);
end

offset_mean = sum(offset_eucl)/size(offset_eucl,2)

offset_var = var(offset_eucl)

offset_stdv = sqrt(offset_var)

%% Data Crunch, individual axes

axes_mean = sum(offset_raw,2)/size(data,3)

axes_stdv = zeros([size(data,2),1]);
for i=1:size(data,2) % for each x,y,z
    axes_stdv(i) = sqrt(var(offset_raw(i,:)));
end
axes_stdv
