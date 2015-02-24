%%
% get_codebook.m
% Bhavya 
% Fetches circle data and generates codebook and discretised data
%
%% Config visualisaiton:
PLOT_DISCRETE = true;
PLOT_CONT = true;
PLOT_FILT = false;

%% Read file
source_name = ['../Data/train/circle/circle_train.txt'];
target_name_codebook = ['../Data/train/circle/circle_codebook.mat'];
target_name_data = ['../Data/train/circle/circle_train_discrete.mat'];
all_data = textread(source_name);

%% Get accel and gyro data
gyro_raw = all_data(:,5:end);
accel_raw= all_data(:,2:4);

%% Low pass Filter each channel
gyro_filter_wt = 0.3;
accel_filter_wt  = 0.15;

gyro_filtered = gyro_raw(1,:);
accel_filtered = accel_raw(1,:);

for i = 2:length(gyro_raw)
   gyro_filtered(i,:) = gyro_filter_wt * gyro_raw(i,:) + (1-gyro_filter_wt) * gyro_filtered(i-1,:);
   accel_filtered(i,:) = accel_filter_wt * accel_raw(i,:) + (1-accel_filter_wt) * accel_filtered(i-1,:);
end

train_data = [accel_filtered,gyro_filtered];


%% Cluster data
NUM_CLUSTERS = 3;
NUM_CV = 10;
[assignments,centroids] = kmeans(train_data,NUM_CLUSTERS,'Display','final','Replicates',NUM_CV);

cluster_indices = 1:NUM_CLUSTERS;


%% Visualise continous data

% plot accel raw vs accel filtered
if PLOT_FILT
figure
subplot(3,1,1);
plot(accel_raw(:,1));
hold on 
plot(accel_filtered(:,1));
ylabel('Accelx')

subplot(3,1,2);
plot(accel_raw(:,2));
hold on 
plot(accel_filtered(:,2));
ylabel('Accely')

subplot(3,1,3);
plot(accel_raw(:,3));
hold on 
plot(accel_filtered(:,3));
ylabel('Accelz')

% plot gyro raw vs gyro actual
figure
title('Gyro')
subplot(3,1,1);
plot(gyro_raw(:,1));
hold on 
plot(gyro_filtered(:,1));
ylabel('gyrox')

subplot(3,1,2);
plot(gyro_raw(:,2));
hold on 
plot(gyro_filtered(:,2));
ylabel('gyroy')

subplot(3,1,3);
plot(gyro_raw(:,3));
hold on 
plot(gyro_filtered(:,3));
ylabel('gyroz')
end
if PLOT_CONT
% visualise data clustering
figure 
grid on
color = zeros(size(gyro_filtered));
for i = 1 :NUM_CLUSTERS
    mask = (i == assignments);
    a = repmat([rand(1,1),rand(1,1),rand(1,1)],length(find(mask)),1);
    b = color(mask,:);
    color(mask,:) = a;

end
scatter3(gyro_filtered(1,1),gyro_filtered(1,2),gyro_filtered(1,3),100);
scatter3(gyro_filtered(:,1),gyro_filtered(:,2),gyro_filtered(:,3),36,color);
title('gyro');
xlabel('x');
ylabel('y');
zlabel('z');


figure
scatter3(accel_filtered(1,1),accel_filtered(1,2),accel_filtered(1,3),100);
scatter3(accel_filtered(:,1),accel_filtered(:,2),accel_filtered(:,3),36,color);
title('accel');
xlabel('x');
ylabel('y');
zlabel('z');
end
% ***  prompt before proceeding *****
%% Discretise data
for i = 1:NUM_CLUSTERS
    mask = i== assignments;
    gyro_filtered(mask,:) = repmat(centroids(i,4:end),length(find(mask)),1);
    accel_filtered(mask,:) = repmat(centroids(i,1:3),length(find(mask)),1);
end

%% Visualise discretised data
if PLOT_DISCRETE
% plot accel raw vs accel filtered
figure

subplot(3,1,1);
plot(accel_raw(:,1));
hold on 
plot(accel_filtered(:,1));
ylabel('Accelx')

subplot(3,1,2);
plot(accel_raw(:,2));
hold on 
plot(accel_filtered(:,2));
ylabel('Accely')

subplot(3,1,3);
plot(accel_raw(:,3));
hold on 
plot(accel_filtered(:,3));
ylabel('Accelz')

% plot gyro raw vs gyro actual
figure
title('Gyro')
subplot(3,1,1);
plot(gyro_raw(:,1));
hold on 
plot(gyro_filtered(:,1));
ylabel('gyrox')

subplot(3,1,2);
plot(gyro_raw(:,2));
hold on 
plot(gyro_filtered(:,2));
ylabel('gyroy')

subplot(3,1,3);
plot(gyro_raw(:,3));
hold on 
plot(gyro_filtered(:,3));
ylabel('gyroz')

% visualise data clustering
figure 
grid on
% color = zeros(size(gyro_filtered));
% for i = 1 :NUM_CLUSTERS
%     mask = (i == assignments);
%     a = repmat([rand(1,1),rand(1,1),rand(1,1)],length(find(mask)),1);
%     b = color(mask,:);
%     color(mask,:) = a;
% 
% end
scatter3(gyro_filtered(1,1),gyro_filtered(1,2),gyro_filtered(1,3),100);
scatter3(gyro_filtered(:,1),gyro_filtered(:,2),gyro_filtered(:,3),36,color);
title('gyro');
xlabel('x');
ylabel('y');
zlabel('z');


figure
scatter3(accel_filtered(1,1),accel_filtered(1,2),accel_filtered(1,3),100);
scatter3(accel_filtered(:,1),accel_filtered(:,2),accel_filtered(:,3),36,color);
title('accel');
xlabel('x');
ylabel('y');
zlabel('z');

end
%% Save cluster centers and discretised data
% clear all -except gyro_filtered accel_filtered centroids target_name_codebook target_name_data
save(target_name_codebook,'centroids');
save(target_name_data,'gyro_filtered','accel_filtered');

