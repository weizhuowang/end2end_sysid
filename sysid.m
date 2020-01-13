clc,clear;
% MOCAP TODO:
% 1. Re-calibrate mocap every time before use
% 2. Resolve the backward time issue (might be from receiving queue)
% 3. Check where is the time stamp generated and if the mocap can send
%    frame time stamp

% Read data
onbod = readmatrix('data/20191023-211656.txt'); onbod = [0,onbod(1,2:end);onbod];
mocap = readmatrix('data/matrice-verification.txt'); mocap = mocap(:,1:end-1);

% Start time
onbod(1,2)

% extract mocap data for same interval as onboard log
mocap_useful = mocap(mocap(:,1)>onbod(1,2)-0.1 & mocap(:,1)<onbod(end,2),:);

% Match the onboard data with mocap data
for i = 1:size(mocap_useful,1)
    cur_t = mocap_useful(i,1);
    idx = find(onbod(:,2)>=cur_t,1,'first');
    onbod_useful(i,:) = onbod(idx,:);
end

%% Plot ground track from mocap data
figure(1);clf
plot3(mocap_useful(:,3),mocap_useful(:,5),mocap_useful(:,4));grid on;
axis equal

% transfer to euler if needed later
mocap_q = quaternion(mocap_useful(:,6:9));
mocap_eul = eulerd(mocap_q,'ZYX','frame');
mocap_eul(:,1) = (mocap_eul(:,1)+180);
% mocap_eul(:,3) = mod(mocap_eul(:,3)+180,180);

% euler plot seems weird so dont use for now
figure(2);clf
plot(1:size(mocap_eul,1),mocap_eul(:,1));grid on;hold on;
plot(1:size(mocap_eul,1), mocap_eul(:,2));hold on;
plot(1:size(mocap_eul,1), mocap_eul(:,3));hold on;

%%
% quaternion plot
figure(3)
plot(1:size(mocap_useful,1), mocap_useful(:,6));grid on;hold on;
plot(1:size(mocap_useful,1), mocap_useful(:,7));hold on;
plot(1:size(mocap_useful,1), mocap_useful(:,8));hold on;
plot(1:size(mocap_useful,1), mocap_useful(:,9));hold on;
legend("6","7","8","9")

% Visualize mocap data 
figure(4);clf
mocap_useful(:,4) = mocap_useful(:,4)-2; % make it above ground
Mocap_visualize(onbod_useful(:,1)', mocap_useful(:,[3,5,4])',...
                mocap_useful(:,[6,7,8,9]), []) % Visualize directly from quaternion

% Time and packet number plot, problem here!
figure(5);clf
plot(mocap_useful(:,2),mocap_useful(:,1));hold on;grid on;
% plot(1:2962,onbod(:,1));grid on;hold on;
% plot(1:2962,onbod(:,2)-onbod(1,2));hold on;

%%
yawrate = [0];
for i = 2:size(mocap,1)
    yawrate(i) = mocap_useful()
end



%%
systemIdentification