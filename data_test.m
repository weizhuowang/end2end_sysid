% data process V2
clc,clear;
addpath(genpath('data'))
onbod = readmatrix('20191023-211656.txt'); 
mocapraw = readmatrix('matrice-verification.txt'); mocapraw = mocapraw(:,1:end-1);

% scale = mocap(1000,1:2)-mocap(1,1:2); scale = vpa(scale(1)/scale(2));
% fix time scale
scale = 0.01;
mocap = sortrows(mocapraw(10:end-10,1:end),2);
mocap(:,2) = (mocap(:,2)-min(mocap(:,2)))*scale + mocap(1,1);
mocap = mocap(:,2:end);

mocap_smooth = mocap;
mocap(:,1:end) = smoothdata(mocap(:,1:end),'movmean',40);
% Rel_time | Abs_time | tau | rpm | steering PWM | DutyCycle | 
% Radio_Throttle_PWM | P_motor | 

%% necessary info

st_o = onbod(1,2);
ed_o = onbod(end,2);

tot_time = ed_o-st_o;

usefulidx = and(mocap(:,1)>=st_o,mocap(:,1)<=ed_o);
mocap = mocap(usefulidx,:);

mocap_t = mocap(:,1);
mocap_pos = mocap(:,2:4);
mocap_quat = mocap(:,5:8);


figure(1);clf
for i = 1:3
    subplot(3,1,i);
    plot(mocap_t-st_o,mocap_pos(:,i));grid on; hold on;
    plot(mocap_t-st_o,mocap_smooth(usefulidx,1+i));grid on; hold on;
end

%% derived info
dt = diff(mocap_t);
dposdt = diff(mocap_pos)./dt;
mocap_v = vecnorm(dposdt,2,2);

figure(2);clf;
for i = 1:3
    subplot(3,1,i);
    plot(mocap_t(1:end-1)-st_o,dposdt(:,i));grid on; hold on;
end

% heading = atan2d(dposdt(:,1),dposdt(:,3));
% heading2 = rad2deg(quat2eul(mocap_quat))

%% heading
figure(4);clf;
[r1 r2 r3] = quat2angle(mocap_quat,'ZYX'); % XYZ ZYX very small XZY YXZ YZX ZXY
subplot(3,1,1);
plot(mocap_t-st_o,rad2deg(r1));grid on;hold on;
subplot(3,1,2);
plot(mocap_t-st_o,rad2deg(r2));grid on;hold on;
subplot(3,1,3);
plot(mocap_t-st_o,rad2deg(r3));grid on;hold on;

% figure(3);clf;
% for i = 800:2997
%     fprintf('i=%d\n',i)
%     quiver(0,0,cos(r3(i)),sin(r3(i)));grid on;
%     xlim([-1.5,1.5]);ylim([-1.5,1.5]);%axis equal
%     drawnow
% %     pause(0.0001)
% end
    
% onbod = [0,onbod(1,2:end);onbod];

%% comparison of onboard and mocap
% Compare speed
figure(5);clf; 

mocap_v = vecnorm(dposdt(:,[1,3]),2,2);
onbod_v = onbod(:,4)/60/2/2.6*(pi*0.095);

plot(mocap_t(1:end-1)-st_o,mocap_v);grid on;hold on;
plot(onbod(:,1),(onbod_v)/8);grid on;hold on;
% plot(onbod(:,1),onbod(:,6)/100000*2.4);grid on;hold on;



% Compare heading
r1cd = abs2cont_heading(rad2deg(r1));
r2cd = abs2cont_heading(rad2deg(r2));
r3cd = abs2cont_heading(rad2deg(r3));

figure(6);clf
subplot(3,1,1);
plot(mocap_t-st_o,r1cd);grid on;hold on;
subplot(3,1,2);
plot(mocap_t-st_o,r2cd);grid on;hold on;
subplot(3,1,3);
plot(mocap_t-st_o,r3cd);grid on;hold on;

figure(7);clf
plot(mocap_t(1:end-1)-st_o,diff(r2cd));grid on;hold on;
plot(onbod(:,1),-(onbod(:,5)-1430)/200);grid on;hold on;
xlim([18,60])















function r3d = abs2cont_heading(r3d)

    for i = 2:length(r3d)
        if abs(r3d(i)-r3d(i-1)) > 300
            if r3d(i)>r3d(i-1)
                r3d(i:end) = r3d(i:end)-360;
            else
                r3d(i:end) = r3d(i:end)+360;
            end
        end
    end

end

