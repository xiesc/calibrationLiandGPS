



%% gps data processing leador
% gpsMatrix =  load('C:\Users\OpenICV\Desktop\matlab work space\calibration_lidar_gps\2018-02-31-17-12-53.txt');

% %set the initial position and translating the WGS to location
% [gpsMatrix(:,5), gpsMatrix(:,6) ] = wgs2local(gpsMatrix(:,5), gpsMatrix(:,6));
% gpsMatrix(:,7) =gpsMatrix(:,7) - ones(size(gpsMatrix,1),1)*gpsMatrix(1,7);
% 
% location_gps  = gpsMatrix(:,5:7);
% 
% % location_gps(:,1) =location_gps(:,1) - ones(size(location_gps,1),1)*location_gps(1,1);
% % location_gps(:,2) =location_gps(:,2) - ones(size(location_gps,1),1)*location_gps(1,2);
% % location_gps(:,3) =location_gps(:,3) - ones(size(location_gps,1),1)*location_gps(1,3);
% RPT_gps = gpsMatrix(:,2:4);
 
% gpsMatrix(:,2:4) = gpsMatrix(:,2:4)./360*pi;
% gpsRTMatrix = gpsToRT(gpsMatrix(:,2:end));
% time_gps = gpsMatrix(:,1);
%% gps data processing oxford
gps =  load('/home/xiesc/xieshichao/matlab work space/calibration_lidar_gps/calibration_sick_gps/2018-04-02-23-16-56gps.txt');
imu =  load('/home/xiesc/xieshichao/matlab work space/calibration_lidar_gps/calibration_sick_gps/2018-04-02-23-16-56imu.txt');
[time_gps,gpsRTMatrix] = gpsToRT_oxford(gps,imu);



%% slam data processing
slamMatrix = load('C:\Users\OpenICV\Desktop\pose.txt');

% %%%%%%%%%%%%%% rigid translation if slam pose is calculated by loam %%%%%%%
% tmpMatrix(:,1) = slamMatrix(:,1);
% tmpMatrix(:,2) = slamMatrix(:,4);
% tmpMatrix(:,3) = slamMatrix(:,2);
% tmpMatrix(:,4) = slamMatrix(:,3);
% tmpMatrix(:,5) = slamMatrix(:,7);
% tmpMatrix(:,6) = slamMatrix(:,5);
% tmpMatrix(:,7) = slamMatrix(:,6);
% 
% slamMatrix = tmpMatrix;
% location_slam = slamMatrix(:,5:7);
% %%%%%%%%%%%%%%

slamRTMatrix = slamMatrix(:,2:end);
time_slam = slamMatrix(:,1);

%% test plot1
plot3 (gpsMatrix(:,4),gpsMatrix(:,8),gpsMatrix(:,12));
hold on
plot3 (slamRTMatrix(:,4),slamRTMatrix(:,8),slamRTMatrix(:,12));
grid on
xlabel('x/m')
ylabel('y/m')
zlabel('z/m')
axis equal

%% synchronic
index_valid = zeros(length(time_slam),1);
index = zeros(length(time_slam),1);
for i=1:1:length(time_slam)
    [tmp,I]=min(abs(time_gps(:)-time_slam(i)));
    if tmp<0.005
        index_valid(i)=1;
        index(i)=I;
    end
    
end


gpsRTMatrix_syned = gpsRTMatrix(index(index>0),:);
slamRTMatrix_syned = slamRTMatrix(index_valid == 1,:);

%% test plot2
plot3 (gpsRTMatrix_syned(:,4),gpsRTMatrix_syned(:,8),gpsRTMatrix_syned(:,12));
hold on
plot3 (slamRTMatrix_syned(:,4),slamRTMatrix_syned(:,8),slamRTMatrix_syned(:,12));
grid on
xlabel('x/m')
ylabel('y/m')
zlabel('z/m')
axis equal

%% calibration
%se(1:3) = roll yaw pitch
%se(4:6) = tx ty tz
min_fun=@(se)lossfunction(se,gpsRTMatrix_syned,slamRTMatrix_syned);
se_cal = fminsearch(min_fun,zeros(1,6),optimset('MaxFunEvals',40000,'MaxIter',40000,...
                            'Algorithm','levenberg-marquardt','ToLX',1e-6,'Display','iter'));%

%% evaluation
R= RPYtoR(se_cal(1:3));
t= [se_cal(4);se_cal(5);se_cal(6)];
RTl2g = [R , t;
    0 0 0 ,1];
for i =1:1:num
    RTl = [slamRTMatrix_syned(i,1:4);slamRTMatrix_syned(i,5:8);
                    slamRTMatrix_syned(i,9:12);0,0,0,1];
    RTltrans = RTl2g*RTl;
    slamRTMatrix_syned_trans(i,:) =  [RTltrans(1,:),RTltrans(2,:),RTltrans(3,:)];
    
end


%% test plot2
plot3 (gpsRTMatrix_syned(:,4),gpsRTMatrix_syned(:,8),gpsRTMatrix_syned(:,12));
hold on
plot3 (slamRTMatrix_syned_trans(:,4),slamRTMatrix_syned_trans(:,8),slamRTMatrix_syned_trans(:,12));
grid on
xlabel('x/m')
ylabel('y/m')
zlabel('z/m')
axis equal






