
%from gps RPY and relative posation to RT
%imu(:,2:6) = qua(x,y,z,w)
%gps(:,2:5) = Lon Lat Ati
% **(:,1) = timestemp
function [time_gps,RTMatrix] = gpsToRT_oxford(gps,imu)
    num1 = size(gps,1);
    num2 = size(imu,1);
    if (abs(num1-num2)>2)
         error('gps and imu are not syned');
    end
    if (num1>num2)
        gps(num2+1:end,:)=[];
        num = num2;
    end
    if (num2>num1)
        imu(num1+1:end,:)=[];
        num = num1;
    end
    
    par=[imu,gps(:,2:end)];
    
    [x,y] = wgs2local(par(:,6:7));
    z = par(:,8)-par(1,8)*ones(num,1);
    RTMatrix = zeros(num,12);
    R1=quat2dcm([par(1,5),par(1,2:4)]);
   R1T = R1';
  for i =1:1:num
     R = R1T*quat2dcm([par(i,5),par(i,2:4)]);
     RTMatrix(i,:) = [R(1,:),x(i),R(2,:),y(i),R(3,:),z(i)];
  end
  time_gps = par(:,1);
end


