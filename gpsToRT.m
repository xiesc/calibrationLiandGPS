
%from gps RPY and relative posation to RT
%par(:,1:3) = Roll Yaw Pitch
%par(:,4:6) = Lon Lat Ati
function RTMatrix = gpsToRT(par)
    num = size(par,1);
    [x,y] = wgs2local(par(:,4:5));
    z = par(:,6)-par(1,6)*ones(num,1);
   RTMatrix = zeros(num,12);
   R1 = RPYtoR(par(1,1:3));
   R1T = R1';
  for i =1:1:num
     R = R1T*RPYtoR(par(i,1:3));
     RTMatrix(i,:) = [R(1,:),x(i),R(2,:),y(i),R(3,:),z(i)];
  end
  
end


