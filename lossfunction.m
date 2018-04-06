%% calculation the loss in optimase the RT between gps and lidar
%se(1:3) = roll yaw pitch
%se(4:6) = tx ty tz
function loss = lossfunction (se,gpsRTMatrix_syned,slamRTMatrix_syned)
       R= RPYtoR(se(1:3));
       t= [se(4);se(5);se(6)];
       RTl2g = [R , t;
            0 0 0 ,1];
       RTg2l = [R',-R'*t;
            0 0 0 , 1];
        num = size(gpsRTMatrix_syned,1);
        loss = 0;
        for i =1 :1 :num
            RTg = [gpsRTMatrix_syned(i,1:4);gpsRTMatrix_syned(i,5:8);
                    gpsRTMatrix_syned(i,9:12);0,0,0,1];
            RTl = [slamRTMatrix_syned(i,1:4);slamRTMatrix_syned(i,5:8);
                    slamRTMatrix_syned(i,9:12);0,0,0,1];
            loss = loss + norm(RTl\RTg2l*RTg*RTl2g-eye(4),2);
            
        end
        loss = loss/num;




end