%% ��ת����ת��ŷ����
%%��ý��Ϊŷ����̬�ǣ���ÿ����ת����������ϵ��ת
%%�����Ϊthtx thty thtz����ת˳��Ϊzyx(������)
%%������x��תӦ��[-pi/2,pi/2]
function  tht= rotationMatrixToRPY(R)

    
if abs(R(3,1)) ~= 1
	theta1 = -asin(R(3,1));
	theta2 = pi - theta1;
	psi1 = atan2(R(3,2)/cos(theta1), R(3,3)/cos(theta1));
	psi2 = atan2(R(3,2)/cos(theta2), R(3,3)/cos(theta2));
	pfi1 = atan2(R(2,1)/cos(theta1), R(1,1)/cos(theta1));
	pfi2 = atan2(R(2,1)/cos(theta2), R(1,1)/cos(theta2));
	thty = theta1; % could be any one of the two
	thtx = psi1;
	thtz = pfi1;
else
	thtz = 0;
	
	if R(3,1) == -1
	thty = pi/2;
	thtx = thty + atan2(R(1,2), R(1,3));
    else
	thty = -pi/2;
	thtx = -thty + atan2(-R(1,2), -R(1,3));
	end
end
tht = [thtx,thty,thtz];


end