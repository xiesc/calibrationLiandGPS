function [ scale ] = length_scale(pos, type)
%������������ת��Ϊƽ�����
if type == 'WGS'
    a = 6378137.0;
    f = 298.2572;               % �˴�fΪ��ƽ�ʵĵ���
elseif type == 'BJg'
    a = 6378245.0;
    f = 298.3;
end
b = (f - 1) / f * a;
e2 = (a^2 - b^2) / a^2;
A = a * (1 - e2) / (1 - e2 * sin(pos(2) / 180 * pi)^2)^1.5;
B = a * cos(pos(2) / 180 * pi) / sqrt(1 - e2 * sin(pos(2) / 180 * pi)^2);
scale = [B A] * 1 / 180 * pi;
% scale = [B A] * 1 / 180 * pi / 1000000;
end

