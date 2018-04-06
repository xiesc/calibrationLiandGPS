function [ x, y ] = wgs2local(lonlat)
% ��γ��ת��Ϊ������
lon = lonlat(:,1);
lat = lonlat(:,2);
lon_b = lon(1);
lat_b = lat(1);
% scale = length_scale([lon_b,lat_b] / 1e6, 'WGS'); 
scale = length_scale([lon_b,lat_b] , 'WGS'); 
x = (lon - lon_b) * scale(1);
y = (lat - lat_b) * scale(2);
end