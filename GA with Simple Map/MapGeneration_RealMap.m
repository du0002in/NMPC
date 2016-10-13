load('D:\Stereo\MapSimulation\map_cordinate4_full.mat');
lane_width=3.5;
map_x=-map_x(1:7:end);
map_z=map_y(1:7:end);

map_x_bar=-Lane_marking_point_x(1:7:end,5)';
map_z_bar=Lane_marking_point_y(1:7:end,5)';

ctr_x=0.5*(map_x+map_x_bar);
ctr_z=0.5*(map_z+map_z_bar);
map_x_bar=fliplr(map_x_bar);
map_z_bar=fliplr(map_z_bar);

figure;patch(-[map_x map_x_bar],[map_z map_z_bar],'g');
hold on;plot(-ctr_x,ctr_z,'k--','LineWidth',0.5);
axis equal;

dist=sqrt((ctr_x(2:end)-ctr_x(1:end-1)).^2+(ctr_z(2:end)-ctr_z(1:end-1)).^2);

clear Lane* map_y;