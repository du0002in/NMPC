lane_width=3.0;
% % L1=50;
% % R2=40; Theta2=pi/2;
% % L3=400;
% % R4=40; Theta4=2*pi/4;
% % R5=40; Theta5=5*pi/4;
% % interval=2.0;

L1=9;
R2=8; Theta2=pi/4;
L3=77;
R4=35; Theta4=40/180*pi;
L5=10;
R6=25; Theta6=80/180*pi;
L7=80;
interval=2;

map_z1=1:interval:L1;
map_x1=-lane_width/2*ones(1,length(map_z1));

x0=map_x1(end)+R2;
z0=map_z1(end);
theta=0:3.795/sqrt(R2)/16:Theta2;
map_x2=x0-R2*cos(theta);
map_z2=z0+R2*sin(theta);

x0=x0-R2*cos(Theta2);
z0=z0+R2*sin(Theta2);
t=0:interval:L3;
map_x3=x0+cos(pi/2-Theta2)*t;
map_z3=z0+sin(pi/2-Theta2)*t;

x0=x0+cos(pi/2-Theta2)*L3;
z0=z0+sin(pi/2-Theta2)*L3;
x0=x0-R4*cos(Theta2);
z0=z0+R4*sin(Theta2);
theta=Theta2:-3.795/sqrt(R4)/16:(Theta2-Theta4);
map_x4=x0+R4*cos(theta);
map_z4=z0-R4*sin(theta);

x0=x0+R4*cos(Theta2-Theta4);
z0=z0-R4*sin(Theta2-Theta4);
t=0:interval:L5;
map_x5=x0+sin(Theta2-Theta4)*t;
map_z5=z0+cos(Theta2-Theta4)*t;

x0=x0+sin(Theta2-Theta4)*L5;
z0=z0+cos(Theta2-Theta4)*L5;
x0=x0+R6*cos(Theta2-Theta4);
z0=z0-R6*sin(Theta2-Theta4);
theta=(Theta2-Theta4):3.795/sqrt(R6)/16:(Theta2-Theta4+Theta6);
map_x6=x0-R6*cos(theta);
map_z6=z0+R6*sin(theta);

x0=x0-R6*cos(Theta2-Theta4+Theta6);
z0=z0+R6*sin(Theta2-Theta4+Theta6);
t=0:interval:L7;
map_x7=x0+sin(Theta2-Theta4+Theta6)*t;
map_z7=z0+cos(Theta2-Theta4+Theta6)*t;

map_x=[map_x1 map_x2 map_x3 map_x4 map_x5 map_x6 map_x7];
map_z=[map_z1 map_z2 map_z3 map_z4 map_z5 map_z6 map_z7];

R2=R2-lane_width;
R4=R4+lane_width;
R6=R6-lane_width;
% interval=2.0;
map_z1=1:interval:L1;
map_x1=lane_width/2*ones(1,length(map_z1));

x0=map_x1(end)+R2;
z0=map_z1(end);
theta=0:Theta2/length(map_z2):Theta2;
if length(theta)~=length(map_z2)
    theta=theta(1:end-1);
end
map_x2=x0-R2*cos(theta);
map_z2=z0+R2*sin(theta);

x0=x0-R2*cos(Theta2);
z0=z0+R2*sin(Theta2);
t=0:interval:L3;
map_x3=x0+cos(pi/2-Theta2)*t;
map_z3=z0+sin(pi/2-Theta2)*t;

x0=x0+cos(pi/2-Theta2)*L3;
z0=z0+sin(pi/2-Theta2)*L3;
x0=x0-R4*cos(Theta2);
z0=z0+R4*sin(Theta2);
theta=Theta2:-(Theta4/length(map_z4)):(Theta2-Theta4);
if length(theta)~=length(map_z4)
    theta=theta(1:end-1);
end
map_x4=x0+R4*cos(theta);
map_z4=z0-R4*sin(theta);

x0=x0+R4*cos(Theta2-Theta4);
z0=z0-R4*sin(Theta2-Theta4);
t=0:interval:L5;
map_x5=x0+sin(Theta2-Theta4)*t;
map_z5=z0+cos(Theta2-Theta4)*t;

x0=x0+sin(Theta2-Theta4)*L5;
z0=z0+cos(Theta2-Theta4)*L5;
x0=x0+R6*cos(Theta2-Theta4);
z0=z0-R6*sin(Theta2-Theta4);
theta=(Theta2-Theta4):(Theta6/length(map_z6)):(Theta2-Theta4+Theta6);
if length(theta)~=length(map_z6)
    theta=theta(1:end-1);
end
map_x6=x0-R6*cos(theta);
map_z6=z0+R6*sin(theta);

x0=x0-R6*cos(Theta2-Theta4+Theta6);
z0=z0+R6*sin(Theta2-Theta4+Theta6);
t=0:interval:L7;
map_x7=x0+sin(Theta2-Theta4+Theta6)*t;
map_z7=z0+cos(Theta2-Theta4+Theta6)*t;

map_x_bar=[map_x1 map_x2 map_x3 map_x4 map_x5 map_x6 map_x7];
map_z_bar=[map_z1 map_z2 map_z3 map_z4 map_z5 map_z6 map_z7];

ctr_x=0.5*(map_x+map_x_bar);
ctr_z=0.5*(map_z+map_z_bar);
map_x_bar=fliplr(map_x_bar);
map_z_bar=fliplr(map_z_bar);

dist=sqrt((ctr_x(2:end)-ctr_x(1:end-1)).^2+(ctr_z(2:end)-ctr_z(1:end-1)).^2);

figure;patch(-[map_x map_x_bar],[map_z map_z_bar],'g');
hold on;plot(-ctr_x,ctr_z,'k--','LineWidth',0.5);
axis equal;