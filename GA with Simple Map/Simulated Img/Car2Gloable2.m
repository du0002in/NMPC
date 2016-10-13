%take the vehicle global coordinate and the vehicle orintation w.r.t road tangent. Return
%the transformation matrix from car to global
%[x_car;y_car;z_car;1]=Tcar2g*[x_g;y_g;z_g;1];
function Tcar2g=Car2Gloable2(X,Y,theta)

%rotate vehicle cordinate by -(theta+pi/2) ard its y axis
R1=[cos(-theta-pi/2) 0 sin(-theta-pi/2); 0 1 0; ...
    -sin(-theta-pi/2) 0 cos(-theta-pi/2)];
%rotate the new coordinate along its x axis by -pi/2
R2=[1 0 0;0 cos(-pi/2) -sin(-pi/2);0 sin(-pi/2) cos(-pi/2)];
Tcar2g=[R1*R2 R1*R2*(-[X;Y;0]);0 0 0 1];

% 
% % load 'map_cordinate4_dotted';
% map_z=zeros(size(map_x));
% simulated_lane_width=3.5;
% ind_x=((map_x>=(X-2*simulated_lane_width))&(map_x<=(X+simulated_lane_width*2)));
% ind_y=((map_y>=(Y-2*simulated_lane_width))&(map_y<=(Y+simulated_lane_width*2)));
% ind_xy=(ind_x)&(ind_y);
% ind_xy=find(ind_xy==1);
% map_xtmp=map_x(ind_xy);
% map_ytmp=map_y(ind_xy);
% [IDX,xc]=knnsearch([map_xtmp map_ytmp],[X,Y]);
% IDX=ind_xy(IDX);
% % [IDX,xc]=knnsearch([map_x' map_y'],[X,Y]);toc
% X_centerline=map_x(IDX);
% Y_centerline=map_y(IDX);
% Z_centerline=map_z(IDX);
% Z=map_z(IDX);
% %slope (angle) at point (X,Y,Z);
% if IDX>=2 && IDX<=(length(map_x)-1)
%     gz=atan((map_z(IDX+1)-map_z(IDX-1))/sqrt((map_x(IDX+1)-map_x(IDX-1))^2+(map_y(IDX+1)-map_y(IDX-1))^2));
%     theta_d=atan2(map_y(IDX+1)-map_y(IDX-1),map_x(IDX+1)-map_x(IDX-1));
% elseif IDX==1
%     gz=atan((map_z(IDX+1)-map_z(IDX))/sqrt((map_x(IDX+1)-map_x(IDX))^2+(map_y(IDX+1)-map_y(IDX))^2));
%     theta_d=atan2(map_y(IDX+1)-map_y(IDX),map_x(IDX+1)-map_x(IDX));
% else
%     gz=atan((map_z(1)-map_z(IDX-1))/sqrt((map_x(1)-map_x(IDX-1))^2+(map_y(1)-map_y(IDX-1))^2));
%     theta_d=atan2(map_y(1)-map_y(IDX-1),map_x(1)-map_x(IDX-1));
% end
% theta_er=theta-theta_d;
% 
% %rotate vehicle cordinate by -theta_er ard its y axis
% R1=[cos(-theta_er) 0 sin(-theta_er);0 1 0;-sin(-theta_er) 0 cos(-theta_er)];
% %translate the new coordinate along its -x axis by xc and -y axis by Z
% vector1=[X_centerline-X, Y_centerline-Y, 0]; %vector from the closest path point to the vehicle position
% vector2=[1*cos(theta), 1*sin(theta), 0]; %vector representing vehicle moving direction
% vector3=cross(vector1, vector2);
% if vector3(3)>=0 %vehicle is at its echo lane
%     T2=[-xc;-Z;0];
% else %vehicle offset to the right lane
%     T2=[xc;-Z;0];
% end
% % if (X_centerline^2+Y_centerline^2)>=(X^2+Y^2) %vehicle is at its echo lane
% %     T2=[-xc;-Z;0];
% % else %vehicle offset to the right lane
% %     T2=[xc;-Z;0];
% % end
% %rotate the new coordinate along its x axis about gz
% R3=[1 0 0; 0 cos(gz) -sin(gz); 0 sin(gz) cos(gz)];
% %rotate the new coordinate along its y axis by -theta_d
% R4=[cos(-(theta_d+pi/2)) 0 sin(-(theta_d+pi/2));0 1 0;-sin(-(theta_d+pi/2)) 0 cos(-(theta_d+pi/2))];
% %rotate the new coordinate along its x axis by -pi/2
% R5=[1 0 0;0 cos(-pi/2) -sin(-pi/2);0 sin(-pi/2) cos(-pi/2)];
% %translate the new coordinate to global coordinate
% T6=[-X_centerline; -Y_centerline; 0];
% 
% Tcar2g=[R1 R1*T2;0 0 0 1]*[R3*R4*R5 R3*R4*R5*T6;0 0 0 1];