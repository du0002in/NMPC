function conjugate_abc=conjugate_lane(abc,z_range,leftrightlane,lane_width)
if leftrightlane==1
    conjugate_abc=abc;
    return;
end
z=z_range(1):(z_range(2)-z_range(1))/20:z_range(2);
x=abc(1)*z.^2+abc(2)*z+abc(3);
slope_x=2*abc(1)*z+abc(2);
theta=atan(slope_x);
if leftrightlane==0
    x2=x-lane_width*cos(theta);
    z2=z+lane_width*sin(theta);
elseif leftrightlane==2
    x2=x+lane_width*cos(theta);
    z2=z-lane_width*sin(theta);
end
Atmp=[z2.^2; z2; ones(1,length(z2))]';
Btmp=x2';
conjugate_abc=pinv(Atmp)*Btmp;
