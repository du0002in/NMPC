function IDX=get_nearest_map_point(X,Z,map_x,map_z)
simulated_lane_width=3.5;
ind_x=((map_x>=(X-2*simulated_lane_width))&(map_x<=(X+simulated_lane_width*2)));
ind_z=((map_z>=(Z-2*simulated_lane_width))&(map_z<=(Z+simulated_lane_width*2)));
ind_xz=(ind_x)&(ind_z);
ind_xz=find(ind_xz==1);
map_xtmp=map_x(ind_xz);
map_ztmp=map_z(ind_xz);
[IDX,~]=knnsearch([map_xtmp' map_ztmp'],[X,Z]);
IDX=ind_xz(IDX);
while isempty(IDX)
    simulated_lane_width=2*simulated_lane_width;
    ind_x=((map_x>=(X-2*simulated_lane_width))&(map_x<=(X+simulated_lane_width*2)));
    ind_z=((map_z>=(Z-2*simulated_lane_width))&(map_z<=(Z+simulated_lane_width*2)));
    ind_xz=(ind_x)&(ind_z);
    ind_xz=find(ind_xz==1);
    map_xtmp=map_x(ind_xz);
    map_ztmp=map_z(ind_xz);
    [IDX,~]=knnsearch([map_xtmp' map_ztmp'],[X,Z]);
    IDX=ind_xz(IDX);
end
end