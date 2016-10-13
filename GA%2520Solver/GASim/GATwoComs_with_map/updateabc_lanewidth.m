function updateabc_lanewidth(abc1,abc2,z_th,lane_width)
fid=fopen('\Stereo\MPC\GA Solver\GASim\GATwoComs_with_map\abc.txt','wt');
formatSpec='%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f %4.3f %4.3f';
fprintf(fid,formatSpec,abc1(1),abc1(2),abc1(3),abc2(1),abc2(2),abc2(3),z_th,lane_width);
fclose(fid);
end