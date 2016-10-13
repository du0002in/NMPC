function updateLocation(status, x, z, theta)
fid=fopen('\Stereo\MPC\MultiSession\location.txt','wt');
formatSpec='%d %4.3f %4.3f %4.3f';
fprintf(fid,formatSpec,status, x,z,theta);
fclose(fid);
end