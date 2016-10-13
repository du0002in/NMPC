function changeacturalpose(i)
fid=fopen('D:\Stereo\MPC\GA Solver\GASim\GA_ActualPosition_log.txt','wt');
formatSpec='%f %f %f %f %f %f %f %f %f %f';
fprintf(fid,formatSpec,i);
fclose(fid);
end