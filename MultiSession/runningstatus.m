function runningstatus(i)
fid=fopen('D:\Stereo\MPC\MultiSession\runningstatus.txt','wt');
formatSpec='%d';
fprintf(fid,formatSpec,i);
fclose(fid);
end