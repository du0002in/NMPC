function closecommunication(i)
fid=fopen('D:\Stereo\MPC\GA Solver\GASim\GATwoComs\closecommunication.txt','wt');
formatSpec='%d';
fprintf(fid,formatSpec,i);
fclose(fid);
end