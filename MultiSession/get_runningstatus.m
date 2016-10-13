function rs_flag=get_runningstatus
ele_no=0;
while(ele_no==0) %in case when the file is being written, it will read in nothing
    fid=fopen('D:\Stereo\MPC\MultiSession\runningstatus.txt','rt');
    C=textscan(fid, '%d', 1);
    ele_no=numel(C{1});
    fclose(fid);
end
rs_flag=C{1}(1);
end