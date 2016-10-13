function [status,x,z,theta]=getPosition
ele_no=0;
while(ele_no==0) %in case when the file is being written, it will read in nothing
    fid=fopen('D:\Stereo\MPC\MultiSession\location.txt','rt');
    C=textscan(fid, '%d %f %f %f', 1);
    ele_no=numel(C{1});
    fclose(fid);
end
status=C{1}(1);
x=C{2}(1);
z=C{3}(1);
theta=C{4}(1);
