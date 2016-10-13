function [loop_t, loop_phi, loop_v]=getloop_t_phi_v
ele_no=0;
while(ele_no==0) %in case when the file is being written, it will read in nothing
    fid=fopen('D:\Stereo\t_phi_v_m.txt','rt');
    C=textscan(fid, '%f %f %f');
    ele_no=numel(C{1});
    fclose(fid);
end
loop_t=C{1};
loop_phi=C{2};
loop_v=C{3};