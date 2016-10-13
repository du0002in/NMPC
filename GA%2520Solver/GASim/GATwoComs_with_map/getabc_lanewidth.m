function [abc1,abc2,z_th, lane_width]=getabc_lanewidth
ele_no=0;
while(ele_no==0) %in case when the file is being written, it will read in nothing
    fid=fopen('D:\Stereo\MPC\GA Solver\GASim\GATwoComs_with_map\abc.txt','rt');
    C=textscan(fid, '%f %f %f %f %f %f %f %f', 1);
    ele_no=numel(C{1});
    fclose(fid);
end
abc1=[C{1} C{2} C{3}];
abc2=[C{4} C{5} C{6}];
z_th=C{7};
lane_width=C{8};