function [x, z, theta, phi, xc, t, v]=GA_getActPose
ele_no=0;
while (ele_no==0)
    fid=fopen('D:\Stereo\MPC\GA Solver\GASim\GA_ActualPosition_log.txt','rt');
    C=textscan(fid,'%f %f %f %f %f %f %f %f %f %f',1);
    ele_no=numel(C{1});
    fclose(fid);
end
t=C{1};
x=C{2};
z=C{3};
theta=C{4};
phi=C{5};
xc=C{7};
d_theta=C{8};
v=C{6};
end