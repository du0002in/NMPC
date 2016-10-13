clear all;
global_varibles;
InitializationGlobalVariable;
load 'cali3.mat';
save_dir='D:\Stereo\MPC\GA with Simple Map\Simulated Img';
load(strcat(save_dir,'\odemotry.mat'));
phi=.3111;D_pre=691*tan(.311);H=1.665;
[Tpix2cam_left, Tpix2cam_right]=Pix2Cam(focal,cc_x_left,cc_x_right,cc_y);
figure;
for i=1:length(act_x)
    [leftimg, rightimg]=stereo_img(-act_x(i),act_z(i),pi-act_theta(i),phi,H,dist_left_right,L_wheel_cam,Tpix2cam_left, Tpix2cam_right);
    leftimg=leftimg*255; rightimg=rightimg*255;
    imwrite(leftimg,strcat(save_dir,'\left',int2str(i),'.jpg'),'jpg');
    imwrite(rightimg,strcat(save_dir,'\right',int2str(i),'.jpg'),'jpg');
    imshow(leftimg,[]); pause(0.001);
end