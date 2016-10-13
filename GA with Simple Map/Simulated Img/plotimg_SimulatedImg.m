leftimg3d=zeros(ImgSize1/2,ImgSize2,3);
leftimg3d(:,:,1)=leftimg(1:2:ImgSize1,:);
leftimg3d(:,:,2)=leftimg(1:2:ImgSize1,:);
leftimg3d(:,:,3)=leftimg(1:2:ImgSize1,:);
leftimg3d=uint8(leftimg3d);
xx=-CorepxL+cc_x_left;
if leftrightlane==0
    yy=(sel_ytmpr+1)/2;
else
    yy=(sel_ytmpl+1)/2;
end
ind=uint64(sub2ind([ImgSize1/2, ImgSize2],-yy,xx));
ind2=uint64(sub2ind([ImgSize1/2, ImgSize2],[-ytmpl -ytmpl2],round([xtmpl xtmpl2])));
% ind3=sub2ind([ImgSize1/2, ImgSize2],-tmpvl,tmpul);
leftimg3d(ind2)=255;
leftimg3d(ind2+ImgSize1/2*ImgSize2)=0;
leftimg3d(ind2+ImgSize1/2*ImgSize2*2)=0;
% leftimg3d(ind3)=0;
% leftimg3d(ind3+ImgSize1/2*ImgSize2)=255;
% leftimg3d(ind3+ImgSize1/2*ImgSize2*2)=0;
leftimg3d(ind)=0;
leftimg3d(ind+ImgSize1/2*ImgSize2)=0;
leftimg3d(ind+ImgSize1/2*ImgSize2*2)=0;
% imwrite(leftimg3d,strcat('D:\Stereo\Image Collection\FresultL',int2str(ii),'.jpg'),'jpg');
% h=subplot(2,1,1);imshow(leftimg3d,[]);

rightimg3d=zeros(ImgSize1/2,ImgSize2,3);
rightimg3d(:,:,1)=rightimg(1:2:ImgSize1,:);
rightimg3d(:,:,2)=rightimg(1:2:ImgSize1,:);
rightimg3d(:,:,3)=rightimg(1:2:ImgSize1,:);
rightimg3d=uint8(rightimg3d);
xx=-CorepxR+cc_x_right;
if leftrightlane==0
    yy=(sel_ytmpr+1)/2;
else
    yy=(sel_ytmpl+1)/2;
end
ind=uint64(sub2ind([ImgSize1/2, ImgSize2],-yy,xx));
ind2=uint64(sub2ind([ImgSize1/2, ImgSize2],[-ytmpr -ytmpr2],round([xtmpr xtmpr2])));
% ind3=sub2ind([ImgSize1/2, ImgSize2],-tmpvr,tmpur);
rightimg3d(ind2)=255;
rightimg3d(ind2+ImgSize1/2*ImgSize2)=0;
rightimg3d(ind2+ImgSize1/2*ImgSize2*2)=0;
% rightimg3d(ind3)=0;
% rightimg3d(ind3+ImgSize1/2*ImgSize2)=255;
% rightimg3d(ind3+ImgSize1/2*ImgSize2*2)=0;
rightimg3d(ind)=0;
rightimg3d(ind+ImgSize1/2*ImgSize2)=0;
rightimg3d(ind+ImgSize1/2*ImgSize2*2)=0;
% imwrite(leftimg3d,strcat('D:\Stereo\Image Collection\FresultR',int2str(ii),'.jpg'),'jpg');
% subplot(2,1,2);imshow(rightimg3d,[]);
% saveas(h,strcat(save_dir,'\Fresult',int2str(ii),'.jpg'),'jpg');
% clf;


rimgtmp3dtmp=rightimg(1:2:ImgSize1,:);
rimgtmp3dtmp(rimgtmp3dtmp<200)=0;
rimgtmp3d=zeros(ImgSize1/2, ImgSize2, 3);
rimgtmp3d(:,:,1)=rimgtmp3dtmp;
rimgtmp3d(:,:,2)=rimgtmp3dtmp;
rimgtmp3d(:,:,3)=rimgtmp3dtmp;
% % % % rimgtmp1d=double(rimgtmp3d(:,:,1));
% % % % % rimgtmp1d=Retify(rimgtmp1d,a1_right,a2_right,a3_right,a4_right,ind_1_right,ind_2_right,ind_3_right,ind_4_right,ind_new_right);
% % % % rimgtmp3d(:,:,1)=rimgtmp1d;
% % % % rimgtmp1d=double(rimgtmp3d(:,:,2));
% % % % % rimgtmp1d=Retify(rimgtmp1d,a1_right,a2_right,a3_right,a4_right,ind_1_right,ind_2_right,ind_3_right,ind_4_right,ind_new_right);
% % % % rimgtmp3d(:,:,2)=rimgtmp1d;
% % % % rimgtmp1d=double(rimgtmp3d(:,:,3));
% % % % % rimgtmp1d=Retify(rimgtmp1d,a1_right,a2_right,a3_right,a4_right,ind_1_right,ind_2_right,ind_3_right,ind_4_right,ind_new_right);
% % % % rimgtmp3d(:,:,3)=rimgtmp1d;
% % % % rimgtmp3d=rimgtmp3d(1:2:ImgSize1,:,:);
% % % % rimgtmp3d(ind2)=255;
% % % % rimgtmp3d(ind2+ImgSize1/2*ImgSize2)=0;
% % % % rimgtmp3d(ind2+ImgSize1/2*ImgSize2*2)=0;
% % % % rimgtmp3d(ind)=0;
% % % % rimgtmp3d(ind+ImgSize1/2*ImgSize2)=255;
% % % % rimgtmp3d(ind+ImgSize1/2*ImgSize2*2)=0;
imshow(rimgtmp3d,[]); hold on;
plot(round(xtmpr),-ytmpr,'r','LineWidth',2);
plot(round(xtmpr2),-ytmpr2,'r','LineWidth',2);
plot(xx,-yy,'g.'); hold off;
frame=getframe(gcf);
im=frame2im(frame);
imwrite(im,strcat(save_dir,'\3DFresult',int2str(ii),'.jpg'),'jpg');