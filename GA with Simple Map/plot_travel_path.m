figure;
for j=1:(i-1)
    tic;
    plot(-ctr_x,ctr_z,'k--','LineWidth',0.5);hold on;
    patch(-[map_x map_x_bar],[map_z map_z_bar],'g');
    plot(-ctr_x,ctr_z,'k--','LineWidth',0.5);
    plotCar(pi-act_theta(j),-act_x(j),act_z(j),act_phi(j));
    plot(-act_x(1:j),act_z(1:j),'r');
    hold off;axis equal;
    if -act_x(j)>=xdumy(1) && -act_x(j)<=xdumy(2)
        xdumy_i=1;
    elseif -act_x(j)>=xdumy(2) && -act_x(j)<=xdumy(3)
        xdumy_i=2;
    elseif -act_x(j)>=xdumy(3) && -act_x(j)<=xdumy(4)
        xdumy_i=3;
    end
    if act_z(j)>=zdumy(1) && act_z(j)<=zdumy(2)
        zdumy_i=1;
    elseif act_z(j)>=zdumy(2) && act_z(j)<=zdumy(3)
        zdumy_i=2;
    elseif act_z(j)>=zdumy(3) && act_z(j)<=zdumy(4)
        zdumy_i=3;
    end
    axis([xdumy(xdumy_i) xdumy(xdumy_i+1) zdumy(zdumy_i) zdumy(zdumy_i+1)]);
    
    frame=getframe(gcf);
    im=frame2im(frame);
    imwrite(im,strcat('D:\Stereo\MPC\GA with Simple Map\Simulated Img\Running',int2str(j),'.jpg'),'jpg');
    
    endt=toc;
    if endt<logt(j)
        pause(logt(j)-endt);
    end
end