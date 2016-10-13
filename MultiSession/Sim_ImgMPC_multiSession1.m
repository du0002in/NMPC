clear all;
sendtarget(0,0);
% load 'MPC_onlineCtrl';
leftimg_dir='D:\Stereo\Image Collection6\right';
rightimg_dir='D:\Stereo\Image Collection6\left';
save_dir='D:\Stereo\Vehicle Simulator\simu_test1';
global_varibles;
InitializationGlobalVariable;
load 'cali3.mat';
phi=.2757;D_pre=852*tan(.2757);H=1.665;marking_width_pre=0.15;diff_marking_width=0;
marking_width_pre2=0.15;continue_frame=0;
top_row=2*floor((cc_y-40)/2)-1; %make sure cc_y-40 is odd
btm_row=2*ceil((cc_y-480)/2); %make sure cc_y-480 is even
cc_y=2*ceil(cc_y/2);
Ridge_width_ini=8.5;
ImgSize1=480; ImgSize2=640;
wd=zeros(5,5,8); wd(3,3,:)=1;
wd(3,:,1)=1;
wd(4,1:2,2)=1; wd(2,4:5,2)=1;wd(3,2,2)=1;wd(3,4,2)=1;
wd(1,5,3)=1;wd(2,4,3)=1;wd(4,2,3)=1;wd(5,1,3)=1;
wd(4:5,2,4)=1;wd(1:2,4,4)=1;wd(4,3,4)=1;wd(2,3,4)=1;
wd(:,3,5)=1;
wd(4:5,4,6)=1;wd(1:2,2,6)=1;wd(4,3,6)=1;wd(2,3,6)=1;
wd(1,1,7)=1;wd(2,2,7)=1;wd(4,4,7)=1;wd(5,5,7)=1;
wd(2,1:2,8)=1; wd(4,4:5,8)=1; wd(3,2,8)=1; wd(3,4,8)=1;
convimg=zeros(240,1310,8);
kcsditmpl=zeros(ImgSize1/2,ImgSize2);
kcsditmpr=zeros(ImgSize1/2,ImgSize2);
xtmpl=[]; xtmpr=[];
Theta=-0.0/180*pi; %for Image Collectio
k_s=0.5; P_s=0.2; Q_s=0.5;exp_n=10;
OMEGA=0.45; 
ite_s=1;
ite_e=20;

Ts=0.03; v0=10;
step_size_theta=pi/17;
step_size_phi=0.18;
sendtarget(v0,0);
updateLocation(0,0,0,0);
runningstatus(1);
ini_xc=-1.8; ini_d_theta=0.186; %These two values need to guarantee that the line is the road center line not the road boundary line
% map_y=(-1:0.01:100)'; map_x=-(tan(ini_d_theta)*map_y+ini_xc/cos(ini_d_theta));
map_y=(-1:0.01:100)'; map_x=1.5*ones(length(map_y),1);
map_z=zeros(length(map_x),1);
% figure;plot(map_x,map_y); hold on; plotCar(pi-(pi/2+ini_d_theta),(ini_xc+1.5),0,0); hold off;axis equal;
abc=[0 tan(ini_d_theta) ini_xc/cos(ini_d_theta)];

Lane_marking_point_x=[map_x+0.075 map_x-0.075];
Lane_marking_point_y=[map_y map_y];Lane_marking_point_z=zeros(size(Lane_marking_point_y,1),size(Lane_marking_point_y,2));
[Tpix2cam_left, Tpix2cam_right]=Pix2Cam(focal,cc_x_left,cc_x_right,cc_y);
[leftimg, rightimg]=stereo_img_SL((ini_xc+1.5),0,pi-(pi/2+ini_d_theta),phi,H,dist_left_right,L_wheel_cam,Tpix2cam_left, Tpix2cam_right,map_x,map_y,map_z, Lane_marking_point_x, Lane_marking_point_y, Lane_marking_point_z);

% VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta);
% x0=-(ini_xc+1.5);theta0=pi/2+ini_d_theta; phi0=0;
% VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta, x0, 0, theta0, phi0, v0);pause(0.02);
prompt='Press enter to start the session';
result=input(prompt,'s');
KickOff_PF_SimulatorMPC_multi;

% inner_thread('Ini',ini_xc,ini_d_theta,abc,L_wheels,Q_s,P_s,0,0.6);

inner_loop_logging('Ini');
for ii=(ite_s+1):ite_e
    in_start=tic;
    
%     [v_m, phai_m]=get_measurements;
    [act_x(ii), act_z(ii), act_theta(ii), act_phi(ii), act_xc(ii), act_d_theta(ii),act_v(ii)]=getActPose;
%     plot(map_x,map_y); hold on;plot(-act_x,act_z,'r'); plotCar(pi-act_theta(ii),-act_x(ii),act_z(ii),act_phi(ii)); hold off;axis equal;
    [leftimg, rightimg]=stereo_img_SL(-act_x(ii),act_z(ii),pi-act_theta(ii),phi,H,dist_left_right,L_wheel_cam,Tpix2cam_left, Tpix2cam_right,map_x,map_y,map_z, Lane_marking_point_x, Lane_marking_point_y, Lane_marking_point_z);
    start=tic;
    imwrite(leftimg,strcat(save_dir,'\left',int2str(ii),'.jpg'),'jpg');
    imwrite(rightimg,strcat(save_dir,'\right',int2str(ii),'.jpg'),'jpg');
%     leftimg=double(max(leftimg,[],3));
%     rightimg=double(max(rightimg,[],3));
%     % tic
%     leftimg=Retify(leftimg,a1_left,a2_left,a3_left,a4_left,ind_1_left,ind_2_left,ind_3_left,ind_4_left,ind_new_left);
%     rightimg=Retify(rightimg,a1_right,a2_right,a3_right,a4_right,ind_1_right,ind_2_right,ind_3_right,ind_4_right,ind_new_right);
    img=[leftimg(1:2:ImgSize1,:) zeros(ImgSize1/2, 30) rightimg(1:2:ImgSize1,:)];
    img(235:end,:)=0;
    mask=img;
    
    ImgSmooth=zeros(size(img));
    for i=1:(length(ix)-1)
        ImgSmooth(ix(i):(ix(i+1)-1),:)=conv2(img(ix(i):(ix(i+1)-1),:),xfil2{i},'same');
    end
    
    ImgSmooth=conv2(ImgSmooth,yfil2,'same');
    [kcsditmp,~]=C_Ridge(ImgSmooth,306400);
    
    th=1.6;
    for i=1:8
        convimg(:,:,i)=conv2(kcsditmp,wd(:,:,i),'same');
    end
    maxconvimg=max(convimg,[],3);
    maxconvimg=maxconvimg.*mask;
    
    dumy1=maxconvimg;
    dumy1(dumy1<connected_pixel*th)=0; dumy1(dumy1>=connected_pixel*th)=1;
    % tic;
    num_ite=15000; %number of iterations for RANSAC
    if ~isempty(xtmpl) && ~isempty(xtmpr)
%         maxconvimg=C_Search_region(maxconvimg,xtmpl,xtmpr+ImgSize2+30,-ytmpr(1),-ytmpr(end));
%         maxconvimg(-ytmpr(end):end,:)=0;
%         maxconvimg(1:-ytmpr(1),:)=0;
        num_ite=960;
    end
    % toc
    dumy2=maxconvimg;
    dumy2(dumy2<connected_pixel*th)=0;dumy2(dumy2>=connected_pixel*th)=1;
    dumy=[dumy1;dumy2];
    % dumy(:,ImgSize2/2:ImgSize2)=0;
    % dumy(:,end-320:end)=0;
    imwrite(dumy,strcat(save_dir,'\dumyRidge',int2str(ii),'.jpg'),'jpg');
    
    [sel_xtmpl,sel_ytmpl, xtmpl, ytmpl,sel_xtmpr,sel_ytmpr, xtmpr, ytmpr, num_in, para]=linefitting_simu(maxconvimg(:,1:ImgSize2),maxconvimg(:,end-ImgSize2+1:end),th, D_pre, H, num_ite);
    lognum_in(ii)=num_in;
    if num_in<100
        update_w_missinfor_SimulatorMPC_multi;
        continue;
    end
    %convert back to original image coordinate 480x640
    % % [~,ir,il]=intersect(sel_ytmpr, ytmpl, 'stable');
    if leftrightlane==-1
% %         if xtmpr(end)<320 %This condition may not be true...Need to update this based on Xworld coordinate
% %             leftrightlane=0;
% %         else
% %             leftrightlane=1;
% %         end
        leftrightlane=1;
    end
    if leftrightlane==0 %left lane
        ir=intersectAB(ytmpl,sel_ytmpr);
        il=intersectAB(sel_ytmpr,ytmpl);
        % toc(start)
        sel_ytmpr=sel_ytmpr(ir==1);
        sel_xtmpr=sel_xtmpr(ir==1);
        Corespx=xtmpl(il==1);
        delta_x=round(0.308*(-sel_ytmpr)+7.385);
        sel_ytmpr=2*sel_ytmpr-1; %convert back to -480x640 coordinate
        ri_lb=round(sel_xtmpr-delta_x); %right image left boundary
        ri_rb=round(sel_xtmpr+delta_x); %right image right boundary
        ri_lb(ri_lb<=1)=1;
        ri_rb(ri_rb>=ImgSize2)=ImgSize2;
        li_lb=round(Corespx-2*delta_x);
        li_rb=round(Corespx+2*delta_x);
        li_lb(li_lb<=1)=1;
        li_rb(li_rb>=ImgSize2)=ImgSize2;
        ri_lb=ri_lb-1;
        ri_rb=ri_rb-1;
        li_lb=li_lb-1;
        li_rb=li_rb-1;
        
        IntensityR=rightimg(-sel_ytmpr,:);
        CorespL=leftimg(-sel_ytmpr,:);
        % toc(start)
        disparitytmp=ZNCC(ri_lb', ri_rb', li_lb', li_rb', IntensityR, CorespL);
        % toc(start)
        li_lb=li_lb+1;
        ri_lb=ri_lb+1;
        % Refer to the sign convention in 3D Vision notes
        % Convert to left and right camera coordinates respectively
        CorepxL=-(li_lb+disparitytmp'+(round(sel_xtmpr)-ri_lb)-cc_x_left);
        CorepxR=-(round(sel_xtmpr)-cc_x_right);
    else %right lane
        il=intersectAB(ytmpr,sel_ytmpl);
        ir=intersectAB(sel_ytmpl,ytmpr);
        % toc(start)
        sel_ytmpl=sel_ytmpl(il==1);
        sel_xtmpl=sel_xtmpl(il==1);
        Corespx=xtmpr(ir==1);
        delta_x=round(0.308*(-sel_ytmpl)+7.385);
        sel_ytmpl=2*sel_ytmpl-1; %convert back to -480x640 coordinate
        li_lb=round(sel_xtmpl-delta_x); %right image left boundary
        li_rb=round(sel_xtmpl+delta_x); %right image right boundary
        li_lb(li_lb<=1)=1;
        li_rb(li_rb>=ImgSize2)=ImgSize2;
        ri_lb=round(Corespx-2*delta_x);
        ri_rb=round(Corespx+2*delta_x);
        ri_lb(ri_lb<=1)=1;
        ri_rb(ri_rb>=ImgSize2)=ImgSize2;
        li_lb=li_lb-1;
        li_rb=li_rb-1;
        ri_lb=ri_lb-1;
        ri_rb=ri_rb-1;
        
        IntensityL=leftimg(-sel_ytmpl,:);
        CorespR=rightimg(-sel_ytmpl,:);
        % toc(start)
        disparitytmp=ZNCC(li_lb', li_rb', ri_lb', ri_rb', IntensityL, CorespR);
        % toc(start)
        li_lb=li_lb+1;
        ri_lb=ri_lb+1;
        % Refer to the sign convention in 3D Vision notes
        % Convert to left and right camera coordinates respectively
        CorepxR=-(ri_lb+disparitytmp'+(round(sel_xtmpl)-li_lb)-cc_x_right);
        CorepxL=-(round(sel_xtmpl)-cc_x_left);
    end
    disparity=CorepxR-CorepxL;
    indtmp=(disparity~=0);
    Zc=dist_left_right*focal./disparity;
    Xc=CorepxL.*Zc/focal+dist_left_right/2;
    % or Xc=CorepxR.*Zc/focal-dist_left_right/2;
    if leftrightlane==0
        Yc=(round(sel_ytmpr)+cc_y).*Zc/focal;
        ind_near=(sel_ytmpr<-192); %Use those pixels at 2/3bottom to calculate phai
    else
        Yc=(round(sel_ytmpl)+cc_y).*Zc/focal;
        ind_near=(sel_ytmpl<-192); %Use those pixels at 2/3bottom to calculate phai
    end
    ind_near=ind_near & indtmp;
    %     if length(Yc)>5
    %         Yc_near=Yc(end-5:end); Zc_near=Zc(end-5:end);
    %     else
    %         Yc_near=Yc; Zc_near=Zc;
    %     end
    if sum(ind_near)>5
        Yc_near=Yc(ind_near); Zc_near=Zc(ind_near); Xc_near=Xc(ind_near);
    else
        Yc_near=Yc(indtmp); Zc_near=Zc(indtmp); Xc_near=Xc(indtmp);
    end
    Atmp=[Zc_near' -ones(length(Zc_near),1)];
    Btmp=Yc_near'*cos(Theta)+Xc_near'*sin(Theta);
    tan_Phai_H=pinv(Atmp)*Btmp;
    tan_Phai=tan_Phai_H(1);
    Phai=atan(tan_Phai);
    logPhaiNaN(ii)=Phai;
    if isnan(Phai)
        Phai=atan(D_pre/focal);
    end
    logPhai(ii)=Phai;
    Phai=median(logPhai(max([ite_s, ii-10]):ii)); %median value of the previous 10 results.
    logPhai_f(ii)=Phai;
    D_pre=focal*tan(Phai);
    %     H=mean((Zc_near*tan(Phai)-Yc_near)*cos(Phai));
    H=tan_Phai_H(2)*cos(Phai);
    logHNaN(ii)=H;
    if H<1.4 || isnan(H) || H>1.8
        H=1.665;
    end
    logH(ii)=H;
    H=median(logH(max([ite_s, ii-10]):ii)); %median value of the previous 10 results.
    logH_f(ii)=H;
    [xtmpl, ytmpl, xtmpr, ytmpr, refine_para]=refine_model(leftrightlane,para,sel_ytmpr,sel_ytmpl,D_pre,CorepxR,CorepxL);
    if numel(ytmpl)==0 || numel(ytmpr)==0
        update_w_missinfor_SimulatorMPC_multi;
        continue;
    end
    
    [sel_xtmpl2,sel_ytmpl2,xtmpl2, ytmpl2, sel_xtmpr2,sel_ytmpr2,xtmpr2, ytmpr2, num_in2, para2, lwtmp, dumytmp]=fit_other_lane(leftrightlane,dumy1,refine_para,-ytmpr(1),-ytmpr(end),H,Phai,D_pre);
    lognum_in2(ii)=num_in2;
    dumy=[dumy; dumytmp];
    imwrite(dumy,strcat(save_dir,'\Ridge',int2str(ii),'.jpg'),'jpg');
    if isempty(lwtmp)
        log_lw(ii,:)=0;
    else
        log_lw(ii,:)=lwtmp;
    end
    log_non_0_lw=log_lw(log_lw~=0);
    log_non_0_num_in2=lognum_in2(log_lw~=0);
    m=-min([10, length(log_non_0_lw)]):-1;
    coe_fil=cos(m*pi/2/max(-m));
    coe_fil=log_non_0_num_in2((end-max(-m)+1):end).*coe_fil;
    coe_fil=coe_fil'/sum(coe_fil);
    lane_width=sum(coe_fil.*log_non_0_lw((end-max(-m)+1):end));
    log_lane_width_f(ii)=lane_width;
    
    R=[cos(Theta) -sin(Theta) 0; ...
        cos(Phai)*sin(Theta) cos(Phai)*cos(Theta) -sin(Phai); ...
        sin(Phai)*sin(Theta) sin(Phai)*cos(Theta) cos(Phai);];
    T=[0;H;L_wheel_cam];
    World=[R T;0 0 0 1]*[Xc; Yc; Zc; ones(1,length(Zc))];
    % logXw=World(1); logYw=World(2); logZw=World(3);
    % toc
    Xw=World(1,:); Yw=World(2,:); Zw=World(3,:);
    inf_ind=(isinf(Xw) | isinf(Yw) | isinf(Zw))| (isnan(Xw) | isnan(Yw) | isnan(Zw));
    Xw(inf_ind)=[];Yw(inf_ind)=[];Zw(inf_ind)=[];
    World(:,inf_ind)=[];
    log_World{ii}=World;
%     log_Yw_mean(ii)=mean(Yw);
    
    if sum(abs(para(:,2)))==0 %model is hypobola
        Atmp=[Zw.^2; Zw; ones(1,length(Zw))]';
        Btmp=Xw';
        abc=pinv(Atmp)*Btmp;
    else %model is line
        Atmp=[Zw; ones(1,length(Zw))]';
        Btmp=Xw';
        bc=pinv(Atmp)*Btmp;
        abc=[0; bc];
    end
    [xc_m, d_theta_m]=solve_xc_d_theta(abc,0,0,pi/2);   
    log_xc_m(ii)=xc_m;
    log_d_theta_m(ii)=d_theta_m;
    if num_in2>num_in
        xtmpl=xtmpl2; ytmpl=ytmpl2; xtmpr=xtmpr2; ytmpr=ytmpr2;
        lanelinechange=1;
    else lanelinechange=0;
    end
    %     logWidth(ii)=f_width;
    if sum(ind_near)>6
        ind_near(1:(end-7))=0;
    end
    if mod(sum(ind_near),2)==0
        indtmp=find(ind_near==1,1);
        ind_near(indtmp)=0;
    end
    if sum(ind_near)>=5
        if leftrightlane==0
            y_for_tdx=sel_ytmpr(ind_near)';
            x_for_tdx=round(sel_xtmpr(ind_near)');
            Intensity=rightimg(-y_for_tdx,:);
        else
            y_for_tdx=sel_ytmpl(ind_near)';
            x_for_tdx=round(sel_xtmpl(ind_near)');
            Intensity=leftimg(-y_for_tdx,:);
        end
        y_for_tdx=y_for_tdx+cc_y; %convert to camera coordinate
        tdxtmp1=round((cos(Phai)/H*(D_pre-y_for_tdx)*0.10)/2);
        tdxtmp2=round((cos(Phai)/H*(D_pre-y_for_tdx)*0.15)/2);
        tdxtmp3=round((cos(Phai)/H*(D_pre-y_for_tdx)*0.20)/2);
        quater_tdxtmp3=round(1/4*tdxtmp3);
        x_for_tdx_lb=x_for_tdx-tdxtmp3-quater_tdxtmp3;
        x_for_tdx_rb=x_for_tdx+tdxtmp3+quater_tdxtmp3;
        x_for_tdx_lb(x_for_tdx_lb<1)=1;
        x_for_tdx_lb(x_for_tdx_lb>ImgSize2)=ImgSize2;
        x_for_tdx_rb(x_for_tdx_rb<1)=1;
        x_for_tdx_rb(x_for_tdx_rb>ImgSize2)=ImgSize2;
        marking_width_all=C_find_lane_marking_width(Intensity,x_for_tdx_lb,x_for_tdx_rb,tdxtmp1,tdxtmp2,tdxtmp3,x_for_tdx);
        marking_width=median(marking_width_all);
        if leftrightlane==0
            log_marking_width_left(ii)=marking_width;
            marking_width=median(log_marking_width_left(max([ite_s, ii-4]):ii));
            log_marking_width_f_left(ii)=marking_width;
            log_marking_width_right(ii)=log_marking_width_right(ii-1);
            log_marking_width_f_right(ii)=log_marking_width_f_right(ii-1);
        else
            log_marking_width_right(ii)=marking_width;
            marking_width=median(log_marking_width_right(max([ite_s, ii-4]):ii));
            log_marking_width_f_right(ii)=marking_width;
            log_marking_width_left(ii)=log_marking_width_left(ii-1);
            log_marking_width_f_left(ii)=log_marking_width_f_left(ii-1);
        end
        if marking_width~=marking_width_pre
            marking_width_pre=marking_width;
            if mod(floor(D_pre),2)==0
                D_pre1=floor(D_pre)-1;
            else D_pre1=floor(D_pre);
            end
            yy=min(floor(D_pre1),top_row):-1:btm_row;
            tdxtmp=(cos(Phai)/H*(D_pre-yy)*marking_width)/2; %0.10m is the lane marking width
            if length(tdxtmp)>=length(tdx)
                tdx=round(tdxtmp((end-length(tdx)+1):end));
            else
                tdx=round([0.5*ones(1,(length(tdx)-length(tdxtmp))) tdxtmp]);
            end
            tdx(tdx==0)=1;
            [~,ix,~]=unique(tdx,'stable');ix=ix';
            j=1;
            for i=ix
                weight3=exp((-0.5*(-tdx(i):tdx(i)).^2/(tdx(i))^2));
                weight3=weight3/sum(weight3);
                xfil2{j}=weight3;
                j=j+1;
            end
            ix(length(ix)+1)=ImgSize1-Offset+1;
            ix=round(ix/2)+Offset/2;
        end
    else
        log_marking_width_left(ii)=log_marking_width_left(ii-1);
        log_marking_width_f_left(ii)=log_marking_width_f_left(ii-1);
        log_marking_width_right(ii)=log_marking_width_right(ii-1);
        log_marking_width_f_right(ii)=log_marking_width_f_right(ii-1);
    end
    
    %generate weights based on measurement and prediction. then resample
    %particles, estimate location, transform other particles based on the
    %estimated location
%     var_w_xc=(2.5071*10^(-7)*num_in*num_in-0.00051078*num_in+0.26296)^2;
%     var_w_d_theta=(-3.4*10^(-5)*num_in+0.033).^2;
    var_w_xc=(450/min([num_in,450])*0.0914)^2; 
    %std 0.07m is based on comparsion between measurements and ground truth. 
    %450 corresponding to average inliers for the corresponding ground truth images
    %xc measurement error (m-gt) follows closely as a normal distribution (0,0.0914)
    var_w_d_theta=(450/min([num_in,450])*0.0334).^2;
    %std 0.0261rad is based on comparison between measurements and ground
    %truth. 450 corresponding to average inliers for the corresponding
    %ground truth images. 
    %d_theta measurment error (m-gt) follws an offset normal distribution
    %(-0.0133, 0.0334)
%     PF_wt=exp(-0.5*((Predict_P(7,:)-v_m).^2/var_w_v+(Predict_P(8,:)-phai_m).^2/var_w_phai+ ...
%                 (Predict_Xc-xc_m).^2/var_w_xc+(Predict_d_theta-d_theta_m).^2/var_w_d_theta));
    if pre_leftrightlane~=leftrightlane
        if pre_leftrightlane==0
            Predict_Xc=Predict_Xc-lane_width;
        else
            Predict_Xc=Predict_Xc+lane_width;
        end
    end
    PF_wt=exp(-0.5*((xc_m-Predict_Xc).^2/var_w_xc+(d_theta_m-(-0.0133)-Predict_d_theta).^2/var_w_d_theta));
    if sum(PF_wt)==0 %meaning the measurement is far way off from the prediction, indicating wrong fitting for this cycle.
        update_w_missinfor_SimulatorMPC_multi;
        xtmpl=[]; xtmpr=[];
        continue;
    end
    PF_wt=PF_wt./sum(PF_wt);
    cumsum_PF_wt=cumsum(PF_wt);
    particle_index=C_resample_PF(cumsum_PF_wt);
    particle_index(particle_index>P_No)=P_No; 
    Particle=Predict_P(:,particle_index);
    Estimate_x_mean=mean(Particle(1,:));
    Estimate_z_mean=mean(Particle(2,:));
    Estimate_theta_mean=mean(Particle(3,:));
    [Estimate_xc, Estimate_d_theta, curvature]=solve_xc_d_theta(pre_abc,Estimate_x_mean,Estimate_z_mean,Estimate_theta_mean);
    phi_d=atan(L_wheels*curvature);
    if pre_leftrightlane~=leftrightlane
        if pre_leftrightlane==0
            Estimate_xc=Estimate_xc-lane_width;
        else
            Estimate_xc=Estimate_xc+lane_width;
        end
    end
    
    
    Rtmp=[cos(Estimate_theta_mean-0.5*pi) sin(Estimate_theta_mean-0.5*pi); ...
        -sin(Estimate_theta_mean-0.5*pi) cos(Estimate_theta_mean-0.5*pi)];
%     Particle(1:2,:)=[cos(Estimate_theta_mean-0.5*pi) sin(Estimate_theta_mean-0.5*pi) -Estimate_x_mean; ...
%         -sin(Estimate_theta_mean-0.5*pi) cos(Estimate_theta_mean-0.5*pi) -Estimate_z_mean;]*[Particle(1:2,:); ones(1,P_No)];
    Particle(1:2,:)=[Rtmp -Rtmp*[Estimate_x_mean;Estimate_z_mean]]*[Particle(1:2,:); ones(1,P_No)];
    Particle(3,:)=Particle(3,:)-(Estimate_theta_mean-0.5*pi);
    Particle(4:5,:)=Rtmp*Particle(4:5,:);
    log_Estimate_xc(ii)=Estimate_xc;
    log_Estimate_d_theta(ii)=Estimate_d_theta;
    %Transform the previous abc and the current abc based on the newly
    %estimated locations. The one with more num_in should be used.
    if num_in<lognum_in(ii-1) && (pre_leftrightlane==leftrightlane) && (continue_frame<5)
        new_pre_World=[Rtmp -Rtmp*[Estimate_x_mean;Estimate_z_mean]]*pre_World([1,3,4],:);
        new_pre_World=[new_pre_World(1,:); pre_World(2,:); new_pre_World(2,:); pre_World(4,:)];
        new_pre_Xw=new_pre_World(1,:); new_pre_Yw=new_pre_World(2,:); new_pre_Zw=new_pre_World(3,:);
        if sum(abs(pre_para(:,2)))==0 %model is hypobola
            Atmp=[new_pre_Zw.^2; new_pre_Zw; ones(1,length(new_pre_Zw))]'; Btmp=new_pre_Xw';
            pre_abc=pinv(Atmp)*Btmp;
        else %model is line
            Atmp=[new_pre_Zw; ones(1,length(new_pre_Zw))]'; Btmp=new_pre_Xw';
            pre_bc=pinv(Atmp)*Btmp;
            pre_abc=[0; pre_bc];
        end
        World=new_pre_World;
        pre_World=World;
        continue_frame=continue_frame+1;
    else
        new_Estimate_theta_mean=Estimate_d_theta+0.5*pi-d_theta_m;
        new_Estimate_x_mean=-(Estimate_xc-xc_m)*cos(d_theta_m);
        new_Estimate_z_mean=(Estimate_xc-xc_m)*sin(d_theta_m);
        Rtmp1=[cos(new_Estimate_theta_mean-0.5*pi) sin(new_Estimate_theta_mean-0.5*pi); ...
        -sin(new_Estimate_theta_mean-0.5*pi) cos(new_Estimate_theta_mean-0.5*pi)];
        new_World=[Rtmp1 -Rtmp1*[new_Estimate_x_mean;new_Estimate_z_mean]]*World([1,3,4],:);
        new_World=[new_World(1,:); World(2,:); new_World(2,:); World(4,:)];
        new_Xw=new_World(1,:); new_Yw=new_World(2,:); new_Zw=new_World(3,:);
        if sum(abs(para(:,2)))==0 %model is hypobola
            Atmp=[new_Zw.^2; new_Zw; ones(1,length(new_Zw))]'; Btmp=new_Xw';
            pre_abc=pinv(Atmp)*Btmp;
        else
            Atmp=[new_Zw; ones(1,length(new_Zw))]'; Btmp=new_Xw';
            pre_bc=pinv(Atmp)*Btmp;
            pre_abc=[0; pre_bc];
        end
        World=new_World;
        pre_World=World;
        pre_para=para;
        continue_frame=0;
    end
    
    xzthetapredict=inner_loop_logging('End');
    [log_predi_xc(ii), log_predi_d_theta(ii)]=solve_xc_d_theta(pre_abc,xzthetapredict(1),xzthetapredict(2),xzthetapredict(3));
    predi_xc=log_predi_xc(ii); predi_d_theta=log_predi_d_theta(ii);
    updateLocation(1,-predi_xc-1.5,act_z(ii)+xzthetapredict(2),pi/2+predi_d_theta);
    
%     [v_m, phai_m]=get_measurements;
%     theta0=pi/2+predi_d_theta;
%     phi0=phai_m;
%     v0=v_m;
%     X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; -(predi_xc+1.5)+v0*cos(theta0)*Ts; ...
%         act_z(ii)+xzthetapredict(2)+v0*sin(theta0)*Ts; theta0+-tan(phi0)*v0*Ts/1.28; phi0; v0;phi0;phi0;];
%     position=floor((phi0--0.45)/step_size_phi)+floor((theta0+-tan(phi0)*v0*Ts/1.28)/step_size_theta)*round(0.9/step_size_phi)+1;
%     [Uonl,~,cost{ii}] = onl_ctrl(position).evaluate(X0);
%     logUonl(:,ii)=Uonl(:,1);
%     logPosition(ii)=position;
%     if isnan(Uonl(1))
%     else
%         phi_t=cost{ii}.U(1,:)+cost{ii}.X(7,1:(end-1));
%         v_t=cost{ii}.U(2,:)+cost{ii}.X(8,1:(end-1));
%     end
%     % sendtarget(v0+d_v_t,phi0+d_phi_t);
%     C_sendtargetMPC('Con',phi_t,v_t);
%     
    
    [loop_t, loop_phi, loop_v]=getloop_t_phi_v;

    inner_loop_logging('Ini');
    
    
    endt(ii)=toc(start);
    endf(ii)=toc(in_start);
    
    %Predict current location based on the v, phai and dt. Predict xc,
    %d_theta
    dt=endf(ii);
    [v_m, phai_m]=get_measurements;
    log_v_m(ii)=v_m;
    log_phi_m(ii)=phai_m;
    log_phi_d(ii)=phi_d;
% %     if log_Estimate_xc(ii)>=0
% %         de=log_Estimate_xc(ii)-0.7;
% %     else
% %         de=log_Estimate_xc(ii)+0.7;
% %     end
% %     de_dot=v_m*sin(log_Estimate_d_theta(ii));
% %     k_s=abs(v_m)/L_wheels*3;
% %     s=de_dot+k_s*de;
% %     phi_t=atan(Q_s*atan(s/P_s)+sign(v_m)*3.0*tan(log_Estimate_d_theta(ii))-tan(phi_d));
% %     log_phi_t(ii)=phi_t;
% %     if (phi_t-log_phi_t(ii-1))>dt*OMEGA
% %         phi_t=log_phi_t(ii-1)+dt*OMEGA;
% %     elseif (phi_t-log_phi_t(ii-1))<-dt*OMEGA
% %         phi_t=log_phi_t(ii-1)-dt*OMEGA;
% %     end
% %     sendtarget(v_t, phi_t);
    
%     inner_thread('Con',act_xc(ii),Estimate_d_theta,pre_abc,L_wheels,Q_s,P_s,phi_d,v_t);
    Predict_P=C_Predict_P(Particle,loop_t,loop_phi,loop_v);
%     if v_m~=0
%         v0=v_m+sqrt(var_v).*randn(1,P_No);
%     else
%         v0=zeros(1,P_No);
%     end
%     phai0=phai_m+sqrt(var_phai).*randn(1,P_No);
%     Particle(7,:)=v0;
%     Particle(8,:)=phai0;
%     Predict_P(8,:)=Particle(8,:);
%     Predict_P(7,:)=Particle(7,:);
%     Predict_P(4,:)=cos(Particle(3,:)).*Particle(7,:);
%     Predict_P(5,:)=sin(Particle(3,:)).*Particle(7,:);
%     Predict_P(6,:)=-tan(Particle(8,:)).*Particle(7,:)/L_wheels;
%     Predict_P(1,:)=Particle(1,:)+Particle(4,:)*dt+0.25*dt/0.2*marking_width*randn(1,P_No);
%     Predict_P(2,:)=Particle(2,:)+Particle(5,:)*dt+0.25*dt/0.2*marking_width*randn(1,P_No);
%     Predict_P(3,:)=Particle(3,:)+Particle(6,:)*dt+0.01*dt/0.2*randn(1,P_No);
    
    Predict_P(1,:)=Predict_P(1,:)+0.25*dt/0.2*marking_width*randn(1,P_No);
    Predict_P(2,:)=Predict_P(2,:)+0.25*dt/0.2*marking_width*randn(1,P_No);
    Predict_P(3,:)=Predict_P(3,:)+0.01*dt/0.2*randn(1,P_No); 
    [Predict_Xc, Predict_d_theta]=C_solve_xc_d_theta(pre_abc, Predict_P(1:3,:));
    
    if Estimate_xc>=0
        leftrightlane=0;
    else
        leftrightlane=1;
    end
%     if (lanelinechange~=1) && (pre_leftrightlane~=leftrightlane)
%         lanechange_flag=1;
%     else lanechange_flag=0;
%     end
    
    pre_leftrightlane=leftrightlane;
    if lanelinechange==1
        leftrightlane=abs(leftrightlane-1);
    end
    
    logParticel{ii}=Particle;
    logPredict{ii}=Predict_P;
    logpre_abc{ii}=pre_abc;
    logPredict_Xc{ii}=Predict_Xc;
    logPredict_d_theta{ii}=Predict_d_theta;
    pause(0.001);
%     plotimg;
    
end
runningstatus(0);
% % 
% % for i=1:70
% %     pause(0.45);
% %     [act_x(i), act_z(i), act_theta(i), act_phi(i), act_xc(i), act_d_theta(i)]=getActPose;
% %     plot(map_x,map_y); hold on;plot(act_x,act_z,'r'); plotCar(act_theta(i),act_x(i),act_z(i),act_phi(i)); hold off;axis equal;
% % %     mea_d_theta(i)=act_d_theta(i)+normrnd(0,0.0334,[1 1]);
% % %     mea_xc(i)=act_xc(i)+normrnd(0,0.0914,[1 1]);
% %     mea_d_theta(i)=act_d_theta(i);
% %     mea_xc(i)=act_xc(i);
% %     abc=[0 tan(mea_d_theta(i)) mea_xc(i)/cos(mea_d_theta(i))];
% %     inner_thread('Con',mea_xc(i), mea_d_theta(i),abc,L_wheels,Q_s,P_s,0,0.6);    
% % end
xzthetapredict=inner_loop_logging('End');
% VehicleSimulatorMPC('End', -5, 0);
VehicleSimulatorMPC('End',-5, 0, 0, 0, 0, 0, 0);
% C_sendtargetMPC('End',phi_t,v_t);
% inner_thread('End',-3.0,0,[0 0 -3.0],L_wheels,0.5,0.2,0,0.6,[0 0 0]);
% hold on; plotCar(pi/2,0,0,0); hold off;