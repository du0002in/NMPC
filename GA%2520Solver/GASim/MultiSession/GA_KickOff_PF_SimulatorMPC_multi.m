% inner_loop_logging('Ini');
ii=ite_s;
num_in_pre=0;num_in=0;


in_start=tic;

% trigger([vid1 vid2]);
% leftimg=getdata(vid1);
% rightimg=getdata(vid2);
imwrite(leftimg,strcat(save_dir,'\left',int2str(ii),'.jpg'),'jpg');
imwrite(rightimg,strcat(save_dir,'\right',int2str(ii),'.jpg'),'jpg');
start=tic;
% leftimg=double(max(leftimg,[],3));
% rightimg=double(max(rightimg,[],3));
% % tic
% leftimg=Retify(leftimg,a1_left,a2_left,a3_left,a4_left,ind_1_left,ind_2_left,ind_3_left,ind_4_left,ind_new_left);
% rightimg=Retify(rightimg,a1_right,a2_right,a3_right,a4_right,ind_1_right,ind_2_right,ind_3_right,ind_4_right,ind_new_right);
img=[leftimg(1:2:ImgSize1,:) zeros(ImgSize1/2, 30) rightimg(1:2:ImgSize1,:)];
img(235:end,:)=0;

for ini_ridge_loop=1:5
    Ridge_width_ini=Ridge_width_ini+2.0;
    RidgeParameter;
    ImgSmooth=zeros(size(img));
    for i=1:(length(ix)-1)
        ImgSmooth(ix(i):(ix(i+1)-1),:)=conv2(img(ix(i):(ix(i+1)-1),:),xfil2{i},'same');
    end
    
    ImgSmooth=conv2(ImgSmooth,yfil2,'same');
    [kcsditmp,th]=C_Ridge(ImgSmooth,306400);
    th=1.6;
    for i=1:8
        convimg(:,:,i)=conv2(kcsditmp,wd(:,:,i),'same');
    end
    maxconvimg=max(convimg,[],3);
    dumy1tmp=maxconvimg;
    dumy1tmp(dumy1tmp<connected_pixel*th)=0; dumy1tmp(dumy1tmp>=connected_pixel*th)=1;
    num_ite=15000; %number of iterations for RANSAC

    [sel_xtmpl0,sel_ytmpl0, xtmpl0, ytmpl0,sel_xtmpr0,sel_ytmpr0, xtmpr0, ytmpr0, num_in0, para0]=linefitting_simu(maxconvimg(:,1:ImgSize2),maxconvimg(:,end-ImgSize2+1:end),th, D_pre, H, num_ite);
    if num_in0>num_in_pre
        dumy1=dumy1tmp;
        dumy2=maxconvimg;
        dumy2(dumy2<connected_pixel*th)=0;dumy2(dumy2>=connected_pixel*th)=1;
        dumy=[dumy1;dumy2];
        imwrite(dumy,strcat(save_dir,'\Ridge',int2str(ii),'.jpg'),'jpg');
        sel_xtmpl=sel_xtmpl0; sel_ytmpl=sel_ytmpl0; xtmpl=xtmpl0; ytmpl=ytmpl0;
        sel_xtmpr=sel_xtmpr0; sel_ytmpr=sel_ytmpr0; xtmpr=xtmpr0; ytmpr=ytmpr0; 
        num_in=num_in0; para=para0;
        num_in_pre=num_in0;
    end
end

lognum_in(ii)=num_in;
if num_in<100
    diff_marking_width=0;
    logPhai(ii)=0.2757;
    logH(ii)=1.665;
    log_marking_width(ii)=marking_width_pre;
    log_marking_width_f(ii)=marking_width_pre;
    % % %         log_marking_width2(ii)=marking_width_pre2;
    endt(ii)=toc(start);
    endf(ii)=toc(in_start);
    disp 'Cannot Kick Off Particle Filter';
    return;
end
%convert back to original image coordinate 480x640
% % [~,ir,il]=intersect(sel_ytmpr, ytmpl, 'stable');
if mean(xtmpr)<320 %This condition may not be true...Need to update this based on Xworld coordinate
    leftrightlane=0;
else
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
    Yc_near=Yc; Zc_near=Zc; Xc_near=Xc;
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
    endt(ii)=toc(start);
    endf(ii)=toc(in_start);
    disp 'Cannot initialize';
    return;
end
[sel_xtmpl2,sel_ytmpl2,xtmpl2, ytmpl2, sel_xtmpr2,sel_ytmpr2,xtmpr2, ytmpr2, num_in2, para2,lwtmp]=fit_other_lane(leftrightlane,dumy1,refine_para,-ytmpr(1),-ytmpr(end),H,Phai,D_pre);
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
[xc_m, d_theta_m, curvature]=solve_xc_d_theta(abc,0,0,pi/2);
phi_d=atan(L_wheels*curvature);
pre_abc=abc;
pre_World=World;
pre_para=para;
log_Yw_mean(ii)=mean(Yw);

log_xc_m(ii)=xc_m;
log_d_theta_m(ii)=d_theta_m;
if num_in2>num_in
    xtmpl=xtmpl2; ytmpl=ytmpl2; xtmpr=xtmpr2; ytmpr=ytmpr2;
    lanelinechange=1;
else lanelinechange=0;
end
%     logWidth(ii)=f_width;
lognum_in2(ii)=num_in2;
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
    
    log_marking_width_left(ii)=marking_width;
    marking_width=median(log_marking_width_left(max([ite_s, ii-4]):ii));
    log_marking_width_f_left(ii)=marking_width;
    log_marking_width_right(ii)=marking_width;
    log_marking_width_f_right(ii)=marking_width;
    
%     if marking_width~=marking_width_pre
%         marking_width_pre=marking_width;
%         if mod(floor(D_pre),2)==0
%             D_pre1=floor(D_pre)-1;
%         else D_pre1=floor(D_pre);
%         end
%         yy=min(floor(D_pre1),top_row):-1:btm_row;
%         tdxtmp=(cos(Phai)/H*(D_pre-yy)*marking_width)/2; %0.10m is the lane marking width
%         if length(tdxtmp)>=length(tdx)
%             tdx=round(tdxtmp((end-length(tdx)+1):end));
%         else
%             tdx=round([0.5*ones(1,(length(tdx)-length(tdxtmp))) tdxtmp]);
%         end
%         tdx(tdx==0)=1;
%         [~,ix,~]=unique(tdx,'stable');ix=ix';
%         j=1;
%         for i=ix
%             weight3=exp((-0.5*(-tdx(i):tdx(i)).^2/(tdx(i))^2));
%             weight3=weight3/sum(weight3);
%             xfil2{j}=weight3;
%             j=j+1;
%         end
%         ix(length(ix)+1)=ImgSize1-Offset+1;
%         ix=round(ix/2)+Offset/2;
%     end
else
    log_marking_width_left(ii)=marking_width_pre;
    log_marking_width_f_left(ii)=marking_width_pre;
    log_marking_width_right(ii)=marking_width_pre;
    log_marking_width_f_right(ii)=marking_width_pre;
end
endt(ii)=toc(start);
endf(ii)=toc(in_start);
% plotimg;

log_Estimate_xc(ii)=xc_m;
log_Estimate_d_theta(ii)=d_theta_m;

dt=endf(ii);
% % if log_Estimate_xc(ii)>=0
% %     de=log_Estimate_xc(ii)-0.7;
% % else
% %     de=log_Estimate_xc(ii)+0.7;
% % end
% % de_dot=v_m*sin(log_Estimate_d_theta(ii));
% % k_s=abs(v_m)/L_wheels*3;
% % s=de_dot+k_s*de;
% % phi_t=atan(Q_s*atan(s/P_s)+sign(v_m)*3.0*tan(log_Estimate_d_theta(ii))-tan(phi_d));
% % log_phi_t(ii)=phi_t;
% % if (phi_t-phai_m)>dt*OMEGA
% %     phi_t=phai_m+dt*OMEGA;
% % elseif (phi_t-phai_m)<-dt*OMEGA
% %     phi_t=phai_m-dt*OMEGA;
% % end
% % % sendtarget(v_t, phi_t);
% xzthetapredict=inner_loop_logging('End');
% [log_predi_xc(ii), log_predi_d_theta(ii)]=solve_xc_d_theta(abc,xzthetapredict(1),xzthetapredict(2),xzthetapredict(3));
% predi_xc=log_predi_xc(ii); predi_d_theta=log_predi_d_theta(ii);

x0=-(ini_xc+1.5);theta0=pi/2+ini_d_theta; phi0=0;
xzthetapredict=[x0,0,theta0];
log_predi_xc(ii)=xc_m;log_predi_d_theta(ii)=d_theta_m;
predi_xc=log_predi_xc(ii); predi_d_theta=log_predi_d_theta(ii);
% inner_thread('Ini',predi_xc,predi_d_theta,pre_abc,L_wheels,Q_s,P_s,phi_d,v_t,xzthetapredict);


GA_VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta, x0, 0, theta0, phi0, v0, vehicle_abc);
in_start=tic;
start=tic;
pause(0.02);
[act_x(ii), act_z(ii), act_theta(ii), act_phi(ii), act_xc(ii), act_d_theta(ii),act_v(ii)]=GA_getActPose;
% [v_m, phai_m]=get_measurements;

updateLocation(1, act_x(ii),act_z(ii),act_theta(ii));

% theta0=pi/2+predi_d_theta;
% phi0=phai_m;
% v0=v_m;
% X0=[v0*cos(theta0)*(Ts+0.02); v0*sin(theta0)*(Ts+0.02); -tan(phi0)*v0*(Ts+0.02)/1.28; -(predi_xc+1.5)+v0*cos(theta0)*(Ts+0.02); ...
%     0+v0*sin(theta0)*(Ts+0.02); theta0+-tan(phi0)*v0*(Ts+0.02)/1.28; phi0; v0;phi0;phi0;];
% position=floor((phi0--0.45)/step_size_phi)+floor((theta0+-tan(phi0)*v0*(Ts+0.02)/1.28)/step_size_theta)*round(0.9/step_size_phi)+1;
% [Uonl,~,cost{ii}] = onl_ctrl(position).evaluate(X0);
% logUonl(:,ii)=Uonl(:,1);
% logPosition(ii)=position;
% if isnan(Uonl(1))
% else
%     phi_t=cost{ii}.U(1,:)+cost{ii}.X(7,1:(end-1));
%     v_t=cost{ii}.U(2,:)+cost{ii}.X(8,1:(end-1));
% end
% % sendtarget(v0+d_v_t,phi0+d_phi_t);
% C_sendtargetMPC('Ini',phi_t,v_t);



% Initilize particles
Particle=zeros(8,P_No);
Particle(3,:)=pi/2; %theta is 90 degree
% [v_m, phai_m]=get_measurements;
v_m=act_v(ii); phai_m=act_phi(ii);
if v_m~=0
    v0=v_m+sqrt(0.1).*randn(1,P_No);
else
    v0=zeros(1,P_No);
end
phai0=phai_m+sqrt(0.1).*randn(1,P_No);
Particle(4,:)=cos(Particle(3,:)).*v0;
Particle(5,:)=sin(Particle(3,:)).*v0;
Particle(6,:)=tan(phai0).*v0/L_wheels;
Particle(7,:)=v0;
Particle(8,:)=phai0;

endt(ii)=toc(start);
endf(ii)=toc(in_start);
dt=endf(ii);

Predict_P=zeros(8,P_No);
Predict_P(4,:)=cos(Particle(3,:)).*Particle(7,:);
Predict_P(5,:)=sin(Particle(3,:)).*Particle(7,:);
Predict_P(6,:)=-tan(Particle(8,:)).*Particle(7,:)/L_wheels;
Predict_P(1,:)=Particle(1,:)+Particle(4,:)*dt+0.25*dt/0.2*marking_width*randn(1,P_No);
Predict_P(2,:)=Particle(2,:)+Particle(5,:)*dt+0.25*dt/0.2*marking_width*randn(1,P_No);
Predict_P(3,:)=Particle(3,:)+Particle(6,:)*dt;

Predict_P(7,:)=Particle(7,:);
Predict_P(8,:)=Particle(8,:);

% Predict_P=C_Predict_P(Particle,loop_t,loop_phi,loop_v);
[Predict_Xc, Predict_d_theta]=C_solve_xc_d_theta(abc, Predict_P(1:3,:));

particle_index=1:P_No;

if xc_m>=0
    leftrightlane=0;
else
    leftrightlane=1;
end
pre_leftrightlane=leftrightlane;
if lanelinechange==1
    leftrightlane=abs(leftrightlane-1);
end
% % % % % Predict_x_mean=mean(Predict_P(1,:));
% % % % % Predict_z_mean=mean(Predict_P(2,:));
% % % % % Predict_theta_mean=mean(Predict_P(3,:));
% % % % % Rtmp2=[cos(Predict_theta_mean-0.5*pi) sin(Predict_theta_mean-0.5*pi); ...
% % % % %         -sin(Predict_theta_mean-0.5*pi) cos(Predict_theta_mean-0.5*pi)];
% newWorld=[cos(Predict_theta_mean-0.5*pi) sin(Predict_theta_mean-0.5*pi) -Predict_x_mean; ...
%         -sin(Predict_theta_mean-0.5*pi) cos(Predict_theta_mean-0.5*pi) -Predict_z_mean;]* ...
%         World([1,3,4],:);
% % % % % newWorld=[Rtmp2 -Rtmp2*[Predict_x_mean;Predict_z_mean]]*World([1,3,4],:);
% % % % % newWorld=[newWorld(1,:); World(2,:); newWorld(2,:); World(4,:)];
% % % % % newXYZc=([R T;0 0 0 1])\newWorld;
% % % % % newXc=newXYZc(1,:); newYc=newXYZc(2,:); newZc=newXYZc(3,:);
% % % % % yimg=focal*newYc./newZc; %[cc_y (cc_y-480)]*640
% % % % % ximgl=(newXc-dist_left_right/2)*focal./newZc;
% % % % % ximgr=(newXc+dist_left_right/2)*focal./newZc;
% % % % % if sum(abs(para(:,1)))==0
% % % % %     Atmp=-[yimg' ones(length(yimg),1)]; Btmp=ximgl';
% % % % %     ef_l=pinv(Atmp)*Btmp;
% % % % %     Btmp=ximgr';
% % % % %     ef_r=pinv(Atmp)*Btmp;
% % % % %     ytmpl=top_row:-2:btm_row;
% % % % %     xtmpl=-(-ef_l(1)*ytmpl-ef_l(2))+cc_x_left;
% % % % %     ytmpr=top_row:-2:btm_row;
% % % % %     xtmpr=-(-ef_r(1)*ytmpr-ef_r(2))+cc_x_right;
% % % % % else
% % % % %     Atmp=[1./(yimg'-D_pre) yimg' ones(length(yimg),1)]; Btmp=ximgl';
% % % % %     ABC_l=pinv(Atmp)*Btmp;
% % % % %     Btmp=ximgr';
% % % % %     ABC_r=pinv(Atmp)*Btmp;
% % % % %     ytmpl=min(floor(D_pre),top_row):-2:btm_row;
% % % % %     xtmpl=-(ABC_l(1)./(ytmpl-D_pre)+ABC_l(2)*ytmpl+ABC_l(3))+cc_x_left;
% % % % %     ytmpr=min(floor(D_pre),top_row):-2:btm_row;
% % % % %     xtmpr=-(ABC_r(1)./(ytmpr-D_pre)+ABC_r(2)*ytmpr+ABC_r(3))+cc_x_right;
% % % % % end
% % % % % ytmpl=-(-ytmpl+cc_y+1)/2; %[-1 -240]x640
% % % % % ytmpr=-(-ytmpr+cc_y+1)/2;
% % % % % 
% % % % % ytmpl(round(xtmpl)<=0)=[];
% % % % % xtmpl(round(xtmpl)<=0)=[];
% % % % % ytmpl(round(xtmpl)>ImgSize2)=[];
% % % % % xtmpl(round(xtmpl)>ImgSize2)=[];
% % % % % 
% % % % % ytmpr(round(xtmpr)<=0)=[];
% % % % % xtmpr(round(xtmpr)<=0)=[];
% % % % % ytmpr(round(xtmpr)>ImgSize2)=[];
% % % % % xtmpr(round(xtmpr)>ImgSize2)=[];