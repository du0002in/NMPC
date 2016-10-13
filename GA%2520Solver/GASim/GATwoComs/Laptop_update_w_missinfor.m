%Update particles when no measurement is available
leftrightlane=-1;
diff_marking_width=0;
logPhai(ii)=0.2757;
logPhaiNaN(ii)=0.2757;
logPhai_f(ii)=0.2757;
logH(ii)=1.665;
logHNaN(ii)=1.665;
logH_f(ii)=1.665;
log_marking_width_left(ii)=log_marking_width_left(ii-1);
log_marking_width_f_left(ii)=log_marking_width_f_left(ii-1);
log_marking_width_right(ii)=log_marking_width_right(ii-1);
log_marking_width_f_right(ii)=log_marking_width_f_right(ii-1);
log_lw(ii)=0;
log_lane_width_f(ii)=0;
lognum_in2(ii)=0;
log_xc_m(ii)=0;
log_d_theta_m(ii)=0;

% [v_m, phai_m]=get_measurements; log_phai_m(ii)=phai_m;
% PF_wt=exp(-0.5*((v_m-Predict_P(7,:)).^2/var_v+(phai_m-Predict_P(8,:)).^2/var_phai));
xc_m=predi_xc; d_theta_m=predi_d_theta;
PF_wt=exp(-0.5*((xc_m-Predict_Xc).^2/var_w_xc+(d_theta_m-(-0.0133)-Predict_d_theta).^2/var_w_d_theta));
if sum(PF_wt)==0 %meaning the measurement is far way off from the prediction, indicating wrong fitting for this cycle.
    PF_wt=ones(1,P_No); %every particle carries equal weightage
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
Rtmp=[cos(Estimate_theta_mean-0.5*pi) sin(Estimate_theta_mean-0.5*pi); ...
    -sin(Estimate_theta_mean-0.5*pi) cos(Estimate_theta_mean-0.5*pi)];
%     Particle(1:2,:)=[cos(Estimate_theta_mean-0.5*pi) sin(Estimate_theta_mean-0.5*pi) -Estimate_x_mean; ...
%         -sin(Estimate_theta_mean-0.5*pi) cos(Estimate_theta_mean-0.5*pi) -Estimate_z_mean;]*[Particle(1:2,:); ones(1,P_No)];
Particle(1:2,:)=[Rtmp -Rtmp*[Estimate_x_mean;Estimate_z_mean]]*[Particle(1:2,:); ones(1,P_No)];
Particle(3,:)=Particle(3,:)-(Estimate_theta_mean-0.5*pi);
Particle(4:5,:)=Rtmp*Particle(4:5,:);
log_Estimate_xc(ii)=Estimate_xc;
log_Estimate_d_theta(ii)=Estimate_d_theta;


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

endt(ii)=toc(start);
endf(ii)=toc(in_start);
dt=endf(ii);
% % % if log_Estimate_xc(ii)>=0
% % %     de=log_Estimate_xc(ii)-0.7;
% % % else
% % %     de=log_Estimate_xc(ii)+0.7;
% % % end
% % % de_dot=v_m*sin(log_Estimate_d_theta(ii));
% % % k_s=abs(v_m)/L_wheels*3;
% % % s=de_dot+k_s*de;
% % % phi_t=atan(Q_s*atan(s/P_s)+sign(v_m)*3.0*tan(log_Estimate_d_theta(ii))-tan(phi_d));
% % % log_phi_t(ii)=phi_t;
% % % if (phi_t-log_phi_t(ii-1))>dt*OMEGA
% % %     phi_t=log_phi_t(ii-1)+dt*OMEGA;
% % % elseif (phi_t-log_phi_t(ii-1))<-dt*OMEGA
% % %     phi_t=log_phi_t(ii-1)-dt*OMEGA;
% % % end
% % % sendtarget(v_t, phi_t);
xzthetapredict=GA_inner_loop_logging('End');
[log_predi_xc(ii), log_predi_d_theta(ii)]=solve_xc_d_theta(pre_abc,xzthetapredict(1),xzthetapredict(2),xzthetapredict(3));
predi_xc=log_predi_xc(ii); predi_d_theta=log_predi_d_theta(ii);
updateLocation(1,-predi_xc-1.5,act_z(ii)+xzthetapredict(2),pi/2+predi_d_theta);
% [v_m, phai_m]=get_measurements;
% theta0=pi/2+predi_d_theta;
% phi0=phai_m;
% v0=v_m;
% X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; -(predi_xc+1.5)+v0*cos(theta0)*Ts; ...
%     act_z(ii)+xzthetapredict(2)+v0*sin(theta0)*Ts; theta0+-tan(phi0)*v0*Ts/1.28; phi0; v0;phi0;phi0;];
% position=floor((phi0--0.45)/step_size_phi)+floor((theta0+-tan(phi0)*v0*Ts/1.28)/step_size_theta)*round(0.9/step_size_phi)+1;
% [Uonl,~,cost{ii}] = onl_ctrl(position).evaluate(X0);
% logUonl(:,ii)=Uonl(:,1);
% logPosition(ii)=position;
% if isnan(Uonl(1))
% else
%     phi_t=cost{ii}.U(1,:)+cost{ii}.X(7,1:(end-1));
%     v_t=cost{ii}.U(2,:)+cost{ii}.X(8,1:(end-1));
% end
% % sendtarget(v0+d_v_t,phi0+d_phi_t);
% C_sendtargetMPC('Con',phi_t,v_t);

% inner_thread('Con',predi_xc,predi_d_theta,pre_abc,L_wheels,Q_s,P_s,phi_d,v_t,xzthetapredict);
[loop_t, loop_phi, loop_v]=getloop_t_phi_v;
GA_inner_loop_logging('Ini');

% [v_m, phai_m]=get_measurements;
% if v_m~=0
%     v0=v_m+sqrt(var_v).*randn(1,P_No);
% else
%     v0=zeros(1,P_No);
% end
% phai0=phai_m+sqrt(var_phai).*randn(1,P_No);
% Particle(7,:)=v0;
% Particle(8,:)=phai0;
% Predict_P(8,:)=Particle(8,:);
% Predict_P(7,:)=Particle(7,:);
% Predict_P(4,:)=cos(Particle(3,:)).*Particle(7,:);
% Predict_P(5,:)=sin(Particle(3,:)).*Particle(7,:);
% Predict_P(6,:)=-tan(Particle(8,:)).*Particle(7,:)/L_wheels;
% Predict_P(1,:)=Particle(1,:)+Particle(4,:)*dt;
% Predict_P(2,:)=Particle(2,:)+Particle(5,:)*dt;
% Predict_P(3,:)=Particle(3,:)+Particle(6,:)*dt;

Predict_P=C_Predict_P(Particle,loop_t,loop_phi,loop_v);
[Predict_Xc, Predict_d_theta]=C_solve_xc_d_theta(pre_abc, Predict_P(1:3,:));

logParticel{ii}=Particle;
logPredict{ii}=Predict_P;
logpre_abc{ii}=pre_abc;
logPredict_Xc{ii}=Predict_Xc;
logPredict_d_theta{ii}=Predict_d_theta;

