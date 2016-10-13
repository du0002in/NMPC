% GA_Sim_MPC
clear all;
terminate_all_threads;
lane_width=3.5;
% MapGeneration_PlayGround;
MapGeneration;
get_status('Ini');
sendtarget(0,0);
global_varibles;
InitializationGlobalVariable;
Ts=0.1; deno_phi=1+23.15*Ts+252.3*Ts*Ts; deno_v=1+2.064*Ts+7.82*Ts*Ts;
ini_xc=-2; ini_d_theta=-0.08; 
x=-(ini_xc+0.5*lane_width);theta=pi/2+ini_d_theta; phi1=-0.0; v1=0.00;
z=0; xdot=cos(theta)*v1; zdot=sin(theta)*v1; thetadot=-tan(phi1)*v1/L_wheels;
v2=v1;v3=v2; phi2=phi1; phi3=phi2; phidumy=phi1;
X=[x z theta xdot zdot thetadot v1 v2 v3 phi1 phi2 phi3];
abc=[0 0 -0.5*lane_width];position=1;
pre_U=zeros(2,15);
[U,~]=GA_Solver(X, ini_xc, ini_d_theta, abc, abc,0, 0, Ts, pre_U, 0,lane_width); horizon=length(U);
v_t=((v2+U(1,1))*deno_v+v3-(2+2.064*Ts)*v2)/(7.82*Ts*Ts);
phi_t=((phi2+U(2,1))*deno_phi+phi3-(2+23.15*Ts)*phi2)/(252.3*Ts*Ts);
sendtarget(v_t,phi_t);
% z_dumy=[-1, 160, 160 -1]; x_dumy=[1.5 1.5 -1.5 -1.5];
% figure;patch(x_dumy,z_dumy,'g');hold on;
% plot([0 0],[-1 160],'k--','LineWidth',0.5);
plotCar(pi-(pi/2+ini_d_theta),(ini_xc+0.5*lane_width),0,phi1); hold off; axis equal;
dumyv_t=v_t; dumyphi_t=phi_t;
%% Everything is the same as the case above, except that this one is in cam cord.
S=[v2 v1 v1+cumsum(U(1,:))];
V_t=(S(3:end)*deno_v+S(1:end-2)-(2+2.064*Ts)*S(2:end-1))/(7.82*Ts*Ts);
Phi=[phi2 phi1 phi1+cumsum(U(2,:))];
Phi_t=(Phi(3:end)*deno_phi+Phi(1:end-2)-(2+23.15*Ts)*Phi(2:end-1))/(252.3*Ts*Ts);
GA_VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta, x, z, theta, phi1, v1, abc);
dt_st=tic;dt=0;
pause(0.01);
loop_t=Ts;
GA_inner_loop_logging_w_initial('Ini',[x z theta]);
C_MPCsendtarget('Ini',[V_t; Phi_t],Ts);
pre_U=[U(:,(position+1):horizon) zeros(2,1:position)];
i=0;
stop_all_flag=1;
while stop_all_flag==1
    stop_all_flag=get_stop_all_flag;
    i=i+1;
    st=tic;
    [act_x(i), act_z(i), act_theta(i), act_phi(i), ~, ~, act_v(i)]=GA_getActPose;
%     [~, act_phi(i)]=get_measurements;
%     [act_phi2(:,i),~]=get_log_phi_v;
    IDX=get_nearest_map_point(act_x(i),act_z(i),map_x,map_z);
    if (IDX+15)>length(map_x)
        act_x(i)=[]; act_z(i)=[]; act_theta(i)=[]; act_phi(i)=[]; act_v(i)=[];
        break;
    end
    sel_map_x=map_x(IDX:(IDX+15));
    sel_map_z=map_z(IDX:(IDX+15));
    rot_theta=act_theta(i)-pi/2;
    R=[cos(rot_theta) sin(rot_theta); ...
        -sin(rot_theta) cos(rot_theta)];
    sel_map_car=R*([sel_map_x-act_x(i); sel_map_z-act_z(i)]);
    sel_map_car_x=sel_map_car(1,:);
    sel_map_car_z=sel_map_car(2,:);
    A=[sel_map_car_z'.^2 sel_map_car_z' ones(length(sel_map_car_z),1)];
    B=sel_map_car_x';
    act_abc=pinv(A)*B;
    [act_xc(i),act_d_theta(i),~]=solve_xc_d_theta(act_abc,0,0,pi/2);
    
    plot(-ctr_x,ctr_z,'k--','LineWidth',0.5);hold on;
    patch(-[map_x map_x_bar],[map_z map_z_bar],'g');
    plot(-ctr_x,ctr_z,'k--','LineWidth',0.5);
    plotCar(pi-act_theta(i),-act_x(i),act_z(i),act_phi(i)); hold off;axis equal;
    
    xztheta=GA_inner_loop_logging_w_initial('End',[0,0,0]);
    dt_endt=toc(dt_st);
    dt=dt+dt_endt;
    dt_st=tic;
    if dt<0.45
        GA_inner_loop_logging_w_initial('Ini',xztheta);
        x=xztheta(1); z=xztheta(2); theta=xztheta(3);
    else
        GA_inner_loop_logging_w_initial('Ini',[0 0 pi/2]);
        x=0; z=0; theta=pi/2;
        dt=0;
        abc=act_abc;
    end
    logx(i)=x; logz(i)=z; logtheta(i)=theta; logdt(i)=dt;
    
    
    
%     xc=-x-1.5; d_theta=theta-pi/2;
    %convert to cam coordinate
%     abc=[0 tan(d_theta) xc/cos(d_theta)];
%     x=0; z=0; theta=pi/2;
    
    [log_phi1(:,i), log_v1(:,i)]=get_log_phi_v;
    phi1=log_phi1(3,i); v1=log_v1(3,i);
    phi2=log_phi1(2,i); v2=log_v1(2,i);
    phi3=log_phi1(1,i); v3=log_v1(1,i);
    xdot=cos(theta)*v1; zdot=sin(theta)*v1; thetadot=-tan(phi1)*v1/L_wheels;
%     v1=act_v(i); phi1=act_phi(i);
%     phi2=phi1; phi3=phi2; v2=v1; v3=v2;   
        
    x=x+xdot*Ts; z=z+zdot*Ts; theta=theta+thetadot*Ts;
    phi3=phi2;phi2=phi1;v3=v2;v2=v1;
    phi1=(phi_t*252.3*Ts*Ts+(2+23.15*Ts)*phi2-phi3)/deno_phi;
    v1=(v_t*7.82*Ts*Ts+(2+2.064*Ts)*v2-v3)/deno_v;
    xdot=cos(theta)*v1; zdot=sin(theta)*v1; thetadot=-tan(phi1)*v1/L_wheels;
    [xc, d_theta,k]=solve_xc_d_theta(abc,x,z,theta);
    log_k(i)=k;
    X=[x z theta xdot zdot thetadot v1 v2 v3 phi1 phi2 phi3];
    log_GA_solver{i}={X,xc,d_theta, abc, .6*(phi1-phi2),loop_t, pre_U};
    GAt=tic;
    [U,logposition(i),V_max, logw(i,:)]=GA_Solver(X, xc, d_theta, abc, abc, 0, .6*(phi1-phi2),loop_t, pre_U, k,lane_width);
    
%     [U,logposition(i),V_max,log_sss(i,:)]=GA_Solver(X, xc, d_theta, abc1,abc2,z_th, .6*(phi1-phi2),loop_t, pre_U, k,lane_width);
    GAt_e=toc(GAt);
    logGAt(i)=GAt_e;
    S=[v2 v1 v1+cumsum(U(1,:))];
    V_t=(S(3:end)*deno_v+S(1:end-2)-(2+2.064*Ts)*S(2:end-1))/(7.82*Ts*Ts);
    Phi=[phi2 phi1 phi1+cumsum(U(2,:))];
    Phi_t=(Phi(3:end)*deno_phi+Phi(1:end-2)-(2+23.15*Ts)*Phi(2:end-1))/(252.3*Ts*Ts);
    position=min([max([round(logGAt(i)/Ts), 1]),horizon]);
    v_t=V_t(position); phi_t=Phi_t(position);
%     v_t=v1+U(1,1); phi_t=phi1+U(2,1);
    log_d_v_t(i)=U(1,position); log_d_phi_t(i)=U(2,position);
    log_v_t(i)=v_t; log_phi_t(i)=phi_t;
    log_position(i)=position;
%     log_pre_d_phi(i)=pre_d_phi;
    log_pre_d_phi2(i)=phi1-phi2;
    V_Phi_t=[V_t(position:end) V_t(end)*ones(1,position-1); ...
        Phi_t(position:end) Phi_t(end)*ones(1,position-1)];
    C_MPCsendtarget('Con',V_Phi_t,Ts);
    if position<horizon
        pre_U=[U(:,(position+1):horizon) zeros(2,position)];
    else
        pre_U=zeros(2,horizon);
    end 
%     sendtarget(v_t,phi_t);
    loop_t=toc(st);
    logt(i)=loop_t;
    pause(0.001);
end
xztheta=GA_inner_loop_logging_w_initial('End',[0,0,0]);
%% Data Plotting
hold on; plotCar(pi-(pi/2+ini_d_theta),(ini_xc+0.5*lane_width),0,phidumy);
hold on; plot(-act_x,act_z,'r.'); plot(-act_x,act_z,'r')
GA_VehicleSimulatorMPC('End',ini_xc, ini_d_theta, x, z, theta, phi1, v1, abc);
C_MPCsendtarget('End',[V_t(position:end) V_t(end)*ones(1,position-1); ...
        Phi_t(position:end) Phi_t(end)*ones(1,position-1)],Ts);
figure;plot(act_d_theta)
hold on;plot(0.09*ones(1,length(logt)),'r--')
hold on;plot(-0.09*ones(1,length(logt)),'r--')
 title('dtheta')
 figure;plot(act_xc);
hold on;plot(-(0.5*lane_width-0.3)*ones(1,length(logt)),'r--')
hold on;plot(-(0.5*lane_width+0.3)*ones(1,length(logt)),'r--')
title('xc');
%  figure;plot(cumsum(logt),log_phi_t)
% hold on;plot(cumsum(logt),[act_phi(3:end),log_phi_t(end-1:end)],'r')
% legend('phi_t','act_phi')
 figure;plot(log_v_t)
hold on;plot([act_v(3:end), log_v_t(end-1:end)],'r')
 legend('v_t','act_v')
figure;plot(log_d_phi_t); title('d_phi_t');
figure;plot(logt); title('total t');
hold on;plot(logGAt,'r'); title('GA t');
figure;plot(abs(act_v.^2.*tan(act_phi)/1.28));
hold on;plot(1.6*ones(1,length(logt)),'r--');
title('Spd Acc');

 figure;plot(log_phi_t)
hold on;plot([act_phi(3:end),log_phi_t(end-1:end)],'r')
legend('phi_t','act_phi')

log_phi_a_t=act_phi;
%  figure;plot(cumsum(logt),log_phi_t)
% hold on;plot(cumsum(logt),log_phi_a_t,'r')
% title('Phi');
% legend('phi_t','phi_a')
omega_a=log_phi_a_t(2:end)-log_phi_a_t(1:end-1);
% omega_a=[log_phi_a_t(1) omega_a];
omega_a=omega_a./logt(1:end-1);
% figure;plot(cumsum(logt),omega_a)
% title('Phi velocity');
acc_a=omega_a(2:end)-omega_a(1:end-1);
% acc_a=[omega_a(1) acc_a];
acc_a=acc_a./logt(2:end-1);

% jerk_a=acc_a(2:end)-acc_a(1:end-1);
% % jerk_a=[0 jerk_a];
% jerk_a=jerk_a./logt(1:end-1);
figure;plot(acc_a);
title('Phi Acc');
hold on;plot(acc_a,'.r')
 hold on;plot(1.5*ones(1,length(logt)),'r--')
hold on;plot(-1.5*ones(1,length(logt)),'r--')
figure;plot(logposition); title('Position');
figure;plot(log_k); title('Curvature');
% figure;plot(jerk_a); hold on;plot(jerk_a,'.r')
% title('Jerk Phi');
 sendtarget(-0.0,-00.0);