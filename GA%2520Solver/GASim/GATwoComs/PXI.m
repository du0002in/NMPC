clear all;
sendtarget(0,0);
global_varibles;
InitializationGlobalVariable;
Ts=0.05; deno_phi=1+23.15*Ts+252.3*Ts*Ts; deno_v=1+2.064*Ts+7.82*Ts*Ts;
ini_xc=-2.3; ini_d_theta=-0.186; phi1=0; v1=0;
vehicle_abc=[0 0 -1.5];
x0=-(ini_xc+1.5);theta0=pi/2+ini_d_theta;

updateLocation(0,0,0,0);
runningstatus(0);
closecommunication(0);
changeacturalpose([0, -1, -1, -1, zeros(1,6)]);

prompt='Waiting for Laptop program and the communication between PXI and Laptop. \nPress enter once everything is ready';
result=input(prompt,'s');

GA_VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta, x0, 0, theta0, phi1, v1, vehicle_abc);pause(Ts);
runningstatus(1);
status=0;
while status==0
    [status,x,z,theta]=getPosition;
    pause(0.001);
end
updateLocation(0,x,z,theta);
GA_inner_loop_logging_w_initial('Ini',[x,z,theta]);
xdot=cos(theta)*v1; zdot=sin(theta)*v1; thetadot=-tan(phi1)*v1/L_wheels;
v2=v1;v3=v2; phi2=phi1; phi3=phi2; phidumy=phi1;
X=[x z theta xdot zdot thetadot v1 v2 v3 phi1 phi2 phi3];
abc=vehicle_abc;
[U,~]=GA_Solver(X, ini_xc, ini_d_theta, abc, 0); 
horizon=length(U);
v_t=((v2+U(1,1))*deno_v+v3-(2+2.064*Ts)*v2)/(7.82*Ts*Ts);
phi_t=((phi2+U(2,1))*deno_phi+phi3-(2+23.15*Ts)*phi2)/(252.3*Ts*Ts);

z_dumy=[-1, 160, 160 -1]; x_dumy=[1.5 1.5 -1.5 -1.5];
figure;patch(x_dumy,z_dumy,'g');hold on;
plot([0 0],[-1 160],'k--','LineWidth',0.5);
plotCar(pi-(pi/2+ini_d_theta),(ini_xc+1.5),0,phi1); hold off; axis equal;
dumyv_t=v_t; dumyphi_t=phi_t;
S=[v2 v1 v1+cumsum(U(1,:))];
V_t=(S(3:end)*deno_v+S(1:end-2)-(2+2.064*Ts)*S(2:end-1))/(7.82*Ts*Ts);
Phi=[phi2 phi1 phi1+cumsum(U(2,:))];
Phi_t=(Phi(3:end)*deno_phi+Phi(1:end-2)-(2+23.15*Ts)*Phi(2:end-1))/(252.3*Ts*Ts);
C_MPCsendtarget('Ini',[V_t; Phi_t],Ts);

%mpc cycles. Number of iteration depends on user self selection
j=0;
for i=1:300
    st=tic;
    [act_x(i), act_z(i), act_theta(i), act_phi(i), act_xc(i), act_d_theta(i), act_v(i)]=GA_getActPose;
    plot([0 0],[-1 160],'k--','LineWidth',0.5);hold on;patch(x_dumy,z_dumy,'g');
    plot([0 0],[-1 160],'k--','LineWidth',0.5);
    plotCar(pi-act_theta(i),-act_x(i),act_z(i),act_phi(i)); hold off;axis equal;
    xztheta=GA_inner_loop_logging_w_initial('End',[0,0,0]);
    log_xztheta(i,:)=xztheta;
    [status,xnew,znew,thetanew]=getPosition;
    if status==0
        GA_inner_loop_logging_w_initial('Ini', xztheta);
        x=xztheta(1);
        z=xztheta(2);
        theta=xztheta(3);
    else
        GA_inner_loop_logging_w_initial('Ini', [xnew,znew,thetanew]);
        x=xnew;
        z=znew;
        theta=thetanew;
        updateLocation(0,x,z,theta);
        j=j+1;
        update_x(j)=xnew;
        update_z(j)=znew;
        update_theta(j)=thetanew;
        log_xztheta(i,:)=[xnew,znew,thetanew];
    end
    
    v1=act_v(i); phi1=act_phi(i); 
    phi2=phi1; phi3=phi2; v2=v1; v3=v2;
    xc=-x-1.5; d_theta=theta-pi/2;
    xdot=cos(theta)*v1; zdot=sin(theta)*v1; thetadot=-tan(phi1)*v1/L_wheels;
    
    x=x+xdot*Ts; z=z+zdot*Ts; theta=theta+thetadot*Ts;
    phi3=phi2;phi2=phi1;v3=v2;v2=v1;
    phi1=(phi_t*252.3*Ts*Ts+(2+23.15*Ts)*phi2-phi3)/deno_phi;
    v1=(v_t*7.82*Ts*Ts+(2+2.064*Ts)*v2-v3)/deno_v;
    xdot=cos(theta)*v1; zdot=sin(theta)*v1; thetadot=-tan(phi1)*v1/L_wheels;
    xc=-x-1.5; d_theta=theta-pi/2;
    X=[x z theta xdot zdot thetadot v1 v2 v3 phi1 phi2 phi3];
    GAt=tic;
    [U,logposition(i)]=GA_Solver(X, xc, d_theta, abc, 0.8*(phi1-phi2));
    logGAt(i)=toc(GAt);   
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
    C_MPCsendtarget('Con',[V_t(position:end) V_t(end)*ones(1,position-1); ...
        Phi_t(position:end) Phi_t(end)*ones(1,position-1)],Ts);
    pause(0.001);
    logt(i)=toc(st); 
end
xztheta=GA_inner_loop_logging_w_initial('End',[0,0,0]);
hold on; plotCar(pi-(pi/2+ini_d_theta),(ini_xc+1.5),0,phidumy);
hold on; plot(-act_x,act_z,'r.'); plot(-act_x,act_z,'r');
C_MPCsendtarget('End',[V_t; Phi_t],Ts);
runningstatus(0);
GA_VehicleSimulatorMPC('End',ini_xc, ini_d_theta, x0, 0, theta0, phi1, v1, vehicle_abc);
pause(Ts);closecommunication(1);
figure;plot(act_d_theta)
hold on;plot(0.09*ones(1,350),'r--')
hold on;plot(-0.09*ones(1,350),'r--')
 title('dtheta')
 figure;plot(act_xc);
hold on;plot(-1.2*ones(1,350),'r--')
hold on;plot(-1.8*ones(1,350),'r--')
title('xc');
 figure;plot(log_phi_t)
hold on;plot(act_phi(3:end),'r')
legend('phi_t','act_phi')
 figure;plot(log_v_t)
hold on;plot(act_v(3:end),'r')
 legend('v_t','act_v')
figure;plot(log_d_phi_t)
figure;plot(logt)
hold on;plot(logGAt,'r')
figure;plot(cumsum(logt),abs(act_v.^2.*tan(act_phi)/1.28));
hold on;plot(cumsum(logt),1.6*ones(1,length(logt)),'r--');