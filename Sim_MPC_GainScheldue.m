%% Initialization
% clear all;
prompt='New Model? Y/N: ';
result=input(prompt,'s');
if ~isempty(result)
    if result=='y' || result=='Y'
        newmodel=1;
        disp('Using new model');
    elseif result=='n' || result=='N'
        newmodel=0;
        disp('Using previous model');
    else
        disp('Not a valid command');
        return;
    end
else
    newmodel=0;
    disp('Using previous model');
end
act_x=[]; act_z=[]; act_theta=[];act_phi=[];act_xc=[]; act_d_theta=[]; act_v=[]; log_xztheta=[];

% onl_ctrl(position).optimizer.toMatlab('D:\Stereo\MPC\mycontroller1.m','evaluate','first-region');

ini_xc=-1.7; ini_d_theta=0.25;
z_dumy=[0, 60]; x_dumy=[1.5 1.5];

Ts=0.03; v0=10;
counter=0;
horizon = 8;
min_mat=[-1;-1;-0.25;-6;-1;0;-0.45;v0-1.7; -0.45; -0.45]; %for Ts=0.05
% min_mat=[-4;-4;-1.3;-6;-1;0;-0.45;v0-0.7]; %for Ts=0.3
max_mat=-min_mat;max_mat(5)=100;max_mat(6)=pi;max_mat(8)=v0+1.7;
step_size_theta=pi/17;
step_size_phi=0.18;
C=[0 0 0 1 0 0 0 0 0 0;0 0 0 0 0 1 0 0 0 0;0 0 0 0 0 0 0 1 0 0]; g=[0;-pi/2;-v0];
deno=1+23.15*Ts+252.3*Ts*Ts;

if newmodel==1
for i=0:step_size_theta:(pi-step_size_theta)
    theta0=i+step_size_theta/2;
    A=[1 0 -sin(theta0)*v0*Ts 0 0 0 0 0 0 0; ...
        0 1 cos(theta0)*v0*Ts 0 0 0 0 0 0 0; ...
        0 0 1 0 0 0 0 0 0 0; ...
        1 0 0 1 0 0 0 0 0 0; ...
        0 1 0 0 1 0 0 0 0 0; ...
        0 0 1 0 0 1 0 0 0 0; ...
        0 0 0 0 0 0 252.2*Ts^2/deno 0 (2+23.15*Ts)/deno -1/deno; ...
        0 0 0 0 0 0 0 1 0 0; ...
        0 0 0 0 0 0 1 0 0 0; ...
        0 0 0 0 0 0 0 0 1 0;];
    for j=-0.45:step_size_phi:(0.45-step_size_phi)
        counter=counter+1;
        phi0=j+step_size_phi/2;
        B=[0 0 -v0*Ts/(1.28*cos(phi0)^2) 0 0 0 252.2*Ts^2/deno 0 0 0; ...
            Ts*cos(theta0) Ts*sin(theta0) Ts*(-tan(phi0)/1.28) 0 0 0 0 1 0 0]';
        sys(counter)=LTISystem('A',A,'B',B,'C',C,'g',g,'Ts',Ts);
        sys(counter).u.min=[-0.026; -1.13];
        sys(counter).u.max=[0.026; 1.13];
        sys(counter).x.min=min_mat;
        sys(counter).x.max=max_mat;
        onl_ctrl(counter)=MPCController(sys(counter), horizon);
        onl_ctrl(counter).model.y.penalty = OneNormFunction(diag([10 8.5 1])); %  for v0<=10
%         onl_ctrl(counter).model.y.penalty = OneNormFunction(diag([10 15 1])); % for v0 around 15
%         onl_ctrl(counter).model.y.penalty = OneNormFunction(diag([10 17.7 1])); % for v0 around 20
        onl_ctrl(counter).model.u.penalty = OneNormFunction(3*eye(2));
        X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; -(ini_xc+1.5); 0; theta0; phi0; v0; phi0;phi0];
        Uonl=onl_ctrl(counter).evaluate(X0);
    end
end
end
% % pwa=PWASystem(sys);
% % pwa.u.min=[-0.026; -0.0113];
% % pwa.u.max=[0.026; 0.0113];        
% % 
% % pwa.x.min=min_mat;
% % pwa.x.max=max_mat;
% % 
% % onl_ctrl = MPCController(pwa, horizon);
% % onl_ctrl.model.y.penalty = OneNormFunction(diag([1 10]));
% % onl_ctrl.model.u.penalty = OneNormFunction(5*eye(2));
sendtarget(v0,0);
% EMPCController
global_varibles;
InitializationGlobalVariable;
P_s=0.2; Q_s=0.5;

%car coordinate:    ini_xc, ini_d_theta, plotCar, abc, VehicleSimulator,
%inner_thread, act_*, mea_d_theta, mea_xc,
%global coordin:    z_dumy, x_dumy, plot, 

% ini_xc=-1.2; ini_d_theta=0.0863;
% z_dumy=[0, 60]; x_dumy=[1.5 1.5];
figure;plot(x_dumy,z_dumy); hold on; plotCar(pi-(pi/2+ini_d_theta),(ini_xc+1.5),0,0); hold off;axis equal;
% abc=[0 tan(ini_d_theta) ini_xc/cos(ini_d_theta)];
theta0=pi/2+ini_d_theta; phi0=0;
% X0=[0; v_ini*Ts; -tan(phi0)*v_ini*Ts/1.28; 0; 0; theta0; phi0; v_ini;];
X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; -(ini_xc+1.5); 0; theta0; phi0; v0;phi0;phi0;];
position=floor((phi0--0.45)/step_size_phi)+floor(theta0/step_size_theta)*round(0.9/step_size_phi)+1;
[Uonl,~,cost{1}] = onl_ctrl(position).evaluate(X0);
sendtarget(v0+Uonl(2),phi0+Uonl(1));

x0=-(ini_xc+1.5);
%% purely based on mpc.
% % % % % % VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta, x0, 0, theta0, phi0, v0);pause(0.03);
% % % % % % dt=0.3;
% % % % % % for i=1:150
% % % % % %     tic;
% % % % % %     [act_x(i), act_z(i), act_theta(i), act_phi(i), act_xc(i), act_d_theta(i), act_v(i)]=getActPose;
% % % % % %     plot(x_dumy,z_dumy); hold on;plot(-act_x,act_z,'r'); plotCar(pi-act_theta(i),-act_x(i),act_z(i),act_phi(i)); hold off;axis equal;
% % % % % %     mea_d_theta(i)=act_d_theta(i);
% % % % % %     mea_xc(i)=act_xc(i);  
% % % % % %     theta0=pi/2+mea_d_theta(i);
% % % % % %     phi0=act_phi(i);
% % % % % %     v0=act_v(i);
% % % % % %     X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; -(mea_xc(i)+1.5)+v0*cos(theta0)*Ts; ...
% % % % % %         act_z(i)+v0*sin(theta0)*Ts; theta0+-tan(phi0)*v0*Ts/1.28; phi0; v0;phi0;phi0;];
% % % % % %     position=floor((phi0--0.45)/step_size_phi)+floor(theta0/step_size_theta)*round(0.9/step_size_phi)+1;
% % % % % %     [Uonl,~,cost{i}] = onl_ctrl(position).evaluate(X0);
% % % % % %     logUonl(:,i)=Uonl(:,1);
% % % % % %     logPosition(i)=position;
% % % % % %     if isnan(Uonl(1))
% % % % % %     else
% % % % % %         d_phi_t=Uonl(1);
% % % % % %         d_v_t=Uonl(2);
% % % % % %     end
% % % % % %     sendtarget(v0+d_v_t,phi0+d_phi_t);
% % % % % %     pause(0.001);
% % % % % %     logt(i)=toc;
% % % % % %     dt=dt+logt(i);
% % % % % %     
% % % % % % end

%% Running MPC, but the actual position is updated every 250ms. For iterations within 250ms, MPC will be calculated based on the model prediction
VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta, x0, 0, theta0, phi0, v0);
tic;
inner_loop_logging_w_initial('Ini',[x0,0,theta0]);pause(0.03);
dt=toc;tic;
for i=1:150
    [act_x(i), act_z(i), act_theta(i), act_phi(i), act_xc(i), act_d_theta(i), act_v(i)]=getActPose;
    plot(x_dumy,z_dumy); hold on;plot(-act_x,act_z,'r'); plotCar(pi-act_theta(i),-act_x(i),act_z(i),act_phi(i)); hold off;axis equal;
    xztheta=inner_loop_logging_w_initial('End',[0,0,0]);
    log_xztheta(i,:)=xztheta;
    hold on;plot(-log_xztheta(:,1),log_xztheta(:,2),'g--');hold off;
    if dt<0.25
        inner_loop_logging_w_initial('Ini', xztheta);
        x0=xztheta(1);
        z0=xztheta(2);
        theta0=xztheta(3);
    else
        inner_loop_logging_w_initial('Ini', [act_x(i),act_z(i),act_theta(i)]);
        x0=act_x(i);
        z0=act_z(i);
        theta0=act_theta(i);
        dt=0;
    end
    
    phi0=act_phi(i);
    v0=act_v(i);
    X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; x0+v0*cos(theta0)*Ts; ...
        z0+v0*sin(theta0)*Ts; theta0+-tan(phi0)*v0*Ts/1.28; phi0; v0;phi0;phi0;];
    position=floor((phi0--0.45)/step_size_phi)+floor(theta0/step_size_theta)*round(0.9/step_size_phi)+1;
    [Uonl,~,cost{i}] = onl_ctrl(position).evaluate(X0);
    logUonl(:,i)=Uonl(:,1);
    logPosition(i)=position;
    if isnan(Uonl(1))
    else
        d_phi_t=Uonl(1);
        d_v_t=Uonl(2);
    end
    sendtarget(v0+d_v_t,phi0+d_phi_t);
    pause(0.001);
    logt(i)=toc;
    dt=dt+logt(i);
    logdt(i)=dt;
    tic;
end

%% Control through the MPC planned horizon (every 200ms rerun MPC)
% % % % % % % % % % % % % phi_t=cost{1}.U(1,:)+cost{1}.X(7,1:(end-1));
% % % % % % % % % % % % % v_t=cost{1}.U(2,:)+cost{1}.X(8,1:(end-1));
% % % % % % % % % % % % % C_sendtargetMPC('Ini',phi_t,v_t);
% % % % % % % % % % % % % VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta, x0, 0, theta0, phi0, v0);pause(0.03);
% % % % % % % % % % % % % dt=0.3;
% % % % % % % % % % % % % for i=1:150
% % % % % % % % % % % % %     tic;
% % % % % % % % % % % % % %     pause(0.45);
% % % % % % % % % % % % %     [act_x(i), act_z(i), act_theta(i), act_phi(i), act_xc(i), act_d_theta(i), act_v(i)]=getActPose;
% % % % % % % % % % % % %     plot(x_dumy,z_dumy); hold on;plot(-act_x,act_z,'r'); plotCar(pi-act_theta(i),-act_x(i),act_z(i),act_phi(i)); hold off;axis equal;
% % % % % % % % % % % % % %     mea_d_theta(i)=act_d_theta(i)+normrnd(0,0.0334,[1 1]);
% % % % % % % % % % % % % %     mea_xc(i)=act_xc(i)+normrnd(0,0.0914,[1 1]);
% % % % % % % % % % % % %     mea_d_theta(i)=act_d_theta(i);
% % % % % % % % % % % % %     mea_xc(i)=act_xc(i);
% % % % % % % % % % % % % %     abc=[0 tan(mea_d_theta(i)) mea_xc(i)/cos(mea_d_theta(i))];
% % % % % % % % % % % % % %     inner_thread('Con',mea_xc(i), mea_d_theta(i),abc,L_wheels,Q_s,P_s,0,2.0);    
% % % % % % % % % % % % %     theta0=pi/2+mea_d_theta(i);
% % % % % % % % % % % % %     phi0=act_phi(i);
% % % % % % % % % % % % %     v0=act_v(i);
% % % % % % % % % % % % %     X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; -(mea_xc(i)+1.5)+v0*cos(theta0)*Ts; ...
% % % % % % % % % % % % %         act_z(i)+v0*sin(theta0)*Ts; theta0+-tan(phi0)*v0*Ts/1.28; phi0; v0;phi0;phi0;];
% % % % % % % % % % % % %     position=floor((phi0--0.45)/step_size_phi)+floor(theta0/step_size_theta)*round(0.9/step_size_phi)+1;

% % % % % % % % % % % % %     if dt>0.2
% % % % % % % % % % % % %         dt=0;
% % % % % % % % % % % % %     [Uonl,~,cost{i}] = onl_ctrl(position).evaluate(X0);
% % % % % % % % % % % % %     logUonl(:,i)=Uonl(:,1);
% % % % % % % % % % % % %     logPosition(i)=position;
% % % % % % % % % % % % %     if isnan(Uonl(1))
% % % % % % % % % % % % %     else
% % % % % % % % % % % % %         phi_t=cost{i}.U(1,:)+cost{i}.X(7,1:(end-1));
% % % % % % % % % % % % %         v_t=cost{i}.U(2,:)+cost{i}.X(8,1:(end-1));
% % % % % % % % % % % % %     end
% % % % % % % % % % % % %     C_sendtargetMPC('Con',phi_t,v_t);
% % % % % % % % % % % % %     end
    
% % % % % % % % % % % % %     pause(0.001);
% % % % % % % % % % % % %     logt(i)=toc;
% % % % % % % % % % % % %     dt=dt+logt(i);
    
% % % % % % % % % % % % % end
%% Ending
% inner_thread('End',-3.0,0,[0 0 -3.0],L_wheels,0.5,0.2,0,0.6);
hold on; plotCar(pi-(pi/2+ini_d_theta),(ini_xc+1.5),0,0); hold off;
hold on;plot(x_dumy-3, z_dumy,'b'); hold off;
VehicleSimulatorMPC('End', -5, 0,0,0,0,0,0,0);
C_sendtargetMPC('End',phi_t,v_t);
inner_loop_logging_w_initial('End',[0,0,0]);