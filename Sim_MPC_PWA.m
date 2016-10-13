clear all;

Ts=0.05; v0=10;
counter=0;

step_size_theta=pi/18;
step_size_phi=0.18;
C=[0 0 0 1 0 0 0 0;0 0 0 0 0 1 0 0]; g=[0;-pi/2];
for i=0:step_size_theta:(pi-step_size_theta)
    theta0=i+step_size_theta/2;
    A=[1 0 -sin(theta0)*v0*Ts 0 0 0 0 0; ...
        0 1 cos(theta0)*v0*Ts 0 0 0 0 0; ...
        0 0 1 0 0 0 0 0; ...
        1 0 0 1 0 0 0 0; ...
        0 1 0 0 1 0 0 0; ...
        0 0 1 0 0 1 0 0; ...
        0 0 0 0 0 0 1 0; ...
        0 0 0 0 0 0 0 1;];
    for j=-0.45:step_size_phi:(0.45-step_size_phi)
        counter=counter+1;
        phi0=j+step_size_phi/2;
        B=[0 0 -v0*Ts/(1.28*cos(phi0)^2) 0 0 0 1 0; ...
            Ts*cos(theta0) Ts*sin(theta0) Ts*(-tan(phi0)/1.28) 0 0 0 0 1]';
        sys(counter)=LTISystem('A',A,'B',B,'C',C,'g',g,'Ts',Ts);
        sys(counter).setDomain('x',Polyhedron('A',[0 0 0 0 0 1 0 0; 0 0 0 0 0 -1 0 0; ...
                0 0 0 0 0 0 1 0;0 0 0 0 0 0 -1 0],'b',[i+step_size_theta;-i;j+step_size_phi;-j]));
    end
end
pwa=PWASystem(sys);
pwa.u.min=[-0.026; -0.0113];
pwa.u.max=[0.026; 0.0113];        
min_mat=[-1;-1;-0.25;-6;-1;0;-0.45;v0-0.7]; %for Ts=0.05
% min_mat=[-4;-4;-1.3;-6;-1;0;-0.45;v0-0.7]; %for Ts=0.3
max_mat=-min_mat;max_mat(5)=100;max_mat(6)=pi;max_mat(8)=v0+0.7;
pwa.x.min=min_mat;
pwa.x.max=max_mat;
horizon = 5;
onl_ctrl = MPCController(pwa, horizon);
onl_ctrl.model.y.penalty = OneNormFunction(diag([1 10]));
onl_ctrl.model.u.penalty = OneNormFunction(5*eye(2));

sendtarget(v0,0);



global_varibles;
InitializationGlobalVariable;
P_s=0.2; Q_s=0.5;

%car coordinate:    ini_xc, ini_d_theta, plotCar, abc, VehicleSimulator,
%inner_thread, act_*, mea_d_theta, mea_xc,
%global coordin:    z_dumy, x_dumy, plot, 

ini_xc=-1.7; ini_d_theta=-0.0863;
z_dumy=[0, 60]; x_dumy=[1.5 1.5];
figure;plot(x_dumy,z_dumy); hold on; plotCar(pi-(pi/2+ini_d_theta),(ini_xc+1.5),0,0); hold off;axis equal;
% abc=[0 tan(ini_d_theta) ini_xc/cos(ini_d_theta)];
theta0=pi/2+ini_d_theta; phi0=0;
% X0=[0; v_ini*Ts; -tan(phi0)*v_ini*Ts/1.28; 0; 0; theta0; phi0; v_ini;];
X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; -(ini_xc+1.5); 0; theta0; phi0; v0];
Uonl = onl_ctrl.evaluate(X0);
sendtarget(v0+Uonl(2),phi0+Uonl(1));
x0=-(ini_xc+1.5);
VehicleSimulatorMPC('Ini',ini_xc, ini_d_theta, x0, 0, theta0, phi0, v0);pause(0.02);
% inner_thread('Ini',ini_xc,ini_d_theta,abc,L_wheels,Q_s,P_s,0,2.0);
for i=1:100
    tic;
%     pause(0.45);
    [act_x(i), act_z(i), act_theta(i), act_phi(i), act_xc(i), act_d_theta(i)]=getActPose;
    plot(x_dumy,z_dumy); hold on;plot(-act_x,act_z,'r'); plotCar(pi-act_theta(i),-act_x(i),act_z(i),act_phi(i)); hold off;axis equal;
%     mea_d_theta(i)=act_d_theta(i)+normrnd(0,0.0334,[1 1]);
%     mea_xc(i)=act_xc(i)+normrnd(0,0.0914,[1 1]);
    mea_d_theta(i)=act_d_theta(i);
    mea_xc(i)=act_xc(i);
%     abc=[0 tan(mea_d_theta(i)) mea_xc(i)/cos(mea_d_theta(i))];
%     inner_thread('Con',mea_xc(i), mea_d_theta(i),abc,L_wheels,Q_s,P_s,0,2.0);    
    theta0=pi/2+mea_d_theta(i);
    phi0=act_phi(i);
    v0=v0;
    X0=[v0*cos(theta0)*Ts; v0*sin(theta0)*Ts; -tan(phi0)*v0*Ts/1.28; -(mea_xc(i)+1.5); act_z(i); theta0; phi0; v0];
    [Uonl,~,cost{i}] = onl_ctrl.evaluate(X0);
    logUonl(:,i)=Uonl(:,1);
    sendtarget(v0+Uonl(2),phi0+Uonl(1));
    pause(0.001);
    logt(i)=toc;
end

VehicleSimulatorMPC('End', -5, 0,0,0,0,0,0,0);
% inner_thread('End',-3.0,0,[0 0 -3.0],L_wheels,0.5,0.2,0,0.6);
hold on; plotCar(pi-(pi/2+ini_d_theta),(ini_xc+1.5),0,0); hold off;
hold on;plot(x_dumy-3.5, z_dumy,'b'); hold off;