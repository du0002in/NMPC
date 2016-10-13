
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
min_mat=[-1;-1;-0.25;-3;-1;0;-0.45;v0-0.5];
max_mat=-min_mat;max_mat(5)=100;max_mat(6)=pi;max_mat(8)=v0+0.5;
pwa.x.min=min_mat;
pwa.x.max=max_mat;

v_ini=v0; phi0=0.4;theta0=pi/2;
X0=[0; v_ini*Ts; -tan(phi0)*v_ini*Ts/1.28; 0; 0; theta0; phi0; v_ini;];
% pwa.initialize(X0);
% U=zeros(2, 5);
% data=pwa.simulate(U);

horizon = 5;
onl_ctrl = MPCController(pwa, horizon);
onl_ctrl.model.y.penalty = OneNormFunction(diag([1 10]));
onl_ctrl.model.u.penalty = OneNormFunction(5*eye(2));
Uonl = onl_ctrl.evaluate(X0);

% % phi_nod=[-.35 0 .35]; theta0=pi/2;v0=10; phi0=0.0;
% a=1/1.28./(cos(phi_nod)).^2;
% b=1/1.28*(tan(phi_nod)-phi_nod./(cos(phi_nod)).^2);
% c=[-.3729 -.9003 -.9003 -.3729];
% d=[1 1.4142 1.4142 .1715];
% e=[.9003 .3729 -.3729 -.9003];
% f=[0 .4142 1.5857 2.8284];
% for i=1:4
%     A=[1 0 c(i)*v0*Ts 0 0 0 0 0; ...
%         0 1 e(i)*v0*Ts 0 0 0 0 0; ...
%         0 0 1 0 0 0 0 0; ...
%         1 0 0 1 0 0 0 0; ...
%         0 1 0 0 1 0 0 0; ...
%         0 0 1 0 0 1 0 0; ...
%         0 0 0 0 0 0 1 0; ...
%         0 0 0 0 0 0 0 1;];
%     for j=1:3
%         B=[0 0 -a(j)*v0*Ts 0 0 0 1 0; ...
%             Ts*(c(i)*theta0+d(i)) Ts*(e(i)*theta0+f(i)) Ts*(-a(j)*phi0-b(j)) 0 0 0 0 1]';
%         sys(j+(i-1)*3)=LTISystem('A',A,'B',B,'C',C,'g',g,'Ts',Ts);
%         if j==1
%             sys(j+(i-1)*3).setDomain('x',Polyhedron('A',[0 0 0 0 0 1 0 0; 0 0 0 0 0 -1 0 0; ...
%                 0 0 0 0 0 0 1 0],'b',[pi/4*i;-pi/4*(i-1);-.2372]));
%         elseif j==2
%             sys(j+(i-1)*3).setDomain('x',Polyhedron('A',[0 0 0 0 0 1 0 0; 0 0 0 0 0 -1 0 0; ...
%                 0 0 0 0 0 0 1 0;0 0 0 0 0 0 -1 0],'b',[pi/4*i;-pi/4*(i-1);.2372;.2372]));
%         else
%             sys(j+(i-1)*3).setDomain('x',Polyhedron('A',[0 0 0 0 0 1 0 0; 0 0 0 0 0 -1 0 0; ...
%                 0 0 0 0 0 0 -1 0],'b',[pi/4*i;-pi/4*(i-1);-.2372]));
%         end
%     end
% end
% pwa=PWASystem(sys);
% pwa.u.min=[-0.026; -0.0113];
% pwa.u.max=[0.026; 0.0113];
% % min_mat=-ones(8,1)*inf; min_mat(7)=-0.45; min_mat(8)=v0-0.5;
% % max_mat=ones(8,1)*inf; max_mat(7)=0.45; max_mat(8)=v0+0.5;
% min_mat=[-1;-1;-0.1227;-3;-1;0;-0.45;v0-0.5];
% max_mat=-min_mat;max_mat(5)=100;max_mat(6)=pi;max_mat(8)=v0+0.5;
% 
% 
% pwa.x.min=min_mat;
% pwa.x.max=max_mat;
% 
% horizon = 5;
% onl_ctrl = MPCController(pwa, horizon);
% onl_ctrl.model.y.penalty = OneNormFunction(diag([1 10]));
% onl_ctrl.model.u.penalty = OneNormFunction(50*eye(2));
% 
% % X0=[0; v0*Ts; -tan(phi0)*v0*Ts/1.28; 0; 0; theta0; phi0; v0;];
% v_ini=v0;
% X0=[0; v_ini*Ts; -tan(phi0)*v_ini*Ts/1.28; 0; 0; theta0; phi0; v_ini;];
% % pwa.initialize(X0);
% % U=zeros(2, 5);
% % data=pwa.simulate(U);
% Uonl = onl_ctrl.evaluate(X0);

