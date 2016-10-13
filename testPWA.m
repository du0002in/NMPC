% Ts=1;A=1;B=Ts;D=0;
% 
% C1=0.5; g1=0;
% sys1=LTISystem('A',A,'B',B,'C',C1,'D',D,'g',g1,'Ts',Ts);
% C2=-0.5; g2=0;
% sys2=LTISystem('A',A,'B',B,'C',C2,'D',D,'g',g2,'Ts',Ts);
% C3=2; g3=-15;
% sys3=LTISystem('A',A,'B',B,'C',C3,'D',D,'g',g3,'Ts',Ts);
% C4=-2; g4=-15;
% sys4=LTISystem('A',A,'B',B,'C',C4,'D',D,'g',g4,'Ts',Ts);
% 
% sys1.setDomain('x',Polyhedron('A',[1;-1],'b',[10;0]));
% sys2.setDomain('x',Polyhedron('A',[1;-1],'b',[0;10]));
% sys3.setDomain('x',Polyhedron('A',[-1],'b',[-10]));
% sys4.setDomain('x',Polyhedron('A',[1],'b',[-10]));
% 
% pwa = PWASystem([sys1 sys2 sys3 sys4]);
% 
% x0=-20;
% pwa.initialize(x0);
% U=ones(1,50);
% data=pwa.simulate(U);

clear all;
Ts=0.05;
phi_nod=[-.35 0 .35]; theta0=pi/2;v0=10; phi0=0.0;
a=1/1.28./(cos(phi_nod)).^2;
b=1/1.28*(tan(phi_nod)-phi_nod./(cos(phi_nod)).^2);
c=[-.3729 -.9003 -.9003 -.3729];
d=[1 1.4142 1.4142 .1715];
e=[.9003 .3729 -.3729 -.9003];
f=[0 .4142 1.5857 2.8284];
C=[0 0 0 1 0 0 0 0;0 0 0 0 0 1 0 0]; g=[0;-pi/2];
for i=1:4
    A=[1 0 c(i)*v0*Ts 0 0 0 0 0; ...
        0 1 e(i)*v0*Ts 0 0 0 0 0; ...
        0 0 1 0 0 0 0 0; ...
        1 0 0 1 0 0 0 0; ...
        0 1 0 0 1 0 0 0; ...
        0 0 1 0 0 1 0 0; ...
        0 0 0 0 0 0 1 0; ...
        0 0 0 0 0 0 0 1;];
    for j=1:3
        B=[0 0 -a(j)*v0*Ts 0 0 0 1 0; ...
            Ts*(c(i)*theta0+d(i)) Ts*(e(i)*theta0+f(i)) Ts*(-a(j)*phi0-b(j)) 0 0 0 0 1]';
        sys(j+(i-1)*3)=LTISystem('A',A,'B',B,'C',C,'g',g,'Ts',Ts);
        if j==1
            sys(j+(i-1)*3).setDomain('x',Polyhedron('A',[0 0 0 0 0 1 0 0; 0 0 0 0 0 -1 0 0; ...
                0 0 0 0 0 0 1 0],'b',[pi/4*i;-pi/4*(i-1);-.2372]));
        elseif j==2
            sys(j+(i-1)*3).setDomain('x',Polyhedron('A',[0 0 0 0 0 1 0 0; 0 0 0 0 0 -1 0 0; ...
                0 0 0 0 0 0 1 0;0 0 0 0 0 0 -1 0],'b',[pi/4*i;-pi/4*(i-1);.2372;.2372]));
        else
            sys(j+(i-1)*3).setDomain('x',Polyhedron('A',[0 0 0 0 0 1 0 0; 0 0 0 0 0 -1 0 0; ...
                0 0 0 0 0 0 -1 0],'b',[pi/4*i;-pi/4*(i-1);-.2372]));
        end
    end
end
pwa=PWASystem(sys);
pwa.u.min=[-0.026; -0.0113];
pwa.u.max=[0.026; 0.0113];
% min_mat=-ones(8,1)*inf; min_mat(7)=-0.45; min_mat(8)=v0-0.5;
% max_mat=ones(8,1)*inf; max_mat(7)=0.45; max_mat(8)=v0+0.5;
min_mat=[-1;-1;-0.1227;-3;-1;0;-0.45;v0-0.5];
max_mat=-min_mat;max_mat(5)=100;max_mat(6)=pi;max_mat(8)=v0+0.5;


pwa.x.min=min_mat;
pwa.x.max=max_mat;

horizon = 5;
onl_ctrl = MPCController(pwa, horizon);
onl_ctrl.model.y.penalty = OneNormFunction(diag([1 10]));
onl_ctrl.model.u.penalty = OneNormFunction(50*eye(2));

% X0=[0; v0*Ts; -tan(phi0)*v0*Ts/1.28; 0; 0; theta0; phi0; v0;];
v_ini=v0;
X0=[0; v_ini*Ts; -tan(phi0)*v_ini*Ts/1.28; 0; 0; theta0; phi0; v_ini;];
% pwa.initialize(X0);
% U=zeros(2, 5);
% data=pwa.simulate(U);
Uonl = onl_ctrl.evaluate(X0);

