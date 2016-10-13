clear all;
Ts=0.1; deno_phi=1+23.15*Ts+252.3*Ts*Ts; deno_v=1+2.064*Ts+7.82*Ts*Ts;
L_wheels=1.28;
v1=0.00; phi1=0.00;
ini_xc_pool=[-2.55:0.2:-1.95];
% ini_xc_pool=[-1.79:0.05:-1.5];
% ini_xc_pool=[-1.79:0.1:-1.21];
ini_d_theta_pool=[-0.4:0.1:-0.1];
% ini_d_theta_pool=[0.11:0.1:0.41];
% ini_d_theta_pool=[-0.088:0.02:-0.008];
% ini_d_theta_pool=[0.008:0.02:0.088];
% ini_d_theta_pool=[-0.088:0.025:0.088];
m=0;
for i=1:length(ini_xc_pool)
    ini_xc=ini_xc_pool(i);
    for j=1:length(ini_d_theta_pool)
        m=m+1;
        ini_d_theta=ini_d_theta_pool(j);
        x=-(ini_xc+1.5);theta=pi/2+ini_d_theta; 
        z=0; xdot=cos(theta)*v1; zdot=sin(theta)*v1; thetadot=-tan(phi1)*v1/L_wheels;
        v2=v1;v3=v2; phi2=phi1; phi3=phi2; phidumy=phi1;
        X=[x z theta xdot zdot thetadot v1 v2 v3 phi1 phi2 phi3];
        abc=[0 0 -1.5];position=1;
        tic;
        [~,logposition(m),logw(m,:),log_f_xc(m),log_f_d_theta(m)]=GA_Solver(X, ini_xc, ini_d_theta, abc, 0, Ts);
        logt(m)=toc;
    end
end

figure;plot(logw(:,1))
hold on;plot(logw(:,2),'r.')
figure;plot(log_f_xc+1.5); hold on;
plot(.3*ones(1,m),'r--'); plot(-.3*ones(1,m),'r--');
figure;plot(log_f_d_theta); hold on;
plot(-0.09*ones(1,m),'r--'); plot(0.09*ones(1,m),'r--');
figure;plot(logposition);
% figure;plot(logt);