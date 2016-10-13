clear all;
for i=1:100
    [xc, d_theta, x, z]=run_Simulation_for_consistency;
    logxc(:,i)=xc';
    logd_theta(:,i)=d_theta';
    logx(:,i)=x';
    logz(:,i)=z';
    i
end
logxc(1,:)=[];
logd_theta(1,:)=[];
logx(1,:)=[];
logz(1,:)=[];