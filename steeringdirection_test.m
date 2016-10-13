sendtarget(0,0);
phi_t=0; sgn=1; delta_phi=0.03;
rand('state',3);
%% constant omega test
% for i=1:100
%     tic;
%     d_phi=delta_phi;
%     phi_t=phi_t+sgn*d_phi;
%     if phi_t>0.35
%         sgn=-1;
%     elseif phi_t<-0.35
%         sgn=1;
%     end
%     log_phi_t(i)=phi_t;
%     sendtarget(0,phi_t);
%     pause(0.05);
%     [~, log_phi_a(i)]=get_measurements;
%     logt(i)=toc;
% end
% figure;plot(cumsum(logt),log_phi_t);
% hold on;plot(cumsum(logt),log_phi_a,'r');

%% omega sign is constant, but value is random
% for i=1:100
%     tic;
%     d_phi=rand(1)*delta_phi;
%     phi_t=phi_t+sgn*d_phi;
%     if phi_t>0.35
%         sgn=-1;
%     elseif phi_t<-0.35
%         sgn=1;
%     end
%     log_phi_t(i)=phi_t;
%     log_d_phi(i)=d_phi;
%     sendtarget(0,phi_t);
%     pause(0.05);
%     [~, log_phi_a(i)]=get_measurements;
%     logt(i)=toc;
% end
% figure;plot(cumsum(logt),log_phi_t);
% hold on;plot(cumsum(logt),log_phi_a,'r');

%% omega sign is random, but value is constant
% for i=1:100
%     tic;
%     d_phi=delta_phi;
%     if rand(1)<0.5
%         sgn=1;
%     else sgn=-1;
%     end
%     phi_t=phi_t+sgn*d_phi;
%     if phi_t>0.35
%         sgn=-1;
%     elseif phi_t<-0.35
%         sgn=1;
%     end
%     log_phi_t(i)=phi_t;
%     log_d_phi(i)=d_phi;
%     sendtarget(0,phi_t);
%     pause(0.05);
%     [~, log_phi_a(i)]=get_measurements;
%     logt(i)=toc;
% end
% figure;plot(cumsum(logt),log_phi_t);
% hold on;plot(cumsum(logt),log_phi_a,'r');

%% omega both value and sign are random
for i=1:100
    tic;
    d_phi=rand(1)*delta_phi;
    if rand(1)<0.5
        sgn=1;
    else sgn=-1;
    end
    phi_t=phi_t+sgn*d_phi;
    if phi_t>0.35
        sgn=-1;
    elseif phi_t<-0.35
        sgn=1;
    end
    log_phi_t(i)=phi_t;
    log_d_phi(i)=d_phi;
    sendtarget(0,phi_t);
    pause(0.05);
    [~, log_phi_a(i)]=get_measurements;
    logt(i)=toc;
end
figure;plot(cumsum(logt),log_phi_t);
hold on;plot(cumsum(logt),log_phi_a,'r');