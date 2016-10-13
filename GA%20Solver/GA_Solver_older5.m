function [U, position,v_max,sss]=GA_Solver(X0, xc, d_theta, abc1,abc2,z_th, pre_d_phi, pre_T, pre_U, k, lane_width)
%X0=[x z theta xdot zdot thetadot v1 v2 v3 phi1 phi2 phi3]
%abc=> x=a*z^2+b*z+c;
%All inputs are in car coordinate
%U=[delta_v1 delta_v2 ... delta_v_horizon; ...
%     delta_phi1 delta_phi2 ... delta_phi_horizon];
% st=tic;
% % v_max=min([sqrt(1.6/abs(k)) sqrt(1.6*1.28/tan(abs(X0(10))))]);
% % if (v_max>5.0)
% %     v_max=5.0;
% % end
phi1_d=atan(1.28*k);
if phi1_d>0.4
    phi1_d=0.4;
elseif phi1_d<-0.4
    phi1_d=-0.4;
end
horizon=15; % Control horizon/ no. of genes
Ts=0.1; % Ts is the sampling time for MPC
P=50;   %Population size
pm=0.1; %(Initial) Mutation probability
pm_adapt=pm*ones(P,1); %adaptive mutation probability for each chromesome
gamma=0.2; %self-adaptive learning rate
std_xc=-0.5*lane_width; %standard vehicle distance to road center line if assuming lane width is 3m.
% std_v0=v_max; %m/s, standard cruising speed/max speed when travelling
%% Initialization
% 21/1/2015
% % if abs(X0(7))>v_max
% %     max_d_v=2*Ts; %m/s, larger so that spd can b reduced in shorter time
% % else
% %     max_d_v=.5*Ts; %m/s
% % end
max_d_phi=0.02; %radian or 3.44 degree original is 0.03
max_d_d_phi=1.5*Ts*Ts; %corresponds to angular acc original is 1.5*Ts*Ts
if pre_T<Ts
    max_d_d_phi1=(pre_T/Ts)^2*max_d_d_phi;
else max_d_d_phi1=max_d_d_phi;
end
xc_td=0.3; 
% if ((xc-std_xc)<=-1.2 || (xc-std_xc)>=1.2)
%     position=0;
%     w_v=0; w_d_v=0; w_d_phi=0;
%     w_d_theta=0;
%     w_xc=1;
%     w_acc=1;
% else
    if ((xc-std_xc)<=-xc_td && d_theta<=-0.09)
        position=1;
        w_v=0; w_d_v=0; w_d_phi=0;
        w_d_theta=1;
%         if X0(10)<-0.3
%             w_xc=0.02*abs((-max_d_v)*tan(X0(10))/1.28/((X0(7)+max_d_v)*sin(abs(d_theta)+(X0(7)+max_d_v)*Ts*tan(X0(10))/2/1.28) ...
%                 -X0(7)*sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28))*w_d_theta);
%         else
            w_xc=0.25*w_d_theta;
%         end
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))-w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28)-sin(abs(d_theta)+X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
    elseif ((xc-std_xc)<=-xc_td && d_theta>=0.09)
        position=2;
        w_v=0; w_d_v=0; w_d_phi=0;
        w_d_theta=1;
%         w_xc=0.05*(tan(max_d_phi+X0(10))-tan(X0(10)))/1.28/(sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28) ...
%             -sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))*w_d_theta;  
        w_xc=1*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))-w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28)-sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
%         if X0(10)<0
%             max_d_d_phi=3*max_d_d_phi;
%         end
        
    elseif ((xc-std_xc)<=-xc_td && d_theta<=0.09 && d_theta>=0)
        position=3;
        w_v=0; w_d_v=0;
        w_d_theta=1;
%         w_xc=0.2*w_d_theta*(-tan(X0(10))+tan(X0(10)+max_d_phi))/1.28/(sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28) ...
%             -sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28));
%         if X0(10)<-0.1
%             w_xc=0.3*w_xc;
%         end
        w_xc=0.6*w_d_theta;
        w_acc=5*(2-1)*w_d_theta*Ts/X0(7);
        w_d_phi=7*(2-1)*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
    elseif ((xc-std_xc)<=-xc_td && d_theta>=-0.09 && d_theta<0)
        position=4;
        w_v=0; w_d_v=0;
        w_d_theta=1;
        %     if X0(10)<-0.1
        %         w_xc=0.06*(-max_d_v)*tan(X0(10))/1.28/((X0(7)+max_d_v)*sin(abs(d_theta)+(X0(7)+max_d_v)*Ts*tan(X0(10))/2/1.28) ...
        %             -X0(7)*sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28))*w_d_theta;
        %     else
        %         w_xc=w_d_theta;
        %     end
        w_xc=0.8*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))-w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28)-sin(abs(d_theta)+X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
        w_d_phi=7*(2-1)*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
        w_d_phi=0;
    elseif ((xc-std_xc)>=-xc_td && (xc-std_xc)<=0 && d_theta>=0.09)
        position=5;
        w_v=0; w_d_v=0;
        w_d_theta=1;
%         w_xc=0.02*(tan(max_d_phi+X0(10))-tan(X0(10)))/1.28/(sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28) ...
%             -sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))*w_d_theta;
        w_xc=1.5*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))-w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28)-sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
        w_d_phi=(1-0.05)*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
    elseif ((xc-std_xc)>=-xc_td && (xc-std_xc)<=0 && d_theta<=-0.09)
        position=6;
        w_v=0; w_d_v=0;
        w_d_theta=1;
%         w_xc=abs(0.02*(-max_d_v)*tan(X0(10))/1.28/((X0(7)+max_d_v)*sin(abs(d_theta)+(X0(7)+max_d_v)*Ts*tan(X0(10))/2/1.28) ...
%             -X0(7)*sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28))*w_d_theta);
        w_xc=0.25*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))-w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28)-sin(abs(d_theta)+X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
        w_d_phi=7*(2-1)*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
    elseif (abs(xc-std_xc)<=xc_td && abs(d_theta)<=0.09)
        position=7;
        w_d_theta=1;
%         w_xc=0.2*(tan(max_d_phi+X0(10))-tan(X0(10)))/1.28/(sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28) ...
%             -sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))*w_d_theta;
        w_xc=0.8*w_d_theta;
        if (w_xc==0) || (w_xc>1)
            w_xc=0.1*w_d_theta;
        end
        w_d_phi=100*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
        w_d_phi=1;
        w_d_v=2*w_d_phi;
        w_v=2.5*w_d_v;
        w_acc=1.0;
%         max_d_phi=0.02;
    elseif (xc-std_xc>=xc_td && d_theta>=0.09)
        position=8;
        w_v=0; w_d_v=0; w_d_phi=0;
        w_d_theta=1;
% %         if X0(10)<0.3
% %             w_xc=max([-15.1*(max_d_v)*tan(X0(10))/1.28/((X0(7)+max_d_v)*sin(abs(d_theta)-(X0(7)+max_d_v)*Ts*tan(X0(10))/2/1.28) ...
% %                 -X0(7)*sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28))*w_d_theta, w_d_theta]);
% %         else
% %             w_xc=0.02*w_d_theta;
% %         end
        w_xc=0.25*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))+w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28)+sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
    elseif (xc-std_xc>=xc_td && d_theta<=-0.09)
        position=9;
        w_v=0; w_d_v=0; w_d_phi=0;
        w_d_theta=1;
% %         w_xc=0.05*(tan(max_d_phi+X0(10))-tan(X0(10)))/1.28/(sin(abs(d_theta)+X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28) ...
% %             -sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28))*w_d_theta;
        w_xc=1*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))-w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)+X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28)-sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
%         if X0(10)>0
%             max_d_d_phi=3*max_d_d_phi;
%         end
    elseif (xc-std_xc>=xc_td && d_theta<=0 && d_theta>=-0.09)
        position=10;
        w_v=0; w_d_v=0;
        w_d_theta=1;
        
% %         w_xc=0.2*w_d_theta*(-tan(X0(10))+tan(X0(10)+max_d_phi))/1.28/(sin(abs(d_theta)+X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28) ...
% %             -sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28));
% %         if X0(10)>0.1
% %             w_xc=0.3*w_xc;
% %         end
        w_xc=0.6*w_d_theta;
        w_acc=5*(2-1)*w_d_theta*Ts/X0(7);
        w_d_phi=7*(2-1)*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
    elseif (xc-std_xc>=xc_td && d_theta<=0.09 && d_theta>=0.0)
        position=11;
        w_v=0; w_d_v=0;
        w_d_theta=1;
        %     if X0(10)<0.1
        %         w_xc=max([-10.1*(max_d_v)*tan(X0(10))/1.28/((X0(7)+max_d_v)*sin(abs(d_theta)-(X0(7)+max_d_v)*Ts*tan(X0(10))/2/1.28) ...
        %             -X0(7)*sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28))*w_d_theta, w_d_theta]);
        %     else
        %         w_xc=0.06*w_d_theta;
        %     end
% %         w_xc=w_d_theta;
        w_xc=0.8*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))+w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28)+sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
        w_d_phi=0.1*(1.1-1)*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
    elseif (xc-std_xc<=xc_td && xc-std_xc>=0 && d_theta<=-0.09)
        position=12;
        w_v=0; w_d_v=0;
        w_d_theta=1;
%         w_xc=0.02*(tan(max_d_phi+X0(10))-tan(X0(10)))/1.28/(sin(abs(d_theta)+X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28) ...
%             -sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28))*w_d_theta;
        w_xc=1.5*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))-w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)+X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28)-sin(abs(d_theta)+X0(7)*Ts*tan(X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
        w_d_phi=7*(2-1)*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
    elseif (xc-std_xc<=xc_td && xc-std_xc>=0 && d_theta>=0.09)
        position=13;
        w_v=0; w_d_v=0;
        w_d_theta=1;
%         w_xc=abs(0.02*(max_d_v)*tan(X0(10))/1.28/((X0(7)+max_d_v)*sin(abs(d_theta)-(X0(7)+max_d_v)*Ts*tan(X0(10))/2/1.28) ...
%             -X0(7)*sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28))*w_d_theta);
        w_xc=0.25*w_d_theta;
        w_acc=3*(w_d_theta*X0(7)*Ts/1.28*(tan(max_d_phi+X0(10))-tan(X0(10)))+w_xc*X0(7)*Ts* ...
            (sin(abs(d_theta)-X0(7)*Ts*tan(X0(10))/2/1.28)+sin(abs(d_theta)-X0(7)*Ts*tan(max_d_phi+X0(10))/2/1.28))) ...
            *1.28/X0(7)/X0(7)/(tan(max_d_phi+X0(10))-tan(X0(10)));
        w_d_phi=7*(1.1-1)*w_d_theta*X0(7)*Ts/1.28*(-tan(X0(10))+tan(X0(10)+max_d_phi))/max_d_phi;
    end
% end
% w_d_phi=0;
if position==7
    max_d_d_phi=max_d_d_phi*0.5*(abs(d_theta)/0.09+abs((xc-std_xc)/xc_td));
    max_d_phi=max_d_phi*0.5*(abs(d_theta)/0.09+abs((xc-std_xc)/xc_td));
%     max_d_d_phi1=min([max_d_d_phi1 max_d_d_phi]);
end
% 22/1/15
v_max=min([sqrt(1.6/abs(k)) sqrt(1.6*1.28/tan(abs(X0(10))))]);
if (v_max>5.0)
    v_max=5.0;
end
if position~=7
    v_max=min([1.5 v_max]);
end
std_v0=v_max; %m/s, standard cruising speed/max speed when travelling

% 21/1/15
if (abs(X0(7))>v_max) || position~=7
    max_d_v=2.0*Ts; %m/s, larger so that spd can b reduced in shorter time
else
    max_d_v=.5*Ts; %m/s
end

w=[w_xc w_d_theta w_v w_d_v w_d_phi w_acc];
d_v=2*max_d_v*rand(P,horizon)-max_d_v; %rand() is btw 0 and 1
d_phi=2*max_d_phi*rand(P,horizon)-max_d_phi;
d_phi(1:2,:)=0; d_v(1,:)=0; d_v(3,:)=0;
d_phi(4,:)=max_d_phi;
d_phi(5,:)=-max_d_phi;
% abs(d_phi(:,1)-pre_d_phi) shd b within max_d_d_phi
d_phi(:,1)=2*max_d_d_phi*rand(P,1)-max_d_d_phi+pre_d_phi;
d_phi(1:2,:)=0;
if position==7
    d_phi(:,1)=0;
else
%     d_phi(1:5:end,1)=0;
end
r=randi([6,P],[1,5]);
for i=1:5
    d_phi(r(i),:)=pre_U(2,:);
    d_v(r(i),:)=pre_U(1,:);
end
r=randi([6,P],[1,5]);
end_ind=1;
for i=1:5
    d_phi(r(i),1:end_ind)=0;
    end_ind=end_ind+3;
end
% logSolvert1=toc(tic);
F=C_Fittness(d_v,d_phi,X0,xc,d_theta,abc1,std_xc,std_v0,w,max_d_d_phi,pre_d_phi,max_d_phi,xc_td,max_d_d_phi1,v_max,phi1_d,abc2,z_th);
[~, I]=max(F);
best_d_v=d_v(I,:); best_d_phi=d_phi(I,:); best_pm=pm_adapt(I);
U=zeros(2,horizon); cost=0;
% toc
%% Genetic algorithm core
% tic;
tt=0;
i=0;
iteration=300;
ND=normrnd(0,1,[P,iteration]);
expND=exp(-gamma*ND);
time_td=1.3*Ts;
GAt=tic;
while (i<iteration && (get_status('Con')~=1 || (get_status('Con')==1 && toc(GAt)<0.5*Ts)) ...
        && toc(GAt)<time_td)
    i=i+1;
% for i=1:200
    ind=C_qTournament(F); %q is 3
%     tic;
%     ND=normrnd(0,1,[P,1]);
%     pm_adapt=pm_adapt./(pm_adapt+(1-pm_adapt).*exp(-gamma*ND));
    pm_adapt=1./(1+(1./pm_adapt-1).*expND(:,i));
    pm_adapt(pm_adapt<0.005)=0.005;
    pm_adapt=pm_adapt(ind);
%     tt=tt+toc;
%     tic;
    d_v=d_v(ind,:); 
    d_phi=d_phi(ind,:);
%     tt=tt+toc;
    C_mutation_adapt(d_v, d_phi, pm_adapt, max_d_v, max_d_phi);
%     tic;
    d_v(P,:)=best_d_v;
    d_phi(P,:)=best_d_phi;
    pm_adapt(P)=best_pm;
%     tt=tt+toc;
    F=C_Fittness(d_v, d_phi, X0, xc, d_theta, abc1, std_xc, std_v0,w,max_d_d_phi,pre_d_phi,max_d_phi,xc_td,max_d_d_phi1,v_max,phi1_d,abc2,z_th);
%     tic;
    [~,I]=max(F);
%     tt=tt+toc;
%     U=[d_v(I,:); d_phi(I,:)];
%     tic;
    best_d_v=d_v(I,:); best_d_phi=d_phi(I,:); best_pm=pm_adapt(I);
%     tt=tt+toc;
%     logmaxF(i)=max(F);
%     status=get_status('Con');
%     endt=toc;
%     tic;
% % % %     if get_status('Con')==1
% % % %         break;
% % % %     end
%     tt=tt+toc;
end
sss=[i get_status('Con') toc(GAt)];
%14/1/15
% if get_status('Con')==1 && toc(GAt)<0.6*Ts
%     U=pre_U;
% else
    U=[d_v(I,:); d_phi(I,:)];
% end
i=tt;
% a=0;
% toc
%% drawing out the MPC results. Optional. Need to comment out later
% % % % % % % % figure;hold on;
% % % % % % % % global_varibles;
% % % % % % % % InitializationGlobalVariable;
% % % % % % % % phi1=X0(10); v1=X0(7); theta=X0(3); x=X0(1); z=X0(2);
% % % % % % % % zz=[-0.5 v1*Ts*horizon+2]; xx=-(abc(1)*zz.^2+abc(2)*zz+abc(3));
% % % % % % % % patch([xx xx(2)-3/cos(d_theta) xx(1)-3/cos(d_theta)],[zz zz(2) zz(1)],'g'); hold on;
% % % % % % % % plot(xx-1.5/cos(d_theta),zz,'k--','LineWidth',0.5);
% % % % % % % % plotCar(pi-X0(3),-X0(1),X0(2),X0(10)); hold on; axis equal;
% % % % % % % % logx(1)=x; logz(1)=z; logtheta(1)=theta;
% % % % % % % % for i=1:horizon
% % % % % % % %     phi1=phi1+U(2,i);
% % % % % % % %     v1=v1+U(1,i);
% % % % % % % %     thetadot=-tan(phi1)*v1/1.28;
% % % % % % % %     theta=theta+thetadot*Ts;
% % % % % % % %     xdot=cos(theta)*v1;
% % % % % % % %     x=x+xdot*Ts;
% % % % % % % %     zdot=sin(theta)*v1;
% % % % % % % %     z=z+zdot*Ts;
% % % % % % % %     logx(i+1)=x; logz(i+1)=z; logtheta(i+1)=theta;
% % % % % % % % end
% % % % % % % % v1=v_max;
% % % % % % % % for i=(horizon+1):(2*horizon)
% % % % % % % %     thetadot=-tan(phi1)*v1/1.28;
% % % % % % % %     theta=theta+thetadot*Ts;
% % % % % % % %     xdot=cos(theta)*v1;
% % % % % % % %     x=x+xdot*Ts;
% % % % % % % %     zdot=sin(theta)*v1;
% % % % % % % %     z=z+zdot*Ts;
% % % % % % % %     logx(i+1)=x; logz(i+1)=z; logtheta(i+1)=theta;
% % % % % % % % end
% % % % % % % % f_xc=-x-1.5;
% % % % % % % % f_d_theta=theta-pi/2;
% % % % % % % % plotCar(pi-theta,-x,z,phi1);
% % % % % % % % hold on; plot(-logx,logz,'r.');plot(-logx,logz,'r');
% % % % % % % % a=0;
% logSolvert=toc(st);