v0=4;T=0.05;x0=0;y0=0;theta0=pi/2;phi0=0.4;
x(1)=x0;y(1)=y0;theta(1)=theta0;
%perfect model
for i=1:20
    x(i+1)=x(i)+cos(theta(i))*v0*T;
    y(i+1)=y(i)+sin(theta(i))*v0*T;
    theta(i+1)=theta(i)-tan(phi0)*v0/1.28*T;
end
figure;plot(x,y,'r');
dx(1)=0;dy(1)=v0*T;dtheta(1)=-tan(phi0)*v0*T/1.28;
%linearization around v0,x0,y0,theta0,phi0
for i=1:20
    dx(i+1)=dx(i)-sin(theta0)*v0*T*dtheta(i);
    dy(i+1)=dy(i)+cos(theta0)*v0*T*dtheta(i);
    dtheta(i+1)=dtheta(i);
end
hold on;plot(cumsum(dx),cumsum(dy),'--')
axis equal

%Piece wise affine model then linearization
clear all;
v0=4;T=0.05;x0=0;y0=0;theta0=pi/2;phi0=0.4;
x(1)=x0;y(1)=y0;thetak(1)=theta0;
dx(2)=0;dy(2)=v0*T;dtheta(2)=-tan(phi0)*v0*T/1.28;
for i=2:20
    theta=sum(dtheta)+theta0;
    if theta<0
        theta=theta+2*pi;
    end
    if (theta>=0) && (theta<pi/4)
        c=-0.3729; d=1; e=.9003; f=0;
    elseif (theta>=pi/4) && (theta<pi/2)
        c=-.9003;d=1.4142;e=.3729;f=.4142;
    elseif (theta>=pi/2) && (theta<3*pi/4)
        c=-.9003;d=1.4142;e=-.3729;f=1.5857;
    elseif (theta>=3*pi/4) && (theta<4*pi/4)
        c=-.3729;d=.1715;e=-.9003;f=2.8284;
        
    elseif (theta>=pi) && (theta<5*pi/4)
        c=.3729;d=2*pi*-.3729+.1715;
        e=-.9003; f=-(2*pi*e+2.8284);
    elseif (theta>=5*pi/4) && (theta<6*pi/4)
        c=.9003;d=2*pi*-.9003+1.4142;
        e=-.3729;f=-(2*pi*e+1.5857);
    elseif (theta>=6*pi/4) && (theta<7*pi/4)
        c=.9003;d=2*pi*-.9003+1.4142;
        e=0.3729;f=-(2*pi*e+.4142);
    elseif (theta>=7*pi/4) && (theta<8*pi/4)
        c=.3729;d=2*pi*-.3729+1;
        e=.9003;f=-(2*pi*e+0);
    end
    dx(i+1)=dx(i)+c*v0*T*dtheta(i);
    dy(i+1)=dy(i)+e*v0*T*dtheta(i);
    dtheta(i+1)=dtheta(i);
    
%     x(i)=x(i-1)+c*v0*T*(thetak(i-1)-thetak(1))+dx(2);
%     y(i)=y(i-1)+e*v0*T*(thetak(i-1)-thetak(1))+dy(2);
%     thetak(i)=thetak(i-1)+dtheta(2);
%     
end
hold on;plot(cumsum(dx),cumsum(dy),'g--')
axis equal