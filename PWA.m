phi=0:0.01:.45;
y=tan(phi);
y1=phi;
sq_pre=100;
for i=0.19:0.01:0.44
    phi0=i;
    y2=1/cos(phi0)^2*phi+tan(phi0)-phi0/cos(phi0)^2;
    [xx, ~]=polyxpoly(phi,y1,phi,y2);
    y3=[y1(phi<=xx) y2(phi>xx)];
    sq=sum((y-y3).^2);
    if sq<sq_pre
        sq_pre=sq;
        f_phi=phi0;
        f_xx=xx;
    end
end