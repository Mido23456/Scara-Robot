function k=pfeilstruct(T,L)
% k=Struct für Pfeilkoordinaten
% T Frame
% L = Pfeillaenge
% Pfeil:


hut=.04*L; % Pfeil Durchmesser oben
hoe=.2; % Pfeillaenge oben (Gesamtlaenge 1)
stiel=.01*L; % Pfeilschaft
pp=0:pi/8:2*pi;
x=sin(pp);
xx=[stiel*x;stiel*x;hut*x;0*x];

y=cos(pp);
yy=[stiel*y;stiel*y;hut*y;0*y];

zz=[zeros(size(y)); (1-hoe)*L*ones(size(y));(1-hoe)*L*ones(size(y)); L*ones(size(y))];

% Erzeugung und Darstellung Koordinatensystem
x1=[]; y1=[]; z1=[];
% z-Pfeil
for kk=1:size(xx,1)
    zw=T*[xx(kk,:); yy(kk,:); zz(kk,:); ones(1,size(xx,2))];
    x1=[x1;zw(1,:)];
    y1=[y1;zw(2,:)];
    z1=[z1;zw(3,:)];
end
k.z.x=x1; k.z.y=y1; k.z.z=z1;

x1=[]; y1=[]; z1=[];
% y-Pfeil
for kk=1:size(xx,1)
    zw=T*[xx(kk,:); zz(kk,:); yy(kk,:); ones(1,size(xx,2))];
    x1=[x1;zw(1,:)];
    y1=[y1;zw(2,:)];
    z1=[z1;zw(3,:)];
end

k.y.x=x1; k.y.y=y1; k.y.z=z1;


x1=[]; y1=[]; z1=[];
% x-Pfeil
for kk=1:size(xx,1)
    zw=T*[zz(kk,:); yy(kk,:); xx(kk,:); ones(1,size(xx,2))];
    x1=[x1;zw(1,:)];
    y1=[y1;zw(2,:)];
    z1=[z1;zw(3,:)];
end
k.x.x=x1; k.x.y=y1; k.x.z=z1;

