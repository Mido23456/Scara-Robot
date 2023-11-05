function plotframe(Tr,L)
% T = Frame von A nach B, 
% oder Framematrix [M1;M2;...]
% L=Laenge der dargestellten Einheitsvektoren

% 
% f1=plot3([0 L], [0 0 ], [0 0 ],'r:');
% hold on
% set(f1,'linewidth',2)
% f1=plot3([0 0], [0 L ], [0 0 ],'g:');
% set(f1,'linewidth',2)
% f1=plot3([0 0], [0 0 ], [0 L ],'b:');
% set(f1,'linewidth',2)
% hold on
% axis on
% grid on
% axis equal
% view(-37.5,30)
% 
hold on
k=pfeilstruct(eye(4),L);        % Pfeilobjekt erzeugen
xx=k.z.x;
yy=k.z.y;
zz=k.z.z;

f4=surf(xx,yy,zz);          % z-Achse
set(f4,'facecolor',[.7 .7 1]);
set(f4,'edgecolor',[.7 .7 1]);

f5=surf(xx,zz,yy);          % y-Achse
set(f5,'facecolor',[.7 1 .7]);
set(f5,'edgecolor',[.7 1 .7]);

f6=surf(zz,yy,xx);          % z-Achse
set(f6,'facecolor',[1 .7 .7]);
set(f6,'edgecolor',[1 .7 .7]);
Talt=zeros(4,4);
ii=1;
for kk=1:size(Tr,1)/4;
T=Tr((kk-1)*4+1:kk*4,:);
k=pfeilstruct(eye(4),ii*L);        % Pfeilobjekt erzeugen
xx=k.z.x;
yy=k.z.y;
zz=k.z.z;
ii=1.03*ii;


x1=[]; y1=[]; z1=[];
% z-Pfeil
for kk=1:size(xx,1)
    zw=T*[xx(kk,:); yy(kk,:); zz(kk,:); ones(1,size(xx,2))];
    x1=[x1;zw(1,:)];
    y1=[y1;zw(2,:)];
    z1=[z1;zw(3,:)];
end
%        vz1=T*[0;0;1;1];
%        vz2=Talt*[0;0;1;1];
%     if vz1==vz2;
%         z1(2:end,:)=z1(2:end,:)*1.2;
%         33
%     end

f1=surf(x1,y1,z1);
set(f1,'facecolor',[0 0 1]);
set(f1,'edgecolor',[0 0 .9]);


x1=[]; y1=[]; z1=[];
% y-Pfeil
for kk=1:size(xx,1)
    zw=T*[xx(kk,:); zz(kk,:); yy(kk,:); ones(1,size(xx,2))];
    x1=[x1;zw(1,:)];
    y1=[y1;zw(2,:)];
    z1=[z1;zw(3,:)];
end
f2=surf(x1,y1,z1);
set(f2,'facecolor',[0 1 0]);
set(f2,'edgecolor',[0 .9 0]);

x1=[]; y1=[]; z1=[];
% x-Pfeil
for kk=1:size(xx,1)
    zw=T*[zz(kk,:); yy(kk,:); xx(kk,:); ones(1,size(xx,2))];
    x1=[x1;zw(1,:)];
    y1=[y1;zw(2,:)];
    z1=[z1;zw(3,:)];
end
f3=surf(x1,y1,z1);
set(f3,'facecolor',[1 0 0]);
set(f3,'edgecolor',[.9 0 0]);

plot3([Talt(1,4) T(1,4)],[Talt(2,4) T(2,4)],[Talt(3,4) T(3,4)],'m');

Talt=T;
end
view(-37,30)
grid




