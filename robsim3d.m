function [xt,yt,zt]=robsim3d(eing,tet)
% robsim3d(eing,tet) = Anzeige + Animation
% oder
% robsim3d(eing) = nur Anzeige
% oder
% [xt,yt,zt]=robsim3d(eing,tet), dabei wird eine Trajektorie 
% in xt,yt,zt mitgeschrieben und punktweise dargestellt
% eing ist die Definitionsmatrix des Roboters --> siehe testrob1, 
% tet eine Matrix der aktuellen Achsvariablen [q11, q12, .. q1n; q21,q22 ...
% oder robsim3d(eing,tet,ax)
% siehe Beispiele in testrob1.m 
%
% P.Duenow,  10.11.2003

% ****************************************************
xt=[]; yt=[]; zt=[];
nn=size(eing,1);         	% nn=Anzahl Objekte
x=[]; y=[]; z=[];		     	% Ergebnisse fuer 3D-plot	
Tr=[]; zeilen=[];	   	 	% Transformationsmatrizen der
				            	% einzelnen Bauteile des Roboters	  
					            % zeilen=Hilfsgroesse 
for kk=1:nn
 if (eing(kk,1)==1) | (eing(kk,1)==3)	% Zylinder oder Quader erzeugen: 

  w=0:pi/8:2*pi; 
  xa=zeros(6,length(w)); ya=zeros(6,length(w)); za=zeros(6,length(w));

  if eing(kk,1)==3   			% Quader erzeugen
   w=pi/4:pi/2:2.25*pi;
  end
					% ab hier gleich 
					% fuer Zylinder und Quader 					
  xx=sin(w); 
  yy=cos(w);
  r=eing(kk,2:4); 
  xaa=[zeros(size(xx));r(1)*xx;r(1)*xx;r(2)*xx;zeros(size(xx));r(2)*xx];
  yaa=[zeros(size(yy));r(1)*yy;r(1)*yy;r(2)*yy;zeros(size(yy));r(2)*yy];
  zaa=[zeros(size(xx)); zeros(size(xx)); ...
  zeros(size(xx)); r(3)*ones(size(xx)); ...
  r(3)*ones(size(xx)); r(3)*ones(size(xx))];
  [mm,nn]=size(xaa);
  xa(1:mm,1:nn)=xaa;
  ya(1:mm,1:nn)=yaa;
  za(1:mm,1:nn)=zaa;

  zeilen=[zeilen;size(xa,1)];
					% Zylinder fertig
  x=[x;xa]; y=[y;ya]; z=[z;za];  		% Eintrag in Plotliste

					% Berechnung der einzelnen
					% Koerpertransformationen	
  if eing(kk,mm)==0   			% Basiskoerper
   Tr1=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1;]; 
   Tr=[Tr;Tr1];
  elseif eing(kk,mm)==1			% Rotationsachse
   Tr1=rov2tr(rotz(eing(kk,mm-1)), [0; 0; 0]); 
   Tr=[Tr;Tr1]; 
  elseif eing(kk,mm)==2			% Translationsachse
   Tr1=rov2tr(rotz(0), [0; 0; eing(kk,mm-1)]); 
   Tr=[Tr;Tr1]; 
  elseif eing(kk,mm)==3			% aufgesetzter Zyl. o. Quad.

   if eing(kk,mm-1)~=0			% falls Anfangswert fuer q1
    Tr1=rov2tr(rotz(eing(kk,mm-1)),[0;0;0]);
    Tr1=Tr1*rov2tr(roty(0),[0; 0; eing(kk-1,4)]);
   else
    Tr1=rov2tr(roty(0),[0; 0; eing(kk-1,4)]);
   end
   Tr=[Tr;Tr1]; 
   elseif eing(kk,mm)==-3			% aufgesetzter Zyl. o. Quad.

   if eing(kk,mm-1)~=0			% falls Anfangswert fuer q1
    Tr1=rov2tr(rotz(eing(kk,mm-1)),[0;0;0]);
    Tr1=Tr1*rov2tr(roty(0),[0; 0; -eing(kk,4)]);
   else
    Tr1=rov2tr(roty(0),[0; 0; -eing(kk,4)]);
   end
   Tr=[Tr;Tr1]; 
   
   
  elseif eing(kk,mm)==4			% angesteckter Z.o.Q. seitlich 	
   if eing(kk,mm-1)~=0			% falls Anfangswert fuer q1
   Tr1=rov2tr(rotz(eing(kk,mm-1)),[0;0;0]);
   Tr1=Tr1*rov2tr(roty(pi/2),[eing(kk-1,2); 0; eing(kk-1,4)/2]);
   else
   Tr1=rov2tr(roty(pi/2),[eing(kk-1,2); 0; eing(kk-1,4)/2]);
   end


  
   Tr=[Tr;Tr1]; 

  elseif eing(kk,mm)==-4			% angesteckter Z.o.Q quer
   if eing(kk,mm-1)~=0			% falls Anfangswert fuer q1
    Tr1=rov2tr(rotz(eing(kk,mm-1)),[0;0;0]);
    Tr1=Tr1*rov2tr(roty(-pi/2),[eing(kk,4)/2; 0; eing(kk-1,4)+(eing(kk,2))]);
   else
    Tr1=rov2tr(roty(-pi/2),[eing(kk,4)/2; 0; eing(kk-1,4)+(eing(kk,2))]);
   end
   Tr=[Tr;Tr1];				% Einzeltransformation eintragen
  end	

 end   % von: if (eing(kk,1)==1) | (eing(kk,1)==3)
end    % von: for kk=1:nn


% **********************************************
% Begin der Darstellung:
% **********************************************

[gg,h]=size(Tr);
xxx=[]; yyy=[]; zzz=[];
g=gg/4;  			% g=Anzahl der darzustellenden Objekte
z1=1;   			%  Zeilenzaehler
Trges=eye(4,4);

for k=1:g
				% Transformation v. x,y,z mit Tr
 Trges=Trges*Tr(( (k-1)*4 )+1:(k*4),:);
 xx=[]; yy=[]; zz=[];
 for ll=1:zeilen(k)
  zw=Trges*([x(z1,:);y(z1,:);z(z1,:);ones(size(x(z1,:)))]);
  xx=[xx;zw(1,:)];	
  yy=[yy;zw(2,:)];
  zz=[zz;zw(3,:)];	
  z1=z1+1;
 end 
 xxx=[xxx;xx]; 
 yyy=[yyy;yy]; 
 zzz=[zzz;zz];
end
					% Transformationen beendet
figs=zeros(1,g);
mi=min(min([xxx;yyy;zzz]));		% Ausdehnung abschaetzen
ma=max(max([xxx;yyy;zzz]));
axis([mi ma mi ma 0 2*ma]);
axis off
axis equal
hold on

zz1=1;
for k=1:g
 xx=xxx(zz1:zz1+zeilen(k)-1,:);
 yy=yyy(zz1:zz1+zeilen(k)-1,:);
 zz=zzz(zz1:zz1+zeilen(k)-1,:);
 zz1=zz1+zeilen(k);
 
 
 figs(k)=surf(xx,yy,zz); 
 if eing(k,size(eing,2))==1		% Rotationsachse
   set(figs(k),'FaceColor',[.8 0 0])
   set(figs(k),'EdgeColor',[0 0 0])
 elseif eing(k,size(eing,2))==2 	% Translationsachse 
   set(figs(k),'FaceColor',[0 0 .6])
   set(figs(k),'EdgeColor',[0 0 0])
 else					% sonstige Koerper
   set(figs(k),'FaceColor',[.8 .8 .8])
 set(figs(k),'EdgeColor',[0 0 0])
end

end
drawnow

% einfache Darstellung beendet

% **************************************************
% falls nargin=2 starte Animation
% **************************************************

if nargin>1    				% Animation starten
					% Bestimme Anzahl der Achsen
 d=find(( eing(:,size(eing,2))==1 )|( eing(:,size(eing,2))==2 ));

 for ll=1:size(tet,1)
					% Transformationsmatrizen
					% fuer variable Achsen berechnen				
  for lll=1:size(d)
   if eing(d(lll),size(eing,2))==1
    par1=eing(d(lll),size(eing,2)-1);
    Tr(( (d(lll)-1)*4 )+1:(d(lll)*4),:)=rov2tr(rotz(par1+tet(ll,lll)),[0;0;0]);
   elseif eing(d(lll),size(eing,2))==2
    par1=eing(d(lll),size(eing,2)-1);
    Tr(( (d(lll)-1)*4 )+1:(d(lll)*4),:)=rov2tr(rotz(0),[0;0;par1+tet(ll,lll)]);
   end
  end

  [gg,h]=size(Tr);
  xxx=[]; yyy=[]; zzz=[];
  g=gg/4;  % g=Anzahl der darzustellenden Objekte
  z1=1;   %  Zeilenzaehler
  Trges=eye(4,4);
  for k=1:g
                                  % transformation v. x,y,z mit Tr
   Trges=Trges*Tr(( (k-1)*4 )+1:(k*4),:);
   xx=[]; yy=[]; zz=[];
   for ll=1:zeilen(k)
    zw=Trges*([x(z1,:);y(z1,:);z(z1,:);ones(size(x(z1,:)))]);
    xx=[xx;zw(1,:)];	
    yy=[yy;zw(2,:)];
    zz=[zz;zw(3,:)];	
    z1=z1+1;
   end 
   xxx=[xxx;xx]; 
   yyy=[yyy;yy]; 
   zzz=[zzz;zz];

   set(figs(k),'XData',xx);
   set(figs(k),'YData',yy);
   set(figs(k),'ZData',zz);
  end
					% Trajektorie des TCP
					% darstellen und ausgeben	
  if nargout>0				
   ttr=Trges*[0;0;0;1];				
   xt=[xt ttr(1)];
   yt=[yt ttr(2)];
   zt=[zt ttr(3)];
   plot3(ttr(1),ttr(2),ttr(3),'.');
end
  drawnow
% pause
end % von: for ll=1:size(tet,1)
end % von: if nargin>1

