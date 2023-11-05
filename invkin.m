function tet=invkin(x,y,z);
  
  % Abmessungen:
  d1= 8;  %Abstand zwischen Basis und Endeffektor
  a1= 10; %Erste Arml?nge 
  a2= 10; %zweite Arml?nge
  
  tet=zeros(1,2);     %Ergebnisvektor
  H=sqrt(x^2+y^2);
  Delta=(atan2(y,x));
  phi=90-Delta;
  alpha=acos((H^2 + a1^2 - a2^2) / (2*H*a1));
  gamma=acos((a2^2 + a1^2 - H^2)/(2*a2*a1));
  
  if (x>0 && y>=0) %1st quadrant
    tet(1)=(Delta-alpha);
    tet(2)=pi-gamma;
  end%if
  
  if (x<0 && y>0) %2st quadrant
    tet(1)=(Delta-alpha);
    tet(2)=(pi-gamma);
  end%if
  
  if (x>=0 && y<=0) %3st quadrant
    tet(1)=(Delta+alpha);
    tet(2)=-(pi-gamma);
  end%if
  
  if (x<0 && y<0) %4st quadrant
    tet(1)= Delta+alpha;
    tet(2)=-(pi-gamma);
  end%if
 
  if (x==20 && y==0)
    tet(1)=0;
    tet(2)=0;
    end%if
  end%if
 