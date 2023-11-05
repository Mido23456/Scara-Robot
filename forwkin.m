function [x,y,z]=forwkin(q1,q2)
  % Abmessungen:
  d1= 8;  %Abstand zwischen Basis und Endeffektor
  a1= 10; %Erste Arml?nge 
  a2= 10; %zweite Arml?nge
  %q1=q1*(pi/180); %Umrechnung von Grad in rad
  %q2=q2*(pi/180);
  
  % DH-Parameter:
  T1_0=dh2Tr(q1,d1,0,0); %%(q, d, a, alpha)
  T2_1=dh2Tr(0,0,a1,0);
  T3_2=dh2Tr(q2,0,a2,0);
  
  %Position des Endeffektors:
  T3_0= T1_0*T2_1*T3_2;
  TCPPos=T3_0;
  
  x = TCPPos(1,4)
  y = TCPPos(2,4)
  z = (TCPPos(3,4))-d1
  
  %Eulerwinkel des Endeffektors:
  alpha=(180/pi)*atan2(T3_0(2,1),T3_0(1,1));                     % Drehwinkel um z
  beta=(180/pi)*atan2(-T3_0(3,1),sqrt(T3_0(1,1)^2+T3_0(2,1)^2)); % Drehwinkel um y
  gamma=(180/pi)*atan2(T3_0(3,2),T3_0(3,3));                     % Drehwinkel um x
  
  figure
  plotframe([T1_0;T1_0*T2_1;T1_0*T2_1*T3_2;T3_0], 10)
  axis equal
  set(gca(),"fontsize",20,"linewidth",2);                   %gca:      get current axis
  title(strcat("Frames f?r die einzelnen Roboterachsen"));  %strcat:   string concatenation
  xlabel('X-Achse');
  ylabel('Y-Achse');
  zlabel('Z-Achse');
  end