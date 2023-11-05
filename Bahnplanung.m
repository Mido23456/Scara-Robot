% Kontinuieliche Bahnbewegung (CP):
%PTP synchron mit �berschleifen 
%Bewegungsgleichung (Achsenweg):
%x(t) = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5

% Geschwingdigkeitsgleichung (Achsengeschwindigkeit):
% v(t) = a1+2*a2*t+3*a3*t^2+4*a4*t^3+5*a5*t^4

% Beschleunigungsgleichung (Achsenbeschleunigung):
% a(t) = 2*a2+6*a3*t+12*a4*t^2+20*a5*t^3

% Randbedingungen:
% x(0) = 0                    --> a0 = 0
% v(0) = v1 (Bewegungsanfang) --> a1 = v1
% v(T) = v2 (Bewegungsende)
% a(0) = 0                    --> a2 = 0
% a(T) = 0                    

% Bleibt 3 unbekannte mit Dreigleichungen --> l�sbar 
% Gleichungssystemsaufstellung:
% L�sungsansatz (t = T)
%
% x(t) = v1*t   + a3*t^3    + a4*t^4    + a5*t^5
% v(t) = v1     + 3*a3*t^2  + 4*a4*t^3  + 5*a5*t^4
% a(t) = 6*a3*t + 12*a4*t^2 + 20*a5*t^3
%
% x(T) - v1*T =  a3*T^3   + a4*T^4    + a5*T^5
% v2 - v1     =  3*a3*T^2 + 4*a4*T^3  + 5*a5*T^4
% 0           =  6*a3*T   + 12*a4*T^2 + 20*a5*T^3
%
% Matrixaufstellung:
% 
% A = [6*T   12*T^2 20*T^3;
%      3*T^2 4*T^3  5*T^4 ;
%      T^3    T^4   T^5  ];

% B = [x(T) - v1*T;
%      v2   - v1  ;
%      0         ];

% [a3,a4,a5] = A\B; --> a3 = (2/T^3)  * (5*x(T)  - 3*T*v1 - 2*v2*T)
%                       a4 = (-1/T^4) * (15*x(T) - 8*T*v1 - 7*T*v2)
%                       a5 = (3/T^5)  * (2*x(T)  - T*v1   - T*v2)
%

% Praktische Umsetzung:

function [vw,sw,tw]=Bahnplanung(a,B,v,wstart,wende,tab)
   % a=winkel (Spaltenvektor in rad) aus der Inversen Kinematik
   % tab=Abtastzeit f�r v und s-Verlauf
   % B=Bahnkurve: enth�lt [x1,y1,z1;x2,y2,z2;usw.] Vorgabe f�r die inverse Kinematik
   % v enth�lt gew�nschte Bahngeschwindigkeit, L�nge(v) = (L�nge von a - 1)
   % wstart = Achsgeschwindigkeit bei Start, z.B. Null
   % wende = Achsgeschwindigkeit am Ende
   % Beispiel:
   % [vw,sw,tw] = Bahnplanung([0;1.15],[50 0 50;20 0 50],10,0,      0,    0.001)
   %                           a               B          v wstart  wende tab
   % alle T berechnen sich aus Bahngeschwindigkeit und Bahnpunkten
   
   T=zeros(length(v),1);
   
   for i=1:length(v)
     dx=sqrt((B(i+1,1)-B(i,1))^2+(B(i+1,2)-B(i,2))^2+(B(i+1,3)-B(i,3))^2);
     %euklidischer Abstand aktueller Punkt zum n�chsten
     T(i)=dx/v(i);
     %Abstand / gew�nschte Bahngeschwindigkeit v = Zeit vom aktuellen Punkt zum n�chsten
   end
   
   T=tab*ceil(T./tab);
   %T ist Vielfaches von tab (aufgerundet ergibt tats�chliche Zeiten von Punkt zu Punkt
   %T auf tab "skalieren"
   schritte=T/tab;%Vektor: Anzahl Abtastschritte f�r jedes T
   
   %L�nge von T und v ist L�nge(a)-1 !
   
   va=zeros(length(a),1);                             %Ergebnis f�r Winkelgeschwindigkeit
   va(1)=wstart;                                      %erste Geschwindigkeit
   vw=[];
   tw=0;                                              %t0
   sw=[];
   sww=a(1);                                          %erster Winkel
   
   for j=2:length(va) 
     v1=va(j-1);                                      %neue Startgeschwindigkeit = alte Endgeschwindigkeit
     v2=(a(j)-a(j-1))./T(j-1);                        %neue Endgeschwindigkeit = Winkeldifferenz / T
     
     if (j==length(va))
       v2=wende;                                      %letzte Endgeschwindigkeit
       
     end
     va(j)=v2;                                        %Aktuelle Endgeschwindigkeit eintragen
     x=(a(j)-a(j-1));                                 %delta phi
     a1=v1;                                           %a1,2,3,4,5 Polynomkoeffizienten
     a2=0;
     a3=[ 2*(5*x-3*T(j-1)*v1-2*v2*T(j-1))/T(j-1)^3];
     a4=[ -(15*x-8*T(j-1)*v1-7*v2*T(j-1))/T(j-1)^4];
     a5=[     3*(2*x-T(j-1)*v1-v2*T(j-1))/T(j-1)^5];
     
     t=0:(T(j-1)/schritte(j-1)):T(j-1);               %Abtastzeiten
     vv=v1+3*a3*(t.^2)+4*a4*(t.^3)+5*a5*(t.^4);       %Berechnung Geschwindigkeitsprofil
     vv=vv(2:end);                                    %startet mit Index 2 (siehe for loop)
     
     vw=[vw(:);vv(:)];
     %Speicherung Profile dieses Schleifendurchlaufs in Gesamtvektor mit allen Geschwindigkeitsprofilen
     t=t(2:end);
     
     sww=[sww [sww(end)+cumsum((T(j-1)/schritte(j-1))*vv(1:end-1)) a(j)] ]; 
     %"Integration" des Geschwindigkeitsprofils ergibt Winkelprofil
     
     tw=[tw(:);tw(end)+t(:)];                         %neue Zeile mit Abtastzeiten anf�gen (startet mit tw(end)
   end
   
   tw=tw(1:end-1);                                    %Zeitwerte zu Profilen
   sw=sww(1:end-1);                                   %Winkelprofil
