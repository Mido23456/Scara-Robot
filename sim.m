clear all ;
close all;

scara1=[1 2.5 2.5 7 0 0;              %Basis des Scaras
        1 2  2  2 0 3;
        1 2  2  2 0 1;         %1 Rotationsachse
        1 2  2  4 0 3;         %Zahnriehmen Scheibe, die direkt an der Motorwelle festgeschraubt wird
        1 2  2  10 0 4;       % Arm1 mit der entsprechende L??nge
        1 2  2  4 0 -4;       % Platz f??r das Servo
        1 2  2  1 0 -3;      %Servo Geh??use
        1 2  2  2 0 1;       %2. Rotationsachse
        1 2  2  1 0 -3;      %Arm 2
        1 2  2  10 0 4;
        1 0.6  0.6 -8 0 -4;     %Stifthalterung
        1 0.6  0.6  9 0 3];
tet1=invkin(20,0,0);
[TCPPos1]=forwkin(tet1(1),tet1(2));
tet2=invkin(7.0711,17.0711,0);

[TCPPos2]=forwkin(tet2(1),tet2(2));
B = [20,0,0
    7.0711,17.0711,0];
vbahn= 50; %mm/sec --> 1cm/s --> 0.1m/s
v = vbahn;
wstart = 0;
wende = 0;
tab = 0.1;
SW = [];
VW = [];
 for j=1:length(tet1)
    [vw,sw,tw] = Bahnplanung(([tet1(j);tet2(j)]),[20,0,0;7.0711,17.0711,0],1,0, 0,tab) % Berechnung der Zwischenwerte
     SW = [SW sw(:)];
     VW = [VW vw(:)];
 end
 figure
 view(-44, 26)
 [xx,yy,zz]=robsim3d(scara1,[SW(:) VW(:)]);
 grid on
 axis on
 axis equal
 set(gca(),"fontsize",20,"linewidth",2);     %gca: get current axis
 title(strcat("Scara-Robot"));             %strcat: string concatenation
 xlabel('X-Achse');
 ylabel('Y-Achse');
 figure
 subplot(2,1,1)
 p1 = plot(tw,VW(:,1),'b',tw,VW(:,2),'r')
 title(strcat("Bahnplanung"));
 xlabel('T(Zeit)');
 ylabel('Achsengeschwindigkeit');
 grid
 subplot(2,1,2)
 p2 = plot(tw,SW(:,1),'b',tw,SW(:,2),'r')
 xlabel('T(Zeit)');
 ylabel('Achsenwinkel');
 grid
