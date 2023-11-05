  xyz=[]; %x-/y-/z-Koordinaten
  
  tet2=0;
  for tet1=90:-0.5:-90 % von 90° bis -90° (Auf 180° Spielfrei beschraenkt)
      for tet3=90:-0.5:-90 % von 90° bis -90° (Auf 180° beschraenkt)
              [x,y,z] = forwkinoD(tet1,tet2,tet3); %forwkinoD: Forwärtskinematik ohne Darstellung der Ergebinsse 
              xyz=[xyz; x,y,z];
      end
  end
  
  figure 
  plot3(xyz(:,1),xyz(:,2),xyz(:,3), '.')
  axis on
  axis([-40 40 -40 40])
  axis equal
  grid on 
  xlabel(strcat("X-Achse"));
  ylabel(strcat("Y-Achse"));
  zlabel(strcat("Z-Achse"));
  set(gca(),"fontsize",14,"linewidth",1);                   %gca: get current axis
  title(strcat("Arbeitsraumanalyse"));                      %strcat: string concatenation