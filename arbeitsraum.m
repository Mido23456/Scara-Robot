% SCARA-Roboter-Parameter
a1 = 10;
a2 = 10;
d1 = 8;

% Gelenkwinkel (in Grad)
theta1 = -90:5:90;
theta2 = -150:5:150;

% Initialisierung der Matrizen f??r die Endeffektor-Positionen
X = zeros(length(theta1), length(theta2));
Y = zeros(length(theta1), length(theta2));
Z = zeros(length(theta1), length(theta2));

% Berechnung der Endeffektor-Positionen f??r jeden Gelenkwinkel
for i = 1:length(theta1)
    for j = 1:length(theta2)
        x = a1 * cosd(theta1(i)) + a2 * cosd(theta1(i) + theta2(j));
        y = a1 * sind(theta1(i)) + a2 * sind(theta1(i) + theta2(j));
        z = d1;

        X(i, j) = x;
        Y(i, j) = y;
        Z(i, j) = z;
    end
end

% Plotten des SCARA-Roboter-Arbeitsraums
figure
surf(X, Y, Z);
xlabel('X-Achse');
ylabel('Y-Achse');
zlabel('Z-Achse');
title('SCARA-Roboter Arbeitsraum');
axis equal
grid on
