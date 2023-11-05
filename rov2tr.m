function Tr=rov2tr(rot,ov)

% Tr=rov2tr(rot,ov)
% rot,Ortsvektor zu Frame

rot=[rot;0 0 0];
Tr=[rot,[ov; 1]];
