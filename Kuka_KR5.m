% Empezamos el robotics toolbox
startup_rvc

% Usamremos el robot Kuka KR5 a traves del siguiente comando
mdl_KR5

% qlim es un objeto de la clase Link. 
% qlim tiene las siguientes propiedades:
% kinematic: joint variable limits [min max]
% Es decir vamos a delimitar cada joint (1,:) y así sucesivamente del robot
% conversion va a tener la constante para convertir de grados que es lo que
% está entre [] a radianes
conversion = pi/180;

% Los valores de [-155 155] son tomados de las especificaciones del robot.
KR5.qlim(1,:) = conversion*[-155 155]; 
KR5.qlim(2,:) = conversion*[-180 65]; 
KR5.qlim(3,:) = conversion*[-15 158];
KR5.qlim(4,:) = conversion*[-350 350];
KR5.qlim(5,:) = conversion*[-130 130];
KR5.qlim(6,:) = conversion*[-350 350];

% Se crea un vector de tiempo en el se establece la duracion de 0 a 2 en
% intervalos de 0.08s para el movimiento completo del robot durante la
% simulacion
tiempo=[0:0.1:3]; 

% pos_A contiene la posicion inicial del robot
pos_A=[0 0 0 0 0 0]; 
% pos_B contiene la posicion final del robot.
pos_B=[0 pi/3 -pi/10 -pi/6 pi/5 pi/3];

% Calculo de la trayectoria por medio de la funcion jtraj
q=jtraj(pos_B,pos_B,tiempo);

% Obtencion de la cinematica directa.
T = KR5.fkine(q); 

% Obtencion de la cinematica inversa.
qi = KR5.ikcon(T); 

% Graficamos
plot(KR5,qi); 


% WAYPOINTS

% Vector de tiempo.
tiempo=[0:0.05:1]; 

% Definimos los waypoints 
% x, y, z
waypoint1 = SE3(0.9, 0.2, 0.5);
waypoint_2 = SE3(.5,-0.5, -2); 
waypoint_3 = SE3(1.5, 1.5, 1.5);
waypoint_4 = SE3(0.5, 0.04, 0.5);
waypoint_5 = SE3(0.9, -5, 3);
waypoint_6 = SE3(-15, -10, -16);

% Cinematica Inversa para poder obtener los angulos a partir de las
% coordenadas x, y, z de los waypoints seleccionados
q1 = KR5.ikcon(waypoint_1);
q2 = KR5.ikcon(waypoint_2);
q3 = KR5.ikcon(waypoint_3);
q4 = KR5.ikcon(waypoint_4);
q5 = KR5.ikcon(waypoint_5);
q6 = KR5.ikcon(waypoint_6);

% Obtenemos los jacobianos de cada waypoint para poder determinar si en ese
% punto existe una singularidad o no
J1 = KR5.jacobn(q1)
det(J1)
J2 = KR5.jacobn(q2)
det(J2)
J3 = KR5.jacobn(q3)
det(J3)
J4 = KR5.jacobn(q4)
det(J4)
J5 = KR5.jacobn(q5)
det(J5)
J6 = KR5.jacobn(q6)
det(J6)
J7 = KR5.jacobn(pos_B)
det(J7)

% Calculo de trayectoria con mtraj para cada waypoint definido hasta llegar
% al punto final
q=mtraj(@tpoly,pos_A,q1,tiempo); 
plot(KR5,q); 

q=mtraj(@tpoly,q1,q2,tiempo);
plot(KR5,q); % Plot
 
q=mtraj(@tpoly,q2,q3,tiempo);
plot(KR5,q);% Plot

q=mtraj(@tpoly,q3,q4,tiempo);
plot(KR5,q);% Plot

q=mtraj(@tpoly,q4,q5,tiempo);
plot(KR5,q); % Plot
 
q=mtraj(@tpoly,q5,q6,tiempo);
plot(KR5,q); % Plot

% Punto final
q=mtraj(@tpoly,q6,pos_B,tiempo);
plot(KR5,q);