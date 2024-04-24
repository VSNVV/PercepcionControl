rosshutdown;
%% INICIALIZACIÓN DE ROS (COMPLETAR ESPACIOS CON LAS DIRECCIONES IP)
setenv('ROS_MASTER_URI','http://172.29.30.172:11311') % IP del robot
setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador
rosinit;

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
MAX_TIME = 1000; %% Numero máximo de iteraciones
medidas = zeros(5,1000);
%% DECLARACIÓN DE SUBSCRIBERS
odom = rossubscriber('/pose'); % Subscripción a la odometría
sonar0 = rossubscriber('/sonar_0', rostype.sensor_msgs_Range);
pubActivaMotor = rospublisher('/cmd_motor_state','std_msgs/Int32');
%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist'); %
msg_vel = rosmessage(pub); %% Creamos un mensaje del tipo declarado en"pub" (geometry_msgs/Twist)
msg_sonar0 = rosmessage(sonar0);
msg_actva_motor = rosmessage(pubActivaMotor);
msg_actva_motor.Data = 1;
send(pubActivaMotor, msg_actva_motor);
%% Definimos la periodicidad del bucle (10 hz)
r = robotics.Rate(10);
waitfor(r);
pause(3); %% Esperamos entre 2 y 5 segundos antes de leer el primer mensaje para aseguramos que empiezan a llegar mensajes.
%% Nos aseguramos recibir un mensaje relacionado con el robot
while (strcmp(odom.LatestMessage.ChildFrameId,'base_link') ~= 1)
 odom.LatestMessage
end
%% Inicializamos variables para el control
i = 0;
pos = odom.LatestMessage.Pose.Pose.Position;
dist = msg_sonar0.Range_;
lastpos = pos;
lastdist = dist;
lastdistav = 0;
D = 1.5;
%Kd = 100;
Kd = 1;
%Kd = 0.01;
%Ko = 1;
Ko = 0.5;
%Ko = 0.0001;
msg_vel.Linear.X = 1; % velocidad lineal hacia adelante
send(pub, msg_vel); % enviar el mensaje de velocidad al robot
pause(1); % esperar 1 segundo
msg_vel.Linear.X = 0; % velocidad lineal hacia adelante
send(pub, msg_vel);
%% Bucle de control
while (1)
i = i + 1;
%% Obtenemos la posición y medidas de sonar
pos=odom.LatestMessage.Pose.Pose.Position;
msg_sonar0 = receive (sonar0);
%% Calculamos la distancia avanzada y medimos la distancia a la pared
distav =sqrt((lastpos.X - pos.X)^2 + (lastpos.Y - pos.Y)^2);
dist = msg_sonar0.Range_;
if dist > 5
 dist = 5;
end

%% Calculamos el error de distancia y orientación
Eori = atan2(abs(dist - lastdist), distav);
Edist = (dist+0.105)*cos(Eori) - D;
medidas(1,i) = dist;
medidas(2,i) = lastdist; %% valor anterior de distancia
medidas(3,i) = distav;
medidas(4,i) = Eori;
medidas(5,i) = Edist;

%% Calculamos las consignas de velocidades
consigna_vel_linear = 0.7;
consigna_vel_ang = Kd*Edist+Ko*Eori;

%% Aplicamos consignas de control
msg_vel.Linear.X = consigna_vel_linear;
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = consigna_vel_ang;

% Comando de velocidad
send(pub,msg_vel);
lastpos = pos;
lastdist = dist;
lastvAng = consigna_vel_ang;
lastdistav = distav;
% Temporización del bucle según el parámetro establecido en r
waitfor(r);
if i == 1000
 disp("finiquitado")
 
 save('medidas.mat','medidas');
 break;
end
end
%% DESCONEXIÓN DE ROS
rosshutdown;