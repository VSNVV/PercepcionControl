%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

rosinit % Iniciamos ROS

%% SUBSCRIBERS
odom = rossubscriber('robot0/odom');
%% PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
%% GENERAMOS EL MENSAJE
msg = rosmessage(pub);
% Velocidad lineal (solo se usa X)
msg.Linear.X = -0.1;
msg.Linear.Y = 0;
msg.Linear.Z = 0;
% Velocidad angular (solo se usa Z)
msg.Angular.X = 0;
msg.Angular.Y = 0;
msg.Angular.Z = 0;
%% Periodicidad del bucle de 10Hz
rate = robotics.Rate(10);
pause(1); % Pausamos para asegurarnos de que llega un mensaje
while(strcmp(odom.LatestMessage.ChildFrameId,'robot0')~= 1)
    odom.LatestMessage
end
%% INICIALIZAMOS LA PRIMERA POSICION
posicionInicial = odom.LatestMessage.Pose.Pose.Position;

%% SISTEMA PARA ALMACENAR DISTANCIAS
listaDistancias = [];
listaDiferencias = [];

while(1)
    posicionActual = odom.LatestMessage.Pose.Pose.Position;
    distanciaRecorrida = sqrt((posicionInicial.X - posicionActual.X)^2 + (posicionInicial.Y - posicionActual.Y)^2);
    listaDistancias = [listaDistancias, distanciaRecorrida];
    send(pub, msg);
    if(distanciaRecorrida > 3)
        msg.Linear.X = 0;
        send(pub, msg);
        break;
    end
    waitfor(rate);
    disp(listaDistancias);
end

%% CALCULO DE DIFERENCIAS
for index = 2 : length(listaDistancias)
    diferencia = listaDistancias(index) - listaDistancias(index - 1);
    listaDiferencias = [listaDiferencias, diferencia];
end

disp(listaDiferencias);

disp(max(listaDiferencias));

%% DESCONEXION DE ROS
rosshutdown;
