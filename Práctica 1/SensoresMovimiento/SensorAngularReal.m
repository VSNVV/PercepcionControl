%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

%% INICIALIZACION DE ROS
rosinit;

%% SUBSCRIBERS
odometria = rossubscriber('robot0/odom');

%% PUBLISHERS
activaPub = rospublisher('/cmd_motor_state','std_msgs/Int32');
publisher = rospublisher('cmd_vel', 'geometry_msgs/Twist');

%% GENERAMOS EL MENSAJE
mensajeActivarMotor = rosmessage(activaPublisher);
mensajeMovimiento = rosmessage(publisher);

% Velocidad lineal (solo se usa X)
mensajeMovimiento.Linear.X = 0.0;
mensajeMovimiento.Linear.Y = 0.0;
mensajeMovimiento.Linear.Z = 0.0;

% Velocidad angular (solo se usa Z)
mensajeMovimiento.Angular.X = 0.0;
mensajeMovimiento.Angular.Y = 0.0;
mensajeMovimiento.Angular.Z = 0.0;

%% Periodicidad del bucle de 10Hz
rate = robotics.Rate(10);
pause(1); % Pausamos para asegurarnos de que llega un mensaje

while(strcmp(odometria.LatestMessage.ChildFrameId,'base_link') ~= 1)
    odometria.LatestMessage
end
%% INICIALIZAMOS LA PRIMERA POSICION
posicionInicial = odometria.LatestMessage.Pose.Pose.Position;

%% ALGORITMO
listaDistancias = [];
listaDiferencias = [];

mensajeActivarMotor.Data = 1;
send(activaPublisher, mensajeActivarMotor);

send(publisher, mensajeMovimiento);

for i = 1:60
    posicionActual = odometria.LatestMessage.Pose.Pose.Position;

    distanciaRecorrida = sqrt((posicionInicial.X - posicionActual.X)^2 + (posicionInicial.Y - posicionActual.Y)^2);
    listaDistancias = [listaDistancias, distanciaRecorrida];
    
    disp(listaDistancias);
    
    waitfor(rate);
end

mensajeMovimiento.Linear.X = 0;
send(publisher, mensajeMovimiento);

%% CALCULO DE DIFERENCIAS
for index = 2 : length(listaDistancias)
    diferencia = listaDistancias(index) - listaDistancias(index - 1);
    if(diferencia ~= 0)
        listaDiferencias = [listaDiferencias, diferencia];
    end
end

disp(listaDiferencias);

disp(min(listaDiferencias));
disp(max(listaDiferencias));
disp(mean(listaDiferencias));

%% DESCONEXION DE ROS
rosshutdown;