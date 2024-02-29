%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://172.29.30.178:11311') % IP de la MV
setenv('ROS_IP','172.25.192.1') % IP de nuestro ordenador
rosshutdown;
%%INICIALIZACION DE ROS
rosinit;

%% PUBLISHERS
activaPublisher = rospublisher('/cmd_motor_state','std_msgs/Int32');
publisher = rospublisher('cmd_vel', 'geometry_msgs/Twist');

%% SUBSCRIBERS
odommetria = rossubscriber('/pose');
sonar3 = rossubscriber('/sonar_3');

%% MENSAJES
mensajeActivarMotor = rosmessage(activaPublisher);
mensajeMovimiento = rosmessage(publisher);

mensajeMovimiento.Linear.X = 0.0;
mensajeMovimiento.Linear.Y = 0.0;
mensajeMovimiento.Linear.Z = 0.0;

mensajeMovimiento.Angular.X = 0.0;
mensajeMovimiento.Angular.Y = 0.0;
mensajeMovimiento.Angular.Z = 0.0;

%% ALGORITMO
mensajeActivarMotor.Data = 1;
send(activaPublisher, mensajeActivarMotor);

mensajeSonar = receive(sonar3, 3);
distanciaActual = mensajeSonar.Range_;

distanciaObjetivo = 2;

while(distanciaActual ~= distanciaObjetivo)

    mensajeSonar = receive(sonar3, 1);
    distanciaActual = mensajeSonar.Range_;

    if(distanciaActual > distanciaObjetivo)
        mensajeMovimiento.Linear.X = 0.1;
    end

    if(distanciaActual < distanciaObjetivo)
        mensajeMovimiento.Linear.X = -0.1;
    end

    if(distanciaActual == distanciaObjetivo)
        mensajeMovimiento.Linear.X = 0.0;
    end
    
    send(publisher, mensajeMovimiento);
    
    disp(distanciaActual);
end

%% DESCONEXION DE ROS
rosshutdown;