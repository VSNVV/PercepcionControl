%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.45:11311') % IP de la MV
setenv('ROS_IP','192.168.1.42') % IP de nuestro ordenador

rosinit;

%% PUBLISHERS
activaPub = rospublisher('/cmd_motor_state','std_msgs/Int32');
publisher = rospublisher('cmd_vel', 'geometry_msgs/Twist');

%% SUBSCRIBERS
laser = rossubscriber('/scan');
odom = rossubscriber('/pose');

%% MENSAJES
mensajeActivarMotor = rosmessage(activaPub);
mensajeMovimiento = rosmessage(publisher);

%% ALGORITMO
mensajeActivarMotor.Data = 1;
send(activaPub, mensajeActivarMotor);

mensajeLaser = receive(laser, 1);
listaDistanciasLaser = mensajeLaser.Ranges;
distanciaFrontal = listaDistanciasLaser(200); % Laser frontal

distanciaObjetivo = 2; % Distancia objetivo en metros

while(distanciaFrontal ~= distanciaObjetivo)

    mensajeLaser = receive(laser, 1);
    listaDistanciasLaser = mensajeLaser.Ranges;
    distanciaFrontal = listaDistanciasLaser(200);

    if(distanciaFrontal > distanciaObjetivo)
        mensajeMovimiento.Linear.X = 0.1;
    end

    if(distanciaFrontal < distanciaObjetivo)
        mensajeMovimiento.Linear.X = -0.1;
    end

    if(distanciaFrontal == distanciaObjetivo)
        mensajeMovimiento.Linear.X = 0.0;
    end

    send(publisher, mensajeMovimiento);

    disp(distanciaFrontal);
end

%% DESCONEXION DE ROS
rosshutdown;