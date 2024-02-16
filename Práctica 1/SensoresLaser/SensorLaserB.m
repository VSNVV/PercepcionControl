%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.45:11311') % IP de la MV
setenv('ROS_IP','192.168.1.42') % IP de nuestro ordenador

rosinit;

%% SUBSCRIBER AL LASER
laser = rossubscriber('/robot0/laser_1');
odom = rossubscriber('/robot0/odom');
publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

mensajeMovimiento = rosmessage(publisher);

mensajeLaser = receive(laser, 1);
listaDistanciasLaser = mensajeLaser.Ranges;
distanciaFrontal = listaDistanciasLaser(200);

%% ALGORITMO DE MOVIMIENTO
distanciaObjetivo = 2;

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

rosshutdown;

