%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://172.22.68.96:11311') % IP de la MV
setenv('ROS_IP','172.22.9.177') % IP de nuestro ordenador

%% INICIALIZACION DE ROS
rosinit;

%% COMANDOS A EJECUTAR
avanzar(2);
girar(90);
avanzar(1);
girar(-90);
avanzar(1);


%% DESCONEXION DE ROS
rosshutdown;

%% FUNCION DE AVANZAR
function avanzar(distancia)

    odometria = rossubscriber('/robot0/odom');
    publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

    mensajeMovimiento = rosmessage(publisher);
    mensajeMovimiento.Linear.X = 0.3;

    pause(1);
    while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0') ~= 1)
        odometria.LatestMessage
    end

    distanciaRecorrida = 0;
    posicionInicial = odometria.LatestMessage.Pose.Pose.Position;
    send(publisher, mensajeMovimiento);
    disp(mensajeMovimiento);
    
    while (distanciaRecorrida < distancia)
        posicionActual = odometria.LatestMessage.Pose.Pose.Position;
        distanciaRecorrida = sqrt((posicionInicial.X - posicionActual.X)^2 + (posicionInicial.Y - posicionActual.Y)^2);
        disp(distanciaRecorrida);
      
    waitfor(r)
    end

    mensajeMovimiento.Linear.X = 0.0;
    send(publisher, mensajeMovimiento);
end

%% FUNCION DE GIRAR
function girar(angulo)
    
    anguloEnRadianes = deg2rad(angulo);
    disp(anguloEnRadianes);

    odometria = rossubscriber('/robot0/odom');
    publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

    mensajeMovimiento = rosmessage(publisher);
    mensajeMovimiento.Angular.Z = 0.0;

    if(angulo > 0)
        mensajeMovimiento.Angular.Z = 0.1;
    else
        mensajeMovimiento.Angular.Z = -0.1;
    end

    pause(1);
    while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0') ~= 1)
        odometria.LatestMessage
    end

    r = robotics.Rate(10);

    posicionInicial = odometria.LatestMessage.Pose.Pose.Orientation;
    posicionAngular = [posicionInicial.W, posicionInicial.X, posicionInicial.Y, posicionInicial.Z];

    posicionObjetivo = quat2angle(posicionAngular, 'ZYX') + anguloEnRadianes;

    while(1)
        posicionActual = odometria.LatestMessage.Pose.Pose.Orientation;
        posicionAngular = [posicionActual.W, posicionActual.X, posicionActual.Y, posicionActual.Z];
        yaw = quat2angle(posicionAngular, 'ZYX');
        disp(posicionObjetivo);
        disp(yaw);
        send(publisher, mensajeMovimiento);

        if(angulo < 0)
            if(yaw < posicionObjetivo)
                mensajeMovimiento.Angular.Z = 0.0;
                send(publisher, mensajeMovimiento);
                break;
            end
        else
            if(yaw > posicionObjetivo)
                mensajeMovimiento.Angular.Z = 0.0;
                send(publisher, mensajeMovimiento);
                break;
            end
        end

        waitfor(r);
    end
    send(publisher, mensajeMovimiento);
end