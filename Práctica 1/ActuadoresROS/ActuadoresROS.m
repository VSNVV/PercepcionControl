girar(180);
avanzar(18);
avanzar(2);
girar(90);


function avanzar(distancia)
%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.0.16:11311')  %Aqui poner la ip de ubuntu 
setenv('ROS_IP','192.168.0.11') %Aqui poner la ip de windows

%% INCIALIZACION DE ROS
rosinit;

%% DECLARACIÓN DE SUBSCRIBERS
odometria = rossubscriber('/robot0/odom');
%% DECLARACIÓN DE PUBLISHERS
publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
%% GENERACIÓN DE MENSAJE
mensajeMovimiento = rosmessage(publisher)

mensajeMovimiento.Linear.X = 0.3;

r = robotics.Rate(10);
%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0"
pause(1);
while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0') ~= 1)
    odometria.LatestMessage
end

%% Inicializamos la primera posición (coordenadas x,y,z)
posicionInicial = odometria.LatestMessage.Pose.Pose.Position;
send(publisher, mensajeMovimiento);
 
%% ALGORITMO
while (distanciaRecorrida < distancia)
    posicionActual = odometria.LatestMessage.Pose.Pose.Position;
    disanciaRecorrida = sqrt((posicionIncial.X - posicionActual.X)^2 + (posicionInicial.Y - posicionActual.Y)^2)
    disp(distanciaRecorrida);
  
waitfor(r)
end

%% DESCONEXIÓN DE ROS
rosshutdown;
end

function girar(angulo)
    anguloEnRadianes = angulo * 3.1/180;
    disp(nuevo_angulo);
    %% INICIALIZACIÓN DE ROS
    setenv('ROS_MASTER_URI','http://192.168.0.16:11311')  %Aqui poner la ip de ubuntu 
    setenv('ROS_IP','192.168.0.11') %Aqui poner la ip de windows

    %% INICIALIZACIÓN DE ROS
    rosinit;

    %% SUBSCRIBERS
    odometria = rossubscriber('/robot0/odom'); % Subscripción a la odometría

    %% PUBLISHERS
    publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

    %% GENERACIÓN DE MENSAJE
    mensajeMovimiento = rosmessage(publisher)

    if (angulo > 0)

        r = robotics.Rate(10);
    
        %% Nos aseguramos recibir un mensaje relacionado con el robot "robot0"
        pause(1);
        while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0') ~= 1)
            odometria.LatestMessage
        end
        posicionInicial = odometria.LatestMessage.Pose.Pose.Orientation;
     
        while (1)
            posicionActual = odometria.LatestMessage.Pose.Pose.Orientation;
            posicionAngular = [posicionActual.W, posicionActual.X, posicionActual.Y, posicionActual.Z];
            [yaw, pitch, roll] = quat2angle(qpos, 'ZYX');
            disp(anguloEnRadianes);
    
            disp(['Yaw (radianes): ', num2str(yaw)]);
            
            if (yaw > anguloEnRadianes)
                mensajeMovimiento.Angular.Z = 0;
                send(publisher, mensajeMovimiento);
                break;
            else
            % Comando de velocidad
                send(publisher, mensajeMovimiento);
            end
        % Temporización del bucle según el parámetro establecido en r
        waitfor(r)
        end
    end 
if (angulo < 0)

    mensajeMovimiento.Angular.Z = -0.3;
    r = robotics.Rate(10);

    pause(1);
    while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0') ~= 1)
     odometria.LatestMessage
    end
    
    posicionInicial = odometria.LatestMessage.Pose.Pose.Orientation;
    
    send(publisher, mensajeMovimiento);

    while (1)
    %% Obtenemos la posición actual
        posicionActual = odometria.LatestMessage.Pose.Pose.Orientation;
        posicionAngular = [posicionActual.W, posicionActual.X, posicionActual.Y, posicionActual.Z];
    
        [yaw, pitch, roll] = quat2angle(posicionAngular, 'ZYX');
        disp(anguloEnRadianes);
        disp(['Yaw (radianes): ', num2str(yaw)]);

        if (yaw < anguloEnRadianes)
            mensajeMovimiento.Angular.Z = 0;
            send(publisher, mensajeMovimiento);
            break;
        else
            send(publisher, mensajeMovimiento);
        end
    waitfor(r)
    end
end

%% DESCONEXIÓN DE ROS
rosshutdown; 
end