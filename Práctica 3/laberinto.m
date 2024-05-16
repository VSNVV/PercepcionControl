%% Conexion a ros
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.241.129:11311') % IP de la MV
setenv('ROS_IP','192.168.1.134') % IP de nuestro ordenador
rosinit;


%% SUBSCRIBERS
odometria = rossubscriber('/robot0/odom');
%% PUBLISHERS
publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
%% MENSAJE
mensajeMovimiento = rosmessage(publisher);

robotRate = robotics.Rate(10);
pause(1);


while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0')~=1)
 odom.LatestMessage
end

%% ALGORITMO DE CONTROL DE POSICION

% Una vez cogidas coordenadas de destino, establecemos el punto de partida
tic

% Inicializamos las variables del ALGORITMO


desplazar(10,10);
desplazar (5,10);
desplazar(5,5);
desplazar(5,10);


% Una vez detenemos el robot nos desconectamos de ROS
rosshutdown;
function desplazar(x_objetivo,y_objetivo)
odometria = rossubscriber('/robot0/odom');
%% PUBLISHERS
publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
%% MENSAJE
mensajeMovimiento = rosmessage(publisher);
robotRate = robotics.Rate(10);
pause(1);

tic
while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0')~=1)
 odom.LatestMessage
end
margen = false;
Kpa = 0.05; % Constante de proporcionalidad del control de la orientacion
Kpd = 0.75; % Constante de proporcionalidad del control de la velocidad
tolerancia = 0.25;
activo = true;
while activo
    % Leemos la posición actual del robot
    x_actual = odometria.LatestMessage.Pose.Pose.Position.X;
    y_actual = odometria.LatestMessage.Pose.Pose.Position.Y;

    % Ahora haremos lo mismo pero con la posicion angular del robot
    posAngular = odometria.LatestMessage.Pose.Pose.Orientation;
    posicionAngularActual = [posAngular.W, posAngular.X, posAngular.Y, posAngular.Z];
    [yaw, ~, ~] = quat2angle(posicionAngularActual, 'ZYX');

    % Calcuamos el error tanto de distancia como de objetivo
    errorDistancia = sqrt((x_objetivo - x_actual)^2 + (y_objetivo - y_actual)^2);
    errorOrientacion = atan2(y_objetivo - y_actual, x_objetivo - x_actual) - yaw;
    errorOrientacion =corregirOrientacion(errorOrientacion);
    errorOrientacion = rad2deg(errorOrientacion);

    % Segun el error de distancia, podremos modelar la velocidad lineal que vamos a seguir de la siguiente manera:
    if(abs(errorOrientacion)<0.5 ||margen)
        if (abs(errorOrientacion)>0.25)
            margen =false;
        else
            margen =true;
        end
        velocidadAngular =0;
        velocidadLineal = Kpd * errorDistancia;
        velocidadLineal = min(velocidadLineal, 1);
    else
        velocidadLineal =0;
        velocidadAngular = Kpa * errorOrientacion;
        if velocidadAngular>0.5
            velocidadAngular=0.5;
        elseif velocidadAngular<-0.5
            velocidadAngular=-0.5;
        end
    end

    % Ahora enviamos el comando de movimiento al robot:
    mensajeMovimiento.Linear.X = velocidadLineal;
    mensajeMovimiento.Angular.Z = velocidadAngular;
    send(publisher, mensajeMovimiento);

    % En el caso de que hayamos llegado a la posicion objetivo, detendremos el algoritmo:
    if ((errorDistancia < tolerancia) && (abs(errorOrientacion) < tolerancia))
        % Se verifica que se ha llegado al objetivo, por tanto, marcamos el punto de finalizacion del algoritmo y lo imprimimos
        toc;
        disp(toc);
        % Indicamos la finalización del algoritmo
        activo = false;
    end

    % Establecemos un tiempo de espera para que el sistema no se sobrecargue
    waitfor(robotRate);
end

% Una vez finalizado en algoritmo, detenemos el robot:
mensajeMovimiento.Linear.X = 0.0;
mensajeMovimiento.Angular.Z = 0.0;
send(publisher, mensajeMovimiento);
end     

function E_ori_corregido = corregirOrientacion(E_ori)
    if E_ori <-pi
        E_ori = E_ori + (2*pi);
    end
    if E_ori > pi
        E_ori = E_ori - (2*pi);
    end
        E_ori_corregido = E_ori;
end 