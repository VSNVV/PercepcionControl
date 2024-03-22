%% Conexion a ros

setenv('ROS_MASTER_URI','http://192.168.0.19:11311')  % IP de la MV
setenv('ROS_IP','192.168.0.16') % IP de nuestro ordenador (windows)
rosinit;

%% SUBSCRIBERS
odometria = rossubscriber('/robot0/odom');
%% PUBLISHERS
publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
%% MENSAJE
mensajeMovimiento = rosmessage(publisher)

robotRate = robotics.Rate(10);
pause(1);

while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
 odom.LatestMessage
end

%% ALGORITMO DE CONTROL DE POSICION

% Pedimos al ususario las coordenadas de destino por teclado
x_objetivo = input('Introduce la coordenada X de destino: ');
y_objetivo = input('Introduce la coordenada Y de destino: ');

% Una vez cogidas coordenadas de destino, establecemos el punto de partida
tic

% Inicializamos las variables del ALGORITMO
Ka = 1.0; % Constante de proporcionalidad del control de la orientacion
Kp = 0.5; % Constante de proporcionalidad del control de la velocidad
tolerancia = 0.1
activo = true;

while activo
    % Leemos la posición actual del robot
    x_actual = odometria.LatestMessage.Pose.Pose.Position.X;
    Y_actual = odometria.LatestMessage.Pose.Pose.Position.Y;

    % Ahora haremos lo mismo pero con la posicion angular del robot
    posAngular = odometria.LatestMessage.Pose.Pose.Orientation;
    posiciónAngularActual = [posAngular.W, posAngular.X, posAngular.Y, posAngular.Z];
    [yaw, pitch, roll] = quat2angle(posicionAngularActual, 'ZYX');

    % Calcuamos el error tanto de distancia como de objetivo
    errorDistancia = sqrt((x_objetivo - x_actual)^2 + (y_objetivo - y_actual)^2);
    errorOrientacion = atan2(y_objetivo - y_actual, x_objetivo - x_actual) - yaw;

    % Segun el error de distancia, podremos modelar la velocidad lineal que vamos a seguir de la siguiente manera:
    velocidadLineal = Kp * errorDistancia;
    velocidadLineal = min(velocidadLineal, 0.5); % Limitamos la velocidad lineal a 0.5 como maximo

    % Hacemos lo mismo con el error de orientacion de la siguiente manera:
    velocidadAngular = Ka * errorOrientacion;
    velocidadAngular = min(velocidadAngular, 1) % Limitamos la velocidad angular a 1 como maximo

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
    end;

    % Establecemos un tiempo de espera para que el sistema no se sobrecargue
    waitfor(robotRate);
end;

% Una vez finalizado en algoritmo, detenemos el robot:
mensajeMovimiento.Linear.X = 0.0;
mensajeMovimiento.Angular.Z = 0.0;
send(publisher, mensajeMovimiento);

% Una vez detenemos el robot nos desconectamos de ROS
rosshutdown;