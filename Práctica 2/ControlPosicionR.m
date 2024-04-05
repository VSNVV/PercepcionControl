
%% Conexion a ros
rosshutdown;
setenv('ROS_MASTER_URI','http://172.29.30.172:11311') % IP de la MV
setenv('ROS_IP','172.29.29.85') % IP de nuestro ordenador
rosinit;


%% SUBSCRIBERS
odometria = rossubscriber('/pose');
pubActivaMotor = rospublisher('/cmd_motor_state','std_msgs/Int32');
%% PUBLISHERS
publisher = rospublisher('cmd_vel', 'geometry_msgs/Twist'); 
%% MENSAJE
mensajeMovimiento = rosmessage(publisher);
mensajeActivarMotor = rosmessage(pubActivaMotor);

robotRate = robotics.Rate(10);
pause(1);

while (strcmp(odometria.LatestMessage.ChildFrameId,'base_link') ~= 1)
 odometria.LatestMessage
end

%% ALGORITMO DE CONTROL DE POSICION
% Activamos motores del robot
mensajeActivarMotor.Data = 1;
send(pubActivaMotor, mensajeActivarMotor);

% Pedimos al ususario las coordenadas de destino por teclado
x_objetivo = input('Introduce la coordenada X de destino: ');
y_objetivo = input('Introduce la coordenada Y de destino: ');

% Una vez cogidas coordenadas de destino, establecemos el punto de partida
tic

% Inicializamos las variables del ALGORITMO
errorIntegralDistancia =0;
errorIntegralDistanciaAnterior=0;
errorIntegralAngular =0;
errorIntegralAngularAnterior=0;
Kpa = 0.5; % Constante de proporcionalidad del control de la orientacion
Kpd = 0.1; % Constante de proporcionalidad del control de la velocidad
Kia = 0.0005; % Constante de integral del control de la orientacion
Kid = 0.001; % Constante de integral del control de la velocidad
tolerancia = 0.1;
activo = true;

while activo
    % Leemos la posición actual del robot
    x_actual = odometria.LatestMessage.Pose.Pose.Position.X;
    y_actual = odometria.LatestMessage.Pose.Pose.Position.Y;

    % Ahora haremos lo mismo pero con la posicion angular del robot
    posAngular = odometria.LatestMessage.Pose.Pose.Orientation;
    posicionAngularActual = [posAngular.W, posAngular.X, posAngular.Y, posAngular.Z];
    [yaw, pitch, roll] = quat2angle(posicionAngularActual, 'ZYX');

    % Calcuamos el error tanto de distancia como de objetivo
    errorDistancia = sqrt((x_objetivo - x_actual)^2 + (y_objetivo - y_actual)^2);
    errorOrientacion = atan2(y_objetivo - y_actual, x_objetivo - x_actual) - yaw;
    
    errorIntegralDistancia = errorIntegralDistanciaAnterior + errorDistancia * robotRate.DesiredPeriod;
    errorIntegralDistanciaAnterior= errorIntegralDistancia;

    errorIntegralAngular =errorIntegralAngularAnterior+errorOrientacion*robotRate.DesiredPeriod;
    errorIntegralAngularAnterior=errorIntegralAngular;


    % Segun el error de distancia, podremos modelar la velocidad lineal que vamos a seguir de la siguiente manera:
    velocidadLineal = Kpd * errorDistancia + Kid*errorIntegralDistanciaAnterior;
    velocidadLineal = min(velocidadLineal, 1); % Limitamos la velocidad lineal a 0.5 como maximo

    % Hacemos lo mismo con el error de orientacion de la siguiente manera:
    velocidadAngular = Kpa * errorOrientacion+ Kia*errorIntegralAngularAnterior;
    velocidadAngular = min(velocidadAngular, 0.5); % Limitamos la velocidad angular a 1 como maximo

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

% Una vez detenemos el robot nos desconectamos de ROS
rosshutdown;