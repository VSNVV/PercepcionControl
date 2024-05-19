%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.134:11311') % IP de la MV
setenv('ROS_IP','192.168.1.133') % IP de nuestro ordenador

%% EJECUCION DEL CODIGO
mover([0,0], [0,2]);
%avanzar(2);



%% FUNCION DE AVANZAR
function avanzar(distancia)
    rosshutdown;
    rosinit;

    odometria = rossubscriber('/robot0/odom');
    publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

    mensajeMovimiento = rosmessage(publisher);

    pause(1);
    while (strcmp(odometria.LatestMessage.ChildFrameId, 'robot0') ~= 1)
        odometria.LatestMessage
    end

    distanciaRecorrida = 0;
    % Ahora tendremos que pasar nuestras coordenadas locales conocidas a globales para medir el incremento de posicion
    posicionInicial = local2Global([0,0], [13,3]); % Aqui tienes que poner el getter de las coordenadas actuales locales y las iniciales globales respectivamente
    disp(posicionInicial);
    Kp = 3;
    
    while (not(distancia - 0.00001 <= distanciaRecorrida) && (distanciaRecorrida <= distancia + 0.00001))
        % Leemos nuestra posición actuale
        posicionActual = odometria.LatestMessage.Pose.Pose.Position;
        disp(posicionActual);
        % Mediante la formula del modulo de un vector, podemos ver cuanta distancia hemos recorrido desde el punto de inicio
        distanciaRecorrida = sqrt((posicionInicial(1) - posicionActual.X)^2 + (posicionInicial(2) - posicionActual.Y)^2);
        mensajeMovimiento.Linear.X = min(Kp * (distancia - distanciaRecorrida), 1);
        send(publisher, mensajeMovimiento);
        disp(distanciaRecorrida);
    end
    disp("Fin avanzar");
    mensajeMovimiento.Linear.X = 0.0;
    send(publisher, mensajeMovimiento);
end

%% FUNCION PARA GIRAR
function girar(angulo)
    rosshutdown;
    rosinit;

    % SUBSCRIBERS
    odometria = rossubscriber('/robot0/odom');
    % PUBLISHERS
    publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
    % MENSAJE
    mensajeMovimiento = rosmessage(publisher);

    pause(1);

    while(strcmp(odometria.LatestMessage.ChildFrameId,'robot0') ~= 1)
    odom.LatestMessage
    end

    % VARIABLES NECESARIAS
    activo = true;
    iteracion = 0;
    sentidoHorario = false;
    Kp = 2;

    while (activo)
        posicionActual = odometria.LatestMessage.Pose.Pose.Orientation;
        posicionAngular = [posicionActual.W, posicionActual.X, posicionActual.Y, posicionActual.Z];
        yaw = quat2angle(posicionAngular, 'ZYX');
        yaw = round(yaw, 4); % Acotamos a 4 decimales el resultado
        disp(['Yaw (radianes): ', num2str(yaw)]);
        if(iteracion == 0)
            % Ahora convertimos estos grados a los radianes del robot
            radianesFinal = getRadianes(mod(angulo, 360));
            % Ahora tenemos que ver el sentido en el que giramos
            if(mod(round(rad2deg(yaw)) + 90, 360) == angulo)
                sentidoHorario = false;
            elseif(mod(round(rad2deg(yaw)) - 90, 360) == angulo)
                sentidoHorario = true;
            else
                sentidoHorario = false;
            end
            iteracion = iteracion + 1;
        end
        controlGiro = min(Kp * abs(radianesFinal - yaw), 1); % Acotamos la velocidad maxima de giro a 1
        mensajeMovimiento.Angular.Z = controlGiro;
        mensajeMovimiento.Linear.X = 0;
        if(sentidoHorario)
            mensajeMovimiento.Angular.Z = mensajeMovimiento.Angular.Z * -1;
        end
        send(publisher, mensajeMovimiento);

        if((radianesFinal - 0.00001 <= yaw) && (yaw <= radianesFinal + 0.00001))
            activo = false;
            mensajeMovimiento.Angular.Z = 0.0;
            send(publisher, mensajeMovimiento);
        end
    end
    disp("Final del algoritmo");
end

%% FUNCION PARA CONVERTIR A RADIANES EL ANGULO DE ENTRADA
function radianes = getRadianes(angulo)
    if((0 <= angulo) && (angulo <= 180))
        radianes = round(deg2rad(angulo), 4);
    else
        radianes = round(deg2rad(angulo - 360), 4);
    end

end

%% FUNCION QUE CONVIERTE COORDENADAS LOCALES EN GLOBALES
function coordenadas_globales = local2Global(coordenadas_locales, origen_global)
    coordenadas_globales = coordenadas_locales + origen_global;
end

%% FUNCION PARA MOVERNOS A UNA COORDENADA LOCAL
function mover(coords_locales_actuales, coords_locales_destino)
    % Tenemos las coordenadas locales hacia donde queremos ir, por tanto, nos orientaremos hacia donde queremos ir
    orientacion = calcularOrientacion(coords_locales_actuales, coords_locales_destino); % Aqui tienes que poner el getter de las coords actuales en coords_locales_actuales
    % Una vez que tengamos la orientacion, giramos hacia ella
    girar(orientacion);
    % Una vez que estemos ya orientados hacia el punto, simplemente avanzamos los 2 metros que equidistan de todas las casillas
    avanzar(2);
end

function angulo = calcularOrientacion(coords_locales_origen, coords_locales_destino)
    % Descomposición de los puntos
    x1 = coords_locales_origen(1);
    y1 = coords_locales_origen(2);
    x2 = coords_locales_destino(1);
    y2 = coords_locales_destino(2);
    
    % Calcular las diferencias en x e y
    dx = x2 - x1;
    dy = y2 - y1;
    
    % Calcular el ángulo en radianes
    theta = atan2(dy, dx);
    
    % Convertir el ángulo a grados
    angulo = rad2deg(theta);
    
    % Asegurarse de que el ángulo está en el rango [0, 360)
    if (angulo < 0)
        angulo = angulo + 360;
    end
end
