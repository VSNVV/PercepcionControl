rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.241.129:11311') % IP de la MV
setenv('ROS_IP','192.168.1.129') % IP de nuestro ordenador
rosinit;    

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
tic;
pose=[0,0,0];%% X,Y,ori
nodeActual=nodeList{idxFromId(nodeList,0)};

[xDes,yDes]=searchEnd(idList);

shortestPath=busquedaProfundidad(nodeActual.id,nodeList,resetIdList(idList),xDes,yDes);

optimizedPath=optimizePath(idList,shortestPath);

[pose(1:2),pose(3),nodeActual]=travelPath(idList,optimizedPath,pose(1),pose(2),nodeList,odometria,publisher,globalPose);

a=toc;
disp(a);
rosshutdown;

function [xFin,yFin]= searchEnd(idList)
    idx=find(idList(:,5)==1);
    xFin=idList(idx,2);
    yFin=idList(idx,3);
end

function shortestPath = busquedaProfundidad(nodeId,nodeList, idList, xDes, yDes)
    idList=setVisited(idList,nodeId,1);
    node=nodeList{idxFromId(nodeList,nodeId)};
    [x, y] = searchId(idList, nodeId);
    adjacentNodes = {node.down, node.right, node.up, node.left};
    shortestPathList = cell(1, length(adjacentNodes));  % Initialize as a cell array to handle paths of varying lengths

    for i = 1:size(adjacentNodes,2)
        if ~isempty(adjacentNodes{i})
            adjNodeId = adjacentNodes{i};
            if ~isVisited(idList, adjNodeId) && ~(x == xDes && y == yDes)
                idList=setVisited(idList,adjNodeId,1);
                shortestPathList{i} = busquedaProfundidad(adjNodeId,nodeList, idList, xDes, yDes);  % Recursive call
            end
        end
    end

    % Initialize the path as an empty array
    path = [];

    % Select the shortest path from the list of paths
    for i = 1:length(shortestPathList)
        if ~isempty(shortestPathList{i})
            if shortestPathList{i}(end)==searchCoords(idList,xDes,yDes)
                if isempty(path)
                    path=shortestPathList{i};
                else
                    if length(shortestPathList{i})<length(path)
                        path=shortestPathList{i};
                    end
                end
            end
        end
    end

    shortestPath = [nodeId, path];
end

function [localCoords,yaw,node]= travelPath(idList,shortestPath,x,y,nodeList,odometria,publisher,globalPose)
localCoords=[x,y];
yaw=[];
node=nodeList{idxFromId(nodeList,shortestPath(1))};
    for i=1:length(shortestPath)
        

        [xDest,yDest]=searchId(idList,shortestPath(i));    
        destinyCoords=[xDest,yDest];
        yaw=mover(localCoords,destinyCoords,odometria,publisher,globalPose);
        localCoords=destinyCoords;
        node=nodeList{idxFromId(nodeList,shortestPath(i))};
    end
  
end

function optPath= optimizePath(idList,path)

    anguloAnterior=NaN;
    optPath=[];

    nodeid=path(1);
 
    [xIni,yIni]= searchId(idList,nodeid);
    coordIni(1)=xIni;
    coordIni(2)=yIni;
    for i=2:length(path)
        [xSig,ySig]= searchId(idList,path(i));
        coordSig(1)=xSig;
        coordSig(2)=ySig;
        anguloAct= calcularOrientacion(coordIni,coordSig);
        if ~isnan(anguloAnterior)
            if ~(anguloAnterior==anguloAct)
                optPath(end+1)=path(i-1);
            
            end
        end
        anguloAnterior=anguloAct;
        coordIni=coordSig;
    end

    optPath(end+1)=path(end);
end

function idx = idxFromId(nodelist, id)
    idx = [];
    for i = 1:length(nodelist)
        if nodelist{i}.id == id
            idx = i;
            return; % Salir de la función una vez encontrado el id
        end
    end
end

function [x,y] = searchId(idList, id)

    idx = find(idList(:, 1) == id);
    if isempty(idx) 
        x=[];
        y=[];
    else
        x= idList(idx,2);
        y= idList(idx,3);
    end
end

function id = searchCoords(idList, x,y)

    if isempty(idList)
        id=[];
    else
        idx = find(idList(:, 2) == x & idList(:, 3) == y);

        if isempty(idx)
          id = [];
        else
          id = idList(idx, 1);
        end
    end
end

function list= setVisited(idlist,id,visited)
    idx=find(idlist(:,1)==id);

    if ~isempty(idx)
        idlist(idx,4)=visited;
    end
    list=idlist;
end

function list= resetIdList(idList)
    for i=1:size(idList,1)
        idList(i,4)=0;
        idList(i,5)=0;
    end

    list=idList;
end

function caso = isVisited(idList, id)
    idx = find(idList(:, 1) == id); % Encuentra las filas donde el primer elemento es igual a id
    if isempty(idx)
        caso = [];
    else
        if idList(idx, 4) == 0
            caso = false;
        else
            caso = true;
        end
    end
end

function avanzar(distancia,odometria,publisher,localPos,globalPos)


    mensajeMovimiento = rosmessage(publisher);

    pause(1);
    while (strcmp(odometria.LatestMessage.ChildFrameId, 'robot0') ~= 1)
        odometria.LatestMessage
    end

    distanciaRecorrida = 0;
    % Ahora tendremos que pasar nuestras coordenadas locales conocidas a globales para medir el incremento de posicion
    posicionInicial = local2Global(localPos, globalPos); % Aqui tienes que poner el getter de las coordenadas actuales locales y las iniciales globales respectivamente
    disp(posicionInicial);
    Kp = 10;
    
    while (not(distancia - 0.1 <= distanciaRecorrida) && (distanciaRecorrida <= distancia + 0.1))
        % Leemos nuestra posición actuale
        posicionActual = odometria.LatestMessage.Pose.Pose.Position;
        %disp(posicionActual);
        % Mediante la formula del modulo de un vector, podemos ver cuanta distancia hemos recorrido desde el punto de inicio
        distanciaRecorrida = sqrt((posicionInicial(1) - posicionActual.X)^2 + (posicionInicial(2) - posicionActual.Y)^2);
        mensajeMovimiento.Linear.X = min(Kp * (distancia - distanciaRecorrida), 1)
        send(publisher, mensajeMovimiento);
        %disp(distanciaRecorrida);
    end
    disp("Fin avanzar");
    mensajeMovimiento.Linear.X = 0.0;
    send(publisher, mensajeMovimiento);
end

%% FUNCION PARA GIRAR
function girar(angulo,odometria,publisher)
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
    Kp = 7;

    while (activo)
        posicionActual = odometria.LatestMessage.Pose.Pose.Orientation;
        posicionAngular = [posicionActual.W, posicionActual.X, posicionActual.Y, posicionActual.Z];
        yaw = quat2angle(posicionAngular, 'ZYX');
        yaw = round(yaw, 4); % Acotamos a 4 decimales el resultado
        %disp(['Yaw (radianes): ', num2str(yaw)]);
        
        if(iteracion == 0)
            % Convertir el ángulo a radianes del robot
            radianesFinal = getRadianes(mod(angulo, 360));
            % Determinar el sentido de giro
            sentidoHorario = mod((rad2deg(radianesFinal) - rad2deg(yaw) + 360), 360) > 180;
            iteracion = iteracion + 1;
        end

        % Control de giro
        error = radianesFinal - yaw;
        % Ajustar el error para que esté en el intervalo [-pi, pi]
        error = wrapToPi(error);
        
        controlGiro = min(Kp * abs(error), 1) % Acotamos la velocidad máxima de giro a 1
        mensajeMovimiento.Angular.Z = controlGiro;
        mensajeMovimiento.Linear.X = 0;
        
        if(sentidoHorario)
            mensajeMovimiento.Angular.Z = -controlGiro;
        end
        
        send(publisher, mensajeMovimiento);

        % Condicion de parada
        if abs(error) < 0.01
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
function orientacion= mover(coords_locales_actuales, coords_locales_destino,odometria,publisher,globalPos)
    % Tenemos las coordenadas locales hacia donde queremos ir, por tanto, nos orientaremos hacia donde queremos ir
    orientacion = calcularOrientacion(coords_locales_actuales, coords_locales_destino); % Aqui tienes que poner el getter de las coords actuales en coords_locales_actuales
    % Una vez que tengamos la orientacion, giramos hacia ella
    girar(orientacion,odometria,publisher);
    % Una vez que estemos ya orientados hacia el punto, simplemente avanzamos los 2 metros que equidistan de todas las casillas
    xDiff=coords_locales_actuales(1)-coords_locales_destino(1);
    yDiff=coords_locales_actuales(2)-coords_locales_destino(2);

    if xDiff ~= 0
        distancia = abs(xDiff);
    else
        distancia = abs(yDiff);
    end
    avanzar(distancia,odometria,publisher,coords_locales_actuales,globalPos);
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
