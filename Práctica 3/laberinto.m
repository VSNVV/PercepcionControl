%% Conexion a ros
clear;
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.241.129:11311') % IP de la MV
setenv('ROS_IP','192.168.1.129') % IP de nuestro ordenador
rosinit;


%% SUBSCRIBERS
odometria = rossubscriber('/robot0/odom');
laser = rossubscriber('/robot0/laser_1');

%% PUBLISHERS
publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
%% MENSAJE
mensajeMovimiento = rosmessage(publisher);

robotRate = robotics.Rate(10);
pause(1);


while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0')~=1)
 odom.LatestMessage
end

idList=[];
globalPose=getGlobalCoords(odometria);
pose=[0,0,0];%% X,Y,ori
shortestPath=[];
nodeActual=createNode(idList);
idList=updateIdList(idList,nodeActual.id,0,0);

nodeToVisit=[];

while(true)
idList=setVisited(idList,nodeActual.id,1);

paredesDetectadas=detectarParedes(laser,pose(3));

[idList,nodeList]=addNode(idList,paredesDetectadas,pose(1),pose(2));

nodeActual=conectNode(nodeActual,nodeList);

nodeToVisit=nextNode(idList,nodeActual);

disp("A")

shortestPath=dijkstra(nodeActual,resetIdList(idList),searchId(idList,nodeToVisit));

pose(1:2)=travelPath(shortestPath,pose(1),pose(2));

end

rosshutdown;
function shortestPath=dijkstra(node,idList,xDes,yDes)
    id=node.id;
    [x,y]=searchId(idList,id);
    adjacentNodes=[node.down,node.right,node.up,node.left];
    shortestPathList=[[],[],[],[]];
    for i= 1,length(adjacentNodes)
        if ~isempty(adjacentNodes(i))
            if ~isVisited(idList,adjacentNodes(i).id) || ~isFinish(node) ||~(x==xDes && y==yDes) 
                shortestPathList(i)=dijkstra(adjacentNodes(i),idList);
            end
        end
    end

    path=[];

    for i=1:length(shortestPathList)
        if ~isempty(shortestPathList(i))
            if isempty(path)
                path= shortestPathList(i);
            elseif shortestPathList(i)<path       
                path= shortestPathList(i);
            end
        end
    end
    shortestPath=[id,path(:)];
end

function [xLocal,yLocal]= travelPath(idList,shortestPath,x,y)
    xLocal=x;
    yLocal=y;
    for i=1:length(shortestPath)
        [xDes,yDes]=searchId(idList,shortestPath(i));
        mover(x,y,xDes,yDes);
        xLocal=xDes;
        yLocal=yDes;
    end
end

function nodeToVisit=nextNode(idList,node)
    nodeList=[node.down,node.right,node.up,node.left];
    nodeToVisit=[];
    for i=1:length(nodeList)
        if ~isVisited(idList,nodeList(i).id)
            nodeToVisit=nodeList(i);
        end
    end
    if isempty(nodeToVisit)
        for i=1:size(idList)
            if idList(i,end)==0
                nodeToVisit=idList(i);
            end
        end
    end
end

function paredesDetectadas=detectarParedes(laser,robotOri)
    tolerancia = sqrt(2);
    mensajeLaser = receive(laser, 1).Ranges;
    listaLasers=[mensajeLaser(1),mensajeLaser(100),mensajeLaser(200),mensajeLaser(300)];%% trasero, derecho, frontal, izquierdo
    paredesDetectadas=1:length(listaLasers);
    for i= 1:length(listaLasers)
        if listaLasers(i)>tolerancia
            paredesDetectadas(i)=0;
        else 
            paredesDetectadas(i)=1;
        end
    end
    paredesDetectadas=desplazarLista(paredesDetectadas,robotOri);
end

function paredes=desplazarLista(paredesDetectadas,robotOri)
    oriObjetivo= mod(90-robotOri,360);%% se utiliza 90 para obtener la referencia con el robot mirando hacia arriba
    numDesp =round(oriObjetivo/90);
    paredes=paredesDetectadas;
    if numDesp>0
        for i=1:numDesp
            paredes=[paredes(2:end),paredes(1)];
        end
    end
end

function newNode = createNode(idList)
    newNode.id = newId(idList);
    newNode.left = [];
    newNode.right = [];
    newNode.up = [];
    newNode.down = [];
end

function list= updateIdList(idList, id, x, y)
    visitado=0;
    list = [idList; id, x, y, visitado];
end

function list= resetIdList(idList)
    for i=1:length(idList)
        idList(i,4)=0;
    end

    list=idList;
end

function [list, nodeList] = addNode(idList, listaParedes, x, y)
    listaDesplazamientos = [0, -2; 2, 0; 0, 2; -2, 0]; %% trasero, derecho, frontal, izquierdo
    nodeList = cell(1, length(listaParedes)); % Inicializa nodeList como un cell array

    for i = 1:length(listaParedes)
        if listaParedes(i) == 0
            xDest = x + listaDesplazamientos(i, 1); 
            yDest = y + listaDesplazamientos(i, 2);
            
            if isempty(searchCoords(idList, xDest, yDest)) % Si no están las coordenadas
                newNode = createNode(idList);  % Crea el nodo
                idList = updateIdList(idList, newNode.id, xDest, yDest); % Actualiza la lista
                nodeList{i} = newNode; % Actualiza nodeList con el nuevo nodo
            else
                
            end
        else
            nodeList{i} = []; % Agrega un array vacío para mantener el índice
        end
    end
    list = idList;
end
function node= conectNode(actualNode,nodelist)

    for i= 1:length(nodelist)
        if ~isempty(nodelist{i})
         switch i
            case 1  %trasero
                nodelist{i}.up = actualNode;
                actualNode.down = nodelist{i};
            case 2  %derecho
                nodelist{i}.left = actualNode;
                actualNode.right = nodelist{i};
            case 3  %frontal
                nodelist{i}.down = actualNode;
                actualNode.up = nodelist{i};
            case 4  %izquierdo
                nodelist{i}.right = actualNode;
                actualNode.left = nodelist{i};
            otherwise
                error('Dirección no válida');
          end
        end
    end
    node=actualNode;
end

function caso =isFinish(node)
    nodeList=[node.down,node.right,node.up,node.left];
    caso=false;
    for i=1:length(nodeList)
        if isempty(nodeList(i))
            caso=true;
        end
    end
end
    
function caso = isVisited(idList, id)
    idx = find(idList(:, 1) == id); % Encuentra las filas donde el primer elemento es igual a id
    if isempty(idx)
        caso = [];
    else
        if idList(idx, 1) == 0
            caso = false;
        else
            caso = true;
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

function id = newId(idList)

id= size(idList,1);

end

function avanzar(distancia,odometria,publisher)


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

function [x,y]= getGlobalCoords(odometria)

pos=odometria.LatestMessage.Pose.Pose.Position;
x=pos.X;
y=pos.Y;
end
    