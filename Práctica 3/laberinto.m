%% Conexion a ros
clear;
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.241.129:11311') % IP de la MV
setenv('ROS_IP','192.168.1.134') % IP de nuestro ordenador
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
pose=[0,0,0];%% X,Y,ori
shortestPath=[];
nodeActual=[];

paredesDetectadas=detectarParedes(laser,pose(3));
[idList,nodeList]=addNode(idList);
node=conectNode(node,nodeList);



rosshutdown;
function shortestPath=dijkstra(node,idList)
    idList()
    adjacentNodes=[node.down,node.right,node.up,node.left];
    shortestPathList=[[],[],[],[]];
    for i= 1,length(adjacentNodes)
        if ~isempty(adjacentNodes(i))
            if ~isVisited(adjacentNodes(i).id)|| ~isFinish(node)  
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

function newNode = createNode(id)
    newNode.id = id;
    newNode.left = [];
    newNode.right = [];
    newNode.up = [];
    newNode.down = [];
end

function idList= updateIdList(idList, id, x, y)
    visitado=0;
    idList = [idList; id, x, y, visitado];
end

function [list, nodeList]= addNode(idList,listaParedes,x,y)
    listaDesplazamientos=[[0,-2],[2,0],[0,2],[-2,0]];%%trasero, derecho, frontal, izquierdo
    nodeList=1:length(listaParedes);

    for i =1:length(listaParedes)

        if listaParedes(i)==0
           coords=[x+listaDesplazamientos(i,1),y+listaDesplazamientos(i,2)];%se establecen las coordenadas

           if isempty(searchCoords(idList,coords(1),coords(2)))% si no están las coordenadas
               nodeList(i)=createNode(newId(idList));  %se crea el nodo
               idList=updateIdList(IdList,newId(IdList),coords(1),coords(2)); %se actualiza la lista
           end
        else
            nodeList(i)=[];
        end
    end
    list=idList;
end

function node= conectNode(actualNode,nodelist)

    for i= 1:length(nodelist)
     switch i
        case 1  %trasero
            nodelist(i).up = actualNode;
            actualNode.down = nodelist(i);
        case 2  %derecho
            nodelist(i).left = actualNode;
            actualNode.right = nodelist(i);
        case 3  %frontal
            nodelist(i).down = actualNode;
            actualNode.up = nodelist(i);
        case 4  %izquierdo
            nodelist(i).right = actualNode;
            actualNode.left = nodelist(i);
        otherwise
            error('Dirección no válida');
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
    
function caso= isVisited(id,idList)
    if isempty(idList)
        caso=[];
    else
    
        idx = find(idList(:, 1) == id);
        if isempty(idx)
            caso = [];
        else
            if idList(idx, 1)==0
                caso=false;
            else 
                caso=true;
            end
        end
    end
end

function id = searchCoords(idList, x,y)
    if isempty(idList)
        id=[];
    else
        idx = find(idList(:, 2) == x & idList(:, end) == y);

        if isempty(idx)
          id = [];
        else
          id = idList(idx, 1);
        end
    end
end

function coords = searchId(idList, id)
    if isempty(idList)
        id=[];
    else
        idx = find(idList(:, 1) == id);
        if isempty(idx) 
            coords = [];
        else
            coords = idList(idx, 2:3); % Devuelve las coordenadas [x, y]
        end
    end
end

function id = newId(idList)
id= length(idList);
end

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
        if velocidadAngular>1
            velocidadAngular=1;
        elseif velocidadAngular<-1
            velocidadAngular=-1;
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