%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.166:11311')  %Aqui poner la ip de ubuntu 
setenv('ROS_IP','192.168.1.132') %Aqui poner la ip de windows

%% INICIALIZACION DE ROS
rosinit;

%% SUBSCRIBER A LA ODOMETRIA
odometria = rossubscriber('/robot0/odom');

%% ASEGURAMOS QUE NOS LLEGUE UN MENSAJE DEL ROBOT
pause(1);

while (strcmp(odometria.LatestMessage.ChildFrameId,'robot0') ~= 1)
        odometria.LatestMessage
end

%% IMPRIMIMOS EL CONTENIDO DEL MENSAJE DE ODOMETRIA
disp(odometria.LatestMessage)

%% DESCONEXIÓN DE ROS
rosshutdown;