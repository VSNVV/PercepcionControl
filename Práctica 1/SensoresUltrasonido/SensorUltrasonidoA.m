%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.166:11311')  %Aqui poner la ip de ubuntu 
setenv('ROS_IP','192.168.1.132') %Aqui poner la ip de windows

%% INICIALIZACIÓN DE ROS
rosinit;

%% SUBSCRIBERS
sonar3 = rossubscriber('/robot0/sonar_3');

%% RECIBIMOS INFORMACION DEL SONAR
mensajeSonar = receive(sonar3, 1);

%% IMPRIMIMOS EL CONTENIDO DEL MENSAJE DE SONAR
disp(mensajeSonar);

%% DESCONEXION DE UN ROS
rosshutdown;