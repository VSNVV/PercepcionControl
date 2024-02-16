%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.45:11311') % IP de la MV
setenv('ROS_IP','192.168.1.42') % IP de nuestro ordenador

rosinit;

%% SUBSCRIBER AL LASER
laser = rossubscriber('/robot0/laser_1');
mensajeLaser = receive(laser, 1);

disp(mensajeLaser);

%% DESCONEXION DE ROS
rosshutdown;