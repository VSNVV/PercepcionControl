%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

%% SUBSCRIPCION AL LASER
laser = rossubscriber('/robot0/laser_1');

%% ALGORITMO

listaDistancias = [];
listaDistanciasFiltradas = [];
listaIteraciones = [];

for i = 0:1000
    mensajeLaser = receive(laser, 1);
    distanciasLaser = mensajeLaser.Ranges;
    