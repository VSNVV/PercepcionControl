%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

%% INICIALIZACION DE ROS
rosinit;

%% SUBSCRIPCION AL LASER
laser = rossubscriber('/scan');

%% ALGORITMO

listaDistancias = [];
listaIteraciones = [];

for i = 0:1000
    mensajeLaser = receive(laser, 1);
    distanciasLaser = mensajeLaser.Ranges;
    listaDistancias = [listaDistancias, distanciasLaser];
    listaIteraciones = [listaIteraciones, i];

    disp(i);
end

plot(listaIteraciones, listaDistancias);
xlabel('Iteraciones');
ylabel('Distancias');

%% DESCONEXION DE ROS
rosshutdown;