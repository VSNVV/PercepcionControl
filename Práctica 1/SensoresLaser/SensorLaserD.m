%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://172.29.29.59:11311') % IP de la MV
setenv('ROS_IP','172.29.29.55') % IP de nuestro ordenador

%% INICIALIZACION DE ROS
rosinit;

%% SUBSCRIPCION AL LASER
laser = rossubscriber('/robot0/laser_1');

%% ALGORITMO
listaDistancias = [];
listaDistanciasFiltradas = [];
listaIteraciones = [];
contador = 0;

for i = 0:1000
    mensajeLaser = receive(laser, 1);
    distanciaLaser = mensajeLaser.Ranges(200);

    listaDistancias = [listaDistancias, distanciaLaser];

    if(length(listaDistancias) == 5)
        distanciaMedia = mean(listaDistancias);
        listaDistanciasFiltradas = [listaDistanciasFiltradas, distanciaMedia];

        listaDistancias = [];

        listaIteraciones = [listaIteraciones, contador];
        contador = contador + 1;
    end

    disp(i);
end

plot(listaIteraciones, listaDistanciasFiltradas);
xlabel('Tiempo');
ylabel('Distancia Filtrada');