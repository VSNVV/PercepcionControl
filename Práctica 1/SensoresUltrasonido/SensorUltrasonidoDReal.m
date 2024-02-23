%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.241.129:11311') % IP de la MV
setenv('ROS_IP','192.168.1.156') % IP de nuestro ordenador

%% INICIALIZACION DE ROS
rosinit;

%% SUBSCRIBERS
sonar2 = rossubscriber('/sonar_2');

%% ALGORITMO
listaDistancias = [];
listaDistanciasFiltradas = [];
listaIteraciones = [];

for i = 0:1000
    
    mensajeSonar = receive(sonar3, 1);
    distanciaActual = mensajeSonar.Range_;

    listaDistancias = [listaDistancias, distanciaActual];

    if(length(listaDistancias) == 5)
        distanciaFiltrada = mean(listaDistancias(end - 4: end));
        listaDistanciasFiltradas = [listaDistanciasFiltradas, distanciaFiltrada];
    else
        listaDistanciasFiltradas = [listaDistanciasFiltradas, distanciaActual];
    end

    listaIteraciones = [listaIteraciones, i];

    disp(i);
end

plot(listaIteraciones, listaDistanciasFiltradas);
xlabel('Tiempo');
ylabel('Distancia Filtrada');

%% DESCONEXION DE ROS
rosshutdown;