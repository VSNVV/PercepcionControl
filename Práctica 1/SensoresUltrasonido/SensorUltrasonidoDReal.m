%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://172.29.30.178:11311') % IP de la MV
setenv('ROS_IP','192.168.56.1') % IP de nuestro ordenador
rosshutdown;
%% INICIALIZACION DE ROS
rosinit;

%% SUBSCRIBERS
sonar2 = rossubscriber('/sonar_2');

%% ALGORITMO
listaDistancias = [];
listaDistanciasFiltradas = [];
listaIteraciones = [];
i=0;
while(i)
    
    mensajeSonar = receive(sonar2, 10);
    distanciaActual = mensajeSonar.Range_;
    if(~isinf(distanciasLaser))
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
end

plot(listaIteraciones, listaDistanciasFiltradas);
xlabel('Tiempo');
ylabel('Distancia Filtrada');

%% DESCONEXION DE ROS
rosshutdown;