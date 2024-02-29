%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://172.29.30.178:11311') % IP de la MV
setenv('ROS_IP','172.25.192.1') % IP de nuestro ordenador

%% INICIALIZACION DE ROS
rosinit;

%% SUBSCRIBERS
sonar3 = rossubscriber('/sonar_2');

%% ALGORITMO
listaDistancias = [];
listaIteraciones = [];

for i = 0:1000

    mensajeSonar = receive(sonar3, 1);
    distanciaSonar = mensajeSonar.Range_;

    listaDistancias = [listaDistancias, distanciaSonar];
    listaIteraciones = [listaIteraciones, i];

    disp(i);
end

plot(listaIteraciones, listaDistancias);
xlabel('Tiempo');
ylabel('Distancia');

%% DESCONEXION DE ROS
rosshutdown;