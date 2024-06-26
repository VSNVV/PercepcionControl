%% CONEXION A ROS
setenv('ROS_MASTER_URI','http://172.29.30.173:11311') % IP de la MV
setenv('ROS_IP','172.29.29.56') % IP de nuestro ordenador

rosshutdown;
%% INICIALIZACION DE ROS
rosinit;

%% SUBSCRIPCION AL LASER
laser = rossubscriber('/scan');

%% ALGORITMO

listaDistancias = [];
listaIteraciones = [];
i=0;
while (i<100)
    
   
    mensajeLaser = receive(laser, 10);
    distanciasLaser = mensajeLaser.Ranges(200);
    if(~isinf(distanciasLaser))
        disp(distanciasLaser);
    i=i+1;
    listaDistancias = [listaDistancias, distanciasLaser];
    listaIteraciones = [listaIteraciones, i];

    disp(i);
    end
end

plot(listaIteraciones, listaDistancias);
xlabel('Iteraciones');
ylabel('Distancias');

%% DESCONEXION DE ROS
rosshutdown;