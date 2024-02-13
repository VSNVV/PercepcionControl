setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

rosinit;

sonar3 = rossubscriber('/robot0/sonar_3');

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
    