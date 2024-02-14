%setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
%setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

setenv('ROS_MASTER_URI','http://192.168.241.129:11311') % IP de la MV
setenv('ROS_IP','192.168.1.156') % IP de nuestro ordenador
rosinit;

Recta = struct('pendiente', 0, 'interseccion', 0);

sonarFrontalIzquierdo = rossubscriber('/robot0/sonar_2');
sonarFrontalDerecho = rossubscriber('/robot0/sonar_3');

sonarLateralIzquierdo = rossubscriber('/robot0/sonar_0');
sonarLateralDerecho = rossubscriber('/robot0/sonar_5');

sonarTraseroIzquierdo = rossubscriber('/robot0/sonar_7');
sonarTraseroDerecho = rossubscriber('/robot0/sonar_6');

ListaPuntosFrontales=[recive(sonarFrontalIzquierdo,1),recive(sonarFrontalIzquierdo,1)];
ListaPuntosTraseros=[receive(sonarTraseroIzquierdo,1),receive(sonarTraseroDerecho,1)];

ListaPuntosLateralesIzquierdos=[];
ListaPuntosLateralesDerechos=[];

rectaFrontal= calcularRecta(ListaPuntosFrontales(1),ListaPuntosFrontales(2),24);
rectaTrasera=calcularRecta(ListaPuntosTraseros(1),ListaPuntosTraseros(2),72);


rosshutdown;

function recta = calcularRecta(distancia1, distancia2, angulo)

    angulo_radianes = deg2rad(angulo);
    if distancia2 == 0 || sin(angulo_radianes) == 0
        % Si la distancia 2 es cero o el seno del ángulo es cero, la pendiente es infinita
        recta.pendiente = Inf;
    else
        recta.pendiente = cos(angulo_radianes) * (distancia2 - distancia1) / sin(angulo_radianes);
    end
    

    recta.interseccion = distancia1 - recta.pendiente * cos(angulo_radianes);
end

function gradoParalelismo = calcularGradoParalelismo(recta1, recta2)

    diferenciaPendientes = abs(recta1.pendiente - recta2.pendiente);
            gradoParalelismo = 100 - (diferenciaPendientes / (1 + abs(recta1.pendiente * recta2.pendiente))) * 100;
end

