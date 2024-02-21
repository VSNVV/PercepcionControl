%setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
%setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.241.129:11311') % IP de la MV
setenv('ROS_IP','172.31.176.1') % IP de nuestro ordenador
rosinit;

Recta = struct('pendiente', 0, 'interseccion', 0);
Punto = struct('ejeX', 0,'ejeY',0);
RadioRobot=0.15;

sonarFrontalIzquierdo = rossubscriber('/robot0/sonar_2');
sonarFrontalDerecho = rossubscriber('/robot0/sonar_3');

sonarLateralIzquierdo = rossubscriber('/robot0/sonar_0');
sonarLateralDerecho = rossubscriber('/robot0/sonar_5');

sonarTraseroIzquierdo = rossubscriber('/robot0/sonar_7');
sonarTraseroDerecho = rossubscriber('/robot0/sonar_6');

sonarDiagonalIzquierdo= rossubscriber('/robot0/sonar_1');
sonarDiagonalDerecho= rossubscriber('/robot0/sonar_4');

PuntoFrontalIzquierdo=calcularPunto(receive(sonarFrontalIzquierdo).Range_+RadioRobot,12);
PuntoFrontalDerecho=calcularPunto(receive(sonarFrontalDerecho).Range_+RadioRobot,348);

PuntoLateralIzquierdo=calcularPunto(receive(sonarLateralIzquierdo).Range_+RadioRobot,90);
PuntoDiagonalIzquierdo=calcularPunto(receive(sonarDiagonalIzquierdo).Range_+RadioRobot,44);

PuntoTraseroIzquierdo=calcularPunto(receive(sonarTraseroIzquierdo).Range_+RadioRobot,144);
PuntoTraseroDerecho=calcularPunto(receive(sonarTraseroDerecho).Range_+RadioRobot,216);

PuntoLateralDerecho=calcularPunto(receive(sonarLateralDerecho).Range_+RadioRobot,270);
PuntoDiagonalDerecho=calcularPunto(receive(sonarDiagonalDerecho).Range_+RadioRobot,316);






rectaFrontal= calcularRecta(PuntoFrontalIzquierdo,PuntoFrontalDerecho);
rectaTrasera=calcularRecta(PuntoTraseroIzquierdo,PuntoTraseroDerecho);
rectaIzquierdaDiagonal= calcularRecta(PuntoLateralIzquierdo,PuntoDiagonalIzquierdo);
rectaDerechaDiagonal= calcularRecta(PuntoLateralDerecho,PuntoDiagonalDerecho);
rectaIzquierdaEjeX= calcularRectaParalelaEjeX(PuntoLateralIzquierdo);
rectaDerechaEjeX= calcularRectaParalelaEjeX(PuntoLateralDerecho);
if isnan(rectaFrontal.pendiente)&& isnan(rectaTrasera.pendiente)

    rectaDerecha=rectaDerechaEjeX;
    rectaIzquierda=rectaIzquierdaEjeX;
else 
    if isnan(rectaFrontal.pendiente)
        rectaDerecha=determinarRecta(rectaDerechaDiagonal,rectaDerechaEjeX,rectaTrasera);
        rectaIzquierda=determinarRecta(rectaIzquierdaDiagonal,rectaIzquierdaEjeX,rectaTrasera);
    else
        rectaDerecha=determinarRecta(rectaDerechaDiagonal,rectaDerechaEjeX,rectaFrontal);
        rectaIzquierda=determinarRecta(rectaIzquierdaDiagonal,rectaIzquierdaEjeX,rectaFrontal);
    end
end


analizarRectas(rectaFrontal, rectaIzquierda, rectaTrasera, rectaDerecha);





%disp(calcularGradoParalelismo(rectaIzquierda,rectaDerecha));
%disp(calcularGradoParalelismo(rectaTrasera,rectaFrontal));
%disp(calcularAnguloEntreRectas(rectaFrontal,rectaIzquierda));

%disp(rectaFrontal);
%disp(rectaIzquierda);
%disp(rectaDerecha);
%disp(rectaTrasera);

rosshutdown;

function punto = calcularPunto(distancia, angulo)
   if isinf(distancia)
    punto.ejeX = NaN;
    punto.ejeY = NaN;
   else
    angulo_radianes= deg2rad(angulo);
    punto.ejeX = cos(angulo_radianes)* distancia;
    punto.ejeY = sin(angulo_radianes)* distancia;
   end
end

function recta = calcularRecta(punto1, punto2)
    if (isnan(punto1.ejeX) || isnan(punto2.ejeX))
        recta.pendiente= NaN;
        recta.interseccion =NaN;
    else
        if punto2.ejeX - punto1.ejeX == 0
            recta.pendiente = (punto2.ejeY - punto1.ejeY) / (punto2.ejeX - punto1.ejeX+0.001); %se añade un ligero desplazamiento para que la recta no sea infinita
        else
            recta.pendiente = (punto2.ejeY - punto1.ejeY) / (punto2.ejeX - punto1.ejeX);
        end
    recta.interseccion = punto1.ejeY - recta.pendiente * punto1.ejeX;
    end
end

function recta = calcularRectaParalelaEjeX(punto)
    if isnan(punto.ejeX)
        recta.pendiente= NaN;
        recta.interseccion =NaN;
    else
        recta.pendiente = 0;
        recta.interseccion = punto.ejeY;
    end
end


function gradoParalelismo = calcularGradoParalelismo(recta1, recta2)

    diferenciaPendientes = abs(recta1.pendiente - recta2.pendiente);
    gradoParalelismo = 100 - (diferenciaPendientes / (1 + abs(recta1.pendiente * recta2.pendiente))) * 100;
end

function recta = determinarRecta(rectaLateralDiagonal, rectaParalelaEjeX, rectaFrontalTrasera)
    if isnan(rectaLateralDiagonal.pendiente)
        recta = rectaParalelaEjeX;
    else
        angulo_rectaLateralDiagonal = abs(calcularAnguloEntreRectas(rectaLateralDiagonal, rectaFrontalTrasera) - 90);       
        angulo_rectaParalelaEjeX = abs(calcularAnguloEntreRectas(rectaParalelaEjeX, rectaFrontalTrasera) - 90);
        
        if angulo_rectaLateralDiagonal <= angulo_rectaParalelaEjeX
            recta = rectaLateralDiagonal;
        else
            recta = rectaParalelaEjeX;
        end
    end
end

function angulo_grados = calcularAnguloEntreRectas(recta1, recta2)
    % Calcula el ángulo entre las dos rectas en radianes
    angulo_radianes = atan((recta2.pendiente - recta1.pendiente) / (1 + recta1.pendiente * recta2.pendiente));
    
    % Convierte el ángulo de radianes a grados
    angulo_grados = abs(rad2deg(angulo_radianes));
end

function analizarRectas(rectaFrontal, rectaIzquierda, rectaTrasera, rectaDerecha)
    listaRectas=[rectaFrontal,rectaIzquierda,rectaTrasera,rectaDerecha];
    % Verificar si las rectas existen
    existen = zeros(1, 4);
    
    if ~isnan(rectaFrontal.pendiente)
        existen(1) = 1;
    end
    
    if ~isnan(rectaIzquierda.pendiente)
        existen(2) = 1;
    end
    
    if ~isnan(rectaTrasera.pendiente)
        existen(3) = 1;
    end
    
    if ~isnan(rectaDerecha.pendiente)
        existen(4) = 1;
    end

    
    fprintf('Existencia de rectas:\n');
    fprintf('Recta Frontal: %d\n', existen(1));
    fprintf('Recta Izquierda: %d\n', existen(2));
    fprintf('Recta Trasera: %d\n', existen(3));
    fprintf('Recta Derecha: %d\n', existen(4));
    
    % Calcular el grado de paralelismo y el ángulo entre las rectas existentes
    for i = 1:4
        for j = i+1:4
            if existen(i) && existen(j)
                fprintf('\nComparación entre recta %d y recta %d:\n', i, j);
                if mod(i + j, 2) == 0
                    grado_paralelismo = abs(calcularGradoParalelismo(listaRectas(i),listaRectas(j)));
                    fprintf('Grado de paralelismo: %.2f%% de grado de confianza\n', grado_paralelismo);
                else
                    angulo_grados= abs(calcularAnguloEntreRectas(listaRectas(i),listaRectas(j)));
                    fprintf('Ángulo entre las rectas: %.2f grados (%.2f%% de grado de confianza respecto a 90º )\n', angulo_grados, abs(100 - abs(angulo_grados - 90)));

                end
            end
        end
    end
end