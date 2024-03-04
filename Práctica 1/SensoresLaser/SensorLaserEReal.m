%setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
%setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

rosshutdown;
setenv('ROS_MASTER_URI','http://172.29.30.178:11311') % IP de la MV
setenv('ROS_IP','172.29.29.50') % IP de nuestro ordenador
rosinit;

Recta = struct('pendiente', 0, 'interseccion', 0);
Punto = struct('ejeX', 0,'ejeY',0);


laser = rossubscriber('/scan');
mensajeLaser = receive(laser, 10);
listaDistanciasLaser = mensajeLaser.Ranges;
distanciaFrontal = listaDistanciasLaser(100);


PuntoFrontalIzquierdo=calcularPunto(listaDistanciasLaser(200),posicionAngulo(200));
PuntoFrontalDerecho=calcularPunto(listaDistanciasLaser(160),posicionAngulo(200));

PuntoIzquierdoIzquierdo=calcularPunto(listaDistanciasLaser(290),posicionAngulo(290));
PuntoIzquierdoDerecho=calcularPunto(listaDistanciasLaser(250),posicionAngulo(250));

PuntoTraseroIzquierdo=calcularPunto(listaDistanciasLaser(20),posicionAngulo(20));
PuntoTraseroDerecho=calcularPunto(listaDistanciasLaser(340),posicionAngulo(340)); 

PuntoDerechoIzquierdo=calcularPunto(listaDistanciasLaser(110),posicionAngulo(110));
PuntoDerechoDerecho=calcularPunto(listaDistanciasLaser(70),posicionAngulo(70));

PuntoFrontalIzquierdo
PuntoFrontalDerecho
PuntoIzquierdoDerecho
PuntoIzquierdoIzquierdo
PuntoDerechoDerecho
PuntoDerechoIzquierdo
PuntoTraseroDerecho
PuntoTraseroIzquierdo

rectaFrontal= calcularRecta(PuntoFrontalIzquierdo,PuntoFrontalDerecho);
rectaTrasera=calcularRecta(PuntoTraseroIzquierdo,PuntoTraseroDerecho);
rectaIzquierda= calcularRecta(PuntoIzquierdoIzquierdo,PuntoIzquierdoDerecho);
rectaDerecha= calcularRecta(PuntoDerechoIzquierdo,PuntoDerechoDerecho);

analizarRectas(rectaFrontal, rectaIzquierda, rectaTrasera, rectaDerecha);

rosshutdown;

function punto = calcularPunto(distancia, angulo)

   if distancia == inf || distancia>1
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

function gradoParalelismo = calcularGradoParalelismo(recta1, recta2)

    diferenciaPendientes = abs(recta1.pendiente - recta2.pendiente);
    gradoParalelismo = 100 - (diferenciaPendientes / (1 + abs(recta1.pendiente * recta2.pendiente))) * 100;
end

function anguloGrados = calcularAnguloEntreRectas(recta1, recta2)
    anguloRadianes = atan((recta2.pendiente - recta1.pendiente) / (1 + recta1.pendiente * recta2.pendiente));
    anguloGrados = abs(rad2deg(anguloRadianes));
end
function grados = posicionAngulo(posicion)
    grados= mod(posicion+180,360);
end

function analizarRectas(rectaFrontal, rectaIzquierda, rectaTrasera, rectaDerecha)
    listaRectas=[rectaFrontal,rectaIzquierda,rectaTrasera,rectaDerecha];
    listaNombres=["Recta frontal","Recta izquierda","Recta trasera","Recta derecha"];
    listaPorcentajes=[];
    
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
    
    % Calcular el grado de paralelismo y el ángulo entre las rectas existentes
    for i = 1:4
        for j = i+1:4
            if existen(i) && existen(j)
                fprintf('\nComparación entre la %s y la %s:\n', listaNombres(i), listaNombres(j));
                if mod(i + j, 2) == 0
                    grado_paralelismo = abs(calcularGradoParalelismo(listaRectas(i),listaRectas(j)));
                    listaPorcentajes=[listaPorcentajes,grado_paralelismo];
                    fprintf('Grado de paralelismo: %.2f%% de grado de confianza\n', grado_paralelismo);
                else
                    angulo_grados= abs(calcularAnguloEntreRectas(listaRectas(i),listaRectas(j)));
                    listaPorcentajes=[listaPorcentajes,(angulo_grados/90)*100];
                    fprintf('Ángulo entre las rectas: %.2f grados (%.2f%% de grado de confianza respecto a 90º )\n', angulo_grados, 100*abs(angulo_grados/90));

                end
            end
        end
    end
    fprintf('\nLas rectas presentes son:\n');
    fprintf('Recta Frontal: %d\n', existen(1));
    fprintf('Recta Izquierda: %d\n', existen(2));
    fprintf('Recta Trasera: %d\n', existen(3));
    fprintf('Recta Derecha: %d\n', existen(4));

    gradoDeConfianza=sum(listaPorcentajes)/length(listaPorcentajes);
    fprintf('\nCon un grado de confianza global de %.2f%% \n',gradoDeConfianza);
end