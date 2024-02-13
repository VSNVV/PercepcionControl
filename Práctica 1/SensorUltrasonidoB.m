setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

rosinit;

odom = rossubscriber('robot0/odom');
sonar3 = rossubscriber('/robot0/sonar_3');

publisher = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

mensajeMovimiento = rosmessage(publisher);

mensajeMovimiento.Linear.X = 0.0;
mensajeMovimiento.Linear.Y= 0.0;
mensajeMovimiento.Linear.Z = 0.0;
mensajeMovimiento.Angular.X = 0.0;
mensajeMovimiento.Angular.Y = 0.0;
mensajeMovimiento.Angular.Z = 0.0;

mensajeSonar = receive(sonar3, 3);
distanciaActual = mensajeSonar.Range_;

while(distanciaActual ~= 2)

    mensajeSonar = receive(sonar3, 3);
    distanciaActual = mensajeSonar.Range_;

    if(distanciaActual > 2)
        mensajeMovimiento.Linear.X = 0.1;
    end

    if(distanciaActual < 2)
        mensajeMovimiento.Linear.X = -0.1;
    end

    if(distanciaActual == 2)
        mensajeMovimiento.Linear.X = 0.0;
    end
    
    send(publisher, mensajeMovimiento);
    
    disp(distanciaActual);
end

rosshutdown;