% CONEXION A ROS
setenv('ROS_MASTER_URI','http://192.168.1.166:11311') % IP de la MV
setenv('ROS_IP','192.168.1.132') % IP de nuestro ordenador

rosshutdown;
rosinit;

pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');