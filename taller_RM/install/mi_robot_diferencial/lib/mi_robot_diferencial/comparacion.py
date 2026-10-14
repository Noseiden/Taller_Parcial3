#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray

class Comparacion(Node):

    def __init__(self):

        super().__init__('comparacion')
        # Parámetros físicos del robot
        self.R = 0.05 #Diametro de las ruedas en metros
        self.L = 0.23/2 #distancia de las ruedas al centro del robot en metros

        # Pose inicial estimada
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0

        #Velocidades angulares de las ruedas
        self.wD = 0.0
        self.wI = 0.0
        # posición y orientación del robot obtenida desde Gazebo
        self.odom_gazebo = None

        #Para integrar delta t:
        self.t_anterior = None 
        self.delta_t = 0.0
        
        #Me suscribo a los tópicos de /joint_states y /odom
        self.create_subscription(JointState, '/joint_states', self.cb_ruedas, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)

        # Topico donde voy a publicar mis resultados:
        self.pub_estimado = self.create_publisher(Vector3, '/pose_estimada', 10)
        self.pub_gazebo = self.create_publisher(Vector3, '/pose_gazebo', 10)
        self.pub_error = self.create_publisher(Vector3, '/error_odometria', 10)
        self.t_ultima_publicacion = None

    ### Funciones que leen las variables que necesito: wD, WI, posición y orientación del robot
    def cb_ruedas(self, msg): #Obtengo las velocidades angulares de las ruedas derecha e izquierda
        self.wD = msg.velocity[msg.name.index('joint_rueda_der1')]
        self.wI = msg.velocity[msg.name.index('joint_rueda_izq1')]

        # Datos para integrar:
        t_actual = self.get_clock().now().nanoseconds / 1e9
        if self.t_anterior is None:
            self.t_anterior = t_actual
            return
        self.delta_t = t_actual - self.t_anterior
        self.t_anterior = t_actual
        if self.delta_t <= 0:
            return

        self.comparar()

    def cb_odom(self, msg): #Obtengo la posición y orientación del robot desde el tópico /odom
        self.odom_gazebo = msg.pose.pose

    ## Funcion para calcular la posición y orientación del robot a partir de las velocidades angulares de las ruedas
    def calcular_velocidades(self):
        w = np.array([self.wD, self.wI]) # Vector de velocidades angulares de las ruedas
        W = np.array([[self.R/2,            self.R/2],
                      [       0,                   0],
                      [self.R/(2*self.L), -self.R/(2*self.L)] ])

        zita = W @ w

        J = np.array([
            [math.cos(self.phi), -math.sin(self.phi), 0],
            [math.sin(self.phi),  math.cos(self.phi), 0],
            [0,                   0,                  1]])

        qpunto = J @ zita

        self.x += qpunto[0] * self.delta_t
        self.y += qpunto[1] * self.delta_t
        self.phi += qpunto[2] * self.delta_t

        q = np.array([self.x, self.y, self.phi])
                    #posicion en x, posicion en y, orientacion

        return q

    def comparar(self):

        #Posicion y orientacion a partir de la matriz W
        Valores_estimados = self.calcular_velocidades()
        if self.odom_gazebo is None:
            return
        
        #Posicion y orientacion obtenida desde odom
        x = self.odom_gazebo.position.x
        y = self.odom_gazebo.position.y
        q= self.odom_gazebo.orientation
        theta = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y**2 + q.z**2)
)
        Valores_gazebo = np.array([x, y, theta])

        #Error
        # Cálculo de errores de posición [m]
        e_x = abs(Valores_gazebo[0] - Valores_estimados[0])
        e_y = abs(Valores_gazebo[1] - Valores_estimados[1])

        # Cálculo del error angular [grados]
        diferencia_phi = Valores_gazebo[2] - Valores_estimados[2]

        e_theta = math.degrees(abs(math.atan2(math.sin(diferencia_phi),math.cos(diferencia_phi))))

        # Publicación de los resultados
        t_actual = self.get_clock().now().nanoseconds / 1e9
        if (self.t_ultima_publicacion is not None and
                t_actual - self.t_ultima_publicacion < 1.0):
            return
        self.t_ultima_publicacion = t_actual


        estimado = Vector3()
        estimado.x = float(Valores_estimados[0])
        estimado.y = float(Valores_estimados[1])
        estimado.z = math.degrees(Valores_estimados[2])
        self.pub_estimado.publish(estimado)

        gazebo = Vector3()
        gazebo.x = float(Valores_gazebo[0])
        gazebo.y = float(Valores_gazebo[1])
        gazebo.z = math.degrees(Valores_gazebo[2])
        self.pub_gazebo.publish(gazebo)

        # Publicar errores
        error = Vector3()
        error.x = float(e_x)
        error.y = float(e_y)
        error.z = float(e_theta)
        self.pub_error.publish(error)
                
def main(args=None):
    rclpy.init(args=args)

    nodo = Comparacion()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()