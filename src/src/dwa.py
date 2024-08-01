#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image,PointCloud2
import math
import cv2
from cv_bridge import CvBridge,CvBridgeError


class Robot:
    def __init__(self):
        rospy.init_node("dwa_robot", anonymous=True)

        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.w = 0
        self.max_v = 0.5
        self.max_w = np.pi/2
        self.max_acc_v = 0.5
        self.max_acc_w = np.pi/8
        self.kp_lin = 0.4
        self.kp_ang = 0.6
        self.px = 0
        self.py = 0
        self.dt = 0.25


        self. altura_minima_do_obstaculo = 0.2
        self.limite_linha = 150
        self.y_limit = 150
        #self.is_obstacle_detected = False

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        #self.pointcloud_sub  = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)

        self.get_obstacle_cost = []
        self.obstacles = []
        self.obstacle_matrix = []
        self.obstacle_distance = np.inf
        self.safety_distance = 0.5
        self.profundidade = None
        self.rate = rospy.Rate (10)

        self.bridge = CvBridge()


    def odom_callback(self,msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        orientation_list = [orientation.x,orientation.y,orientation.z,orientation.w]
        _,_, self.theta = euler_from_quaternion(orientation_list)
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z
    
    def depth_callback (self,msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.profundidade = depth_image
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        self.print_depth_values(depth_image)

        #status, media_profundidade = self.process_image_depth(depth_image)
        #rospy.loginfo(f"{object_detect} com profundidade média de {media_profundidade.2f}")

        self.obstacles = self.detect_obstacles_from_depth_image(depth_image)
        self.update_obstacle_matrix(self.obstacles)  
        #if self.is_object_detected_nearby(depth_image) == True:
        #    rospy.loginfo("Objeto detectado")
        return depth_image

    def process_image_depth (self, depth_image):
        depth_array = np.array(depth_image)

        #aplicando o limite da linha para ignorar o chão
        depth_array[:self.y_limit,:] = np.nan

        media_profundidade = np.nanmean(depth_array)/1000.0

        if media_profundidade < self.altura_minima_do_obstaculo:
            return "Objeto detectado", media_profundidade
        else:
            return "Nenhum objeto detectado", media_profundidade
        
    def print_depth_values(self, depth_image):
        h, w = depth_image.shape
        cx, cy = w//2,h//2

        center_value = depth_image[cx, cy]

        min_value = np.min(depth_image[cy-10:cy+10, cx-10:cx+10 ])
        max_value = np.max(depth_image[cy-10:cy+10, cx-10:cx+10 ])
        #rospy.loginfo(f"Center value: {center_value:.2f}, Min value: {min_value:.2f}, Max value: {max_value:.2f}")

    def is_object_detected_nearby(self):
        #object_detect, media_profundidade = self.process_image_depth(depth_image) 
        #if not self.obstacles:
        if self.profundidade is None:
            return False
        
        detection_distance = 1.0  # Distância de detecção em metros
        threshold = 0.8 # Tolerância para a detecção
        depth_distance = 1.0

        h, w = self.profundidade.shape
        
        #if object_detect == True:
        for i in range(0,self.y_limit, 20):  # Reduz a resolução para processamento mais rápido
            for j in range(0, w, 40):
                d = self.profundidade[i, j] / 1000.0
                if detection_distance - threshold < d < detection_distance + threshold:
                    return True  # Objeto detectado a aproximadamente 1 metro
                #else:
        return False
    
    def detect_obstacles_from_depth_image(self, depth_image):
        obstacles = []
        h, w = depth_image.shape
        cx, cy = w // 2, h // 2  # Coordenadas do centro da imagem
        fov = 60.0  # Campo de visão da câmera em graus
        fov_rad = np.deg2rad(fov)
        fx = w / (2 * np.tan(fov_rad / 2))  # Fator de focalização horizontal

        for i in range(0,self.y_limit,20):  
            for j in range(0, w, 20):
                d = depth_image[i, j] / 1000.0  # Converter de milímetros para metros
                if 0.1 < d < self.safety_distance:  
                    angle = np.arctan2(j - cx, fx)
                    x = (self.x + d * np.cos(self.theta + angle)) 
                    y = (self.y + d * np.sin(self.theta + angle)) 
                    obstacles.append((x, y, 200.0))  # Ajuste o raio conforme necessário

                    # Adicionar impressão de coordenadas para debug
                    #rospy.loginfo(f'Detected obstacle at (x: {x}, y: {y}, distance: {d} meters)')
        return obstacles
    
    #def pointcloud_callback(self,msg):
        #processar nuvem de pontos
        #pass
    def update_obstacle_matrix(self,obstacles):
        self.obstacle_matrix = []
        for x,y, size in obstacles:
            self.obstacle_matrix.append([x,y,size])
        #rospy.loginfo(f'Obstacle matrix update: {self.obstacle_matrix}')

    def update_state(self, dt):
        # Atualiza o estado do robô com base nas velocidades atuais
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += self.w * dt

    def dynamic_window(self, dt):
        # Calcula a janela dinâmica
        min_v = max(self.max_v, self.v - self.max_acc_v * dt)
        max_v = min(self.max_v, self.v + self.max_acc_v * dt)
        min_w = max(-self.max_w, self.w - self.max_acc_w * dt)
        max_w = min(self.max_w, self.w + self.max_acc_w * dt)
        
        if self.profundidade is not None:
            h,w = self.profundidade.shape
            for i in range (0,self.y_limit,20):
                for j in range (0,w,40):
                    d = self.profundidade[i,j]/1000.0
                    if 0.1 < d < self.safety_distance:
                        max_v = min(max_v, d/2.0)
                        max_w = self.max_w
                        break
                 
        #rospy.loginfo(f"Dynamic Window: min_v={min_v}, max_v ={max_v}, min_w = {min_w}, max_w = {max_w}")
        return min_v, max_v, min_w, max_w

    def evaluate_trajectory(self, v, w, dt, goal):
        # Simula uma trajetória e avalia seu custo
        x, y, theta = self.x, self.y, self.theta
        total_cost = 0
        for _ in range(int(dt / 0.1)):
            x += v * np.cos(theta) * 0.1
            y += v * np.sin(theta) * 0.1
            theta += w * 0.1

        #cost_to_goal = np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)
        #cost_to_obstacle = self.get_obstacle_cost(x, y)

        #total_cost += cost_to_goal + 10 * cost_to_obstacle + 0.1 * abs(v)  # Aumentar o peso do custo de obstáculo
            '''
        for ox,oy,radius in self.obstacles:
            distance_to_obstacle = np.linalg.norm([ox - x, oy - y])
            if np.linalg.norm([ox - x,oy - y]) < radius + 1.0:
                #rospy.loginfo(f"Trajectory too close to ostacle at ({ox},{oy})")
                total_cost += float('inf')
            '''
        for ox,oy,radius in self.obstacle_matrix:
            distance_to_obstacle = np.linalg.norm([ox - x, oy - y])
            if distance_to_obstacle < radius + 1.0:
                total_cost += float('inf')
            
        distance_to_goal = np.linalg.norm([goal[0] - x, goal[1] - y])
        total_cost += distance_to_goal

        return total_cost

    def replan_velocity(self, dt, goal):
        min_v, max_v, min_w, max_w = self.dynamic_window(dt)
        best_v, best_w = 0, 0
        min_cost = float('inf')

        for v in np.linspace(min_v, max_v, num=30):
            for w in np.linspace(min_w, max_w, num=30):
                cost = self.evaluate_trajectory(v, w, dt, goal)
                #rospy.loginfo(f"Evaluating trajectory: v={v}, w={w}, cost={cost}")
                if cost < min_cost:
                    min_cost = cost
                    best_v, best_w = v, w
        if self.is_object_detected_nearby():
            best_v = min(min_v,max_v)
            best_w = max_w

        #rospy.loginfo(f"Best trajectory: best_v={best_v}, best_w={best_w}, min_cost={min_cost}")
        return best_v, best_w

    
    def move_robot(self,goal):
        dt = self.dt
        while not rospy.is_shutdown():
            distance_to_goal = np.linalg.norm([goal[0] - self.x, goal[1] - self.y])
            if distance_to_goal < 0.1: #Tolerancia para atingir o objetivo máximo
                rospy.loginfo ("Objetivo alcançado")
                return
            
            best_v,best_w = self.replan_velocity(dt,goal)

            
            if self.is_object_detected_nearby():
                #rospy.loginfo("Obstáculo detectado. Mudando trajetoria")
                twist_msg = Twist()
                twist_msg.linear.x = 0
                twist_msg.linear.z = self.max_w
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()

                best_v,best_w = self.replan_velocity(dt,goal)
                #rospy.loginfo(f"Best trajectory: best_v={best_v}, best_w={best_w}")
            
                # Verificar se uma nova trajetória é segura
                if best_v != 0 or best_w != 0:
                    #rospy.loginfo("Replanejamento de trajetória bem-sucedido.")
                    twist_msg.linear.x = best_v
                    twist_msg.angular.z = best_w
                    self.cmd_vel_pub.publish(twist_msg)
                    self.update_state(dt)
                continue


            u_x = goal[0] - self.x
            u_y = goal [1] - self.y

            angle_to_goal = math.atan2(u_y,u_x)
            angle_diff = angle_to_goal - self.theta

            if angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            elif angle_diff < -np.pi:
                angle_diff += 2 * np.pi 
            
            self.phi_desired = math.atan2(u_y,u_x)
            distance_error = abs(math.sqrt(u_x**2 + u_y**2))
            angular_error = math.atan2(math.sin(self.phi_desired - self.theta), math.cos(self.phi_desired - self.theta))

            linear_vel_prop= self.kp_lin * distance_error #* best_v
            angular_vel_prop= self.kp_ang * angular_error #* best_w

            linear_vel = min(linear_vel_prop, best_v) if best_v > 0 else linear_vel_prop
            angular_vel = best_w if abs(angular_vel_prop) < abs(best_w) else angular_vel_prop

            corrected_linear_vel = min(linear_vel_prop, best_v)

            corrected_angular_vel = min(angular_vel_prop, best_w)

            #linear_vel = min(linear_vel_prop, best_v) if best_v > 0 else linear_vel_prop
            #angular_vel = best_w if abs(angular_vel_prop) < abs(best_w) else angular_vel_prop

            #linear_final = (linear_vel +best_v)/2
            #angular_final = (angular_vel + best_w)/2

            #rospy.loginfo(f"Goal: ({goal[0]}, {goal[1]})")
            rospy.loginfo(f"Current Position: ({self.x}, {self.y})")
            #rospy.loginfo(f"Distance to Goal: {distance_to_goal}")
            #rospy.loginfo(f"Angle to Goal: {angle_to_goal}")
            #rospy.loginfo(f"Angle Difference: {angle_diff}")
            #rospy.loginfo(f"Best V: {best_v}, Best W: {best_w}")
            #rospy.loginfo(f"Linear Velocity: {linear_vel}, Angular Velocity: {angular_vel}")

            twist_msg = Twist()  
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            #rospy.loginfo(f'linear velocity={linear_vel}')            
            #rospy.loginfo(f'angular velocity={angular_vel}')
            #distance_error = abs(math.sqrt(((goal[1] - self.y)**2) + (goal[0] - self.x)**2))
            #angular_error = math.atan2
            self.cmd_vel_pub.publish(twist_msg)
            self.update_state(dt)
            self.rate.sleep()

    def user_interface (self):
        while not rospy.is_shutdown():
            goal_x = float(input("Digite a coordenada x do objetivo: "))
            goal_y = float(input("Digite a coordenada y do objetivo: "))
            goal = [goal_x,goal_y]
            self.move_robot(goal)
            rospy.loginfo(f'coordenada x={self.x}')            
            rospy.loginfo(f'coordenada y={self.y}')
            cont = input ("Deseja enviar outro comando? (s,n): ")
            if cont.lower() != 's':
                break

if __name__ == "__main__":
    try:
        robot = Robot()
        robot.user_interface()
    except rospy.ROSInterruptException:
        pass
