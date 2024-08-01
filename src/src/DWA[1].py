#!/usr/bin/env python3

import rospy
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
import tkinter as tk
#===========================

show_animation = False  # Flag que indica se a animação deverá ser exibida


class RobotProp:
    """Parãmetros"""
    def __init__(self):
        #rospy.init_node('Robot_Prop', anonymous=True)
        self.maxV = 0.6
        self.maxW = 2.5
        #Vel linear e angular máximas
        self.accV = 1.5
        self.accW = 1.5
        #Acel linear e angular máximas
        self.maxV = 0.4
        self.maxW = 1.5 
        self.accV = 1.5
        self.accW = 1.5

        #Resolução para discretizar as vel
        self.resolV = 0.1
        self.resolW = 0.01
        #intervalo de tempo (dt)
        self.dt     = 0.1
        #Tempo total para a simulação 
        self.T      = 3.0
        #Constantes usadas em cálculos
        self.kalpha = 0.6
        self.kro = 3.0
        self.kv  = 1.0
        #Pesos usados nos cálculos
        self.weightV = 6.0
        self.weightW = 6.0
        self.weightObs = 7.0

        self.current_position = 0

#=========Cálculo da janela Dinâmica=========#
def calc_dynamic_window(x, robot_prop):
    #x = Lista do estado do robô 
    #robot_prop = Prâmetros do robô 
   
    # Vel.lin.(min e max), Vel. ang. (min e max) das especificações do robô  
    vs = [-robot_prop.maxV, robot_prop.maxV, -robot_prop.maxW, robot_prop.maxW]

    # Vel.lin.(min e max), Vel. ang. (min e max) do estado atual do robô 
    vd = [x[3] - robot_prop.accV * robot_prop.dt,
          x[3] + robot_prop.accV * robot_prop.dt,
          x[4] - robot_prop.accW * robot_prop.dt,
          x[4] + robot_prop.accW * robot_prop.dt]

    # Dynamic Window
    vr = [max(vs[0], vd[0]), min(vs[1], vd[1]),
          max(vs[2], vd[2]), min(vs[3], vd[3])]
          

    return vr #Retorna a janela dinâmica em formato de lista 

#==========Movimento==========#
def motion(x, u, robot_prop):
    """
    Motion Model 
    :param x: Lista do estado do robô 
    :param u: Lista que apresenta informaçãoes de controle atuais 
    :param robot_prop: Prâmetros do robô
    :return: a lista x atualizada 
    """
    dt = robot_prop.dt
    x[0] += u[0] * math.cos(x[2]) * dt  # Atualiza a posição x com base na velocidade linear e a orientação atual
    x[1] += u[0] * math.sin(x[2]) * dt  # Atualiza a posição y com base na velocidade linear e a orientação atual
    x[2] += u[1] * dt  # Atualiza a orientação 
    x[3] = u[0]  # Atualiza a velocidade linear
    x[4] = u[1]  # Atualiza a velocidade angular 

    return x

#===========Gera uma trajetória===========#
def calc_traj(x, v, w, robot_prop):
    """
    轨迹生成
    :param: x，v，w，robot_prop
    :return: x(estado do robô após o período de tempo T) , traj(matriz trajetória)
    """
    x = np.array(x)
    time = 0
    traj = np.array(x)
    while time <= robot_prop.T:
        time += robot_prop.dt
        x = motion(x, [v, w], robot_prop)
        traj = np.vstack((traj, x))

    return x, traj

#============Orientação do robô em relação ao objetivo============#
def calc_heading(x, goal):
    """
    航向参数得分  当前车的航向和相对于目标点的航向 偏离程度越小 分数越高
    :param x: lista que representa o estado atual do robô 
    :param goal: lista que contém as coordenadas do objetivo 
    :return: heading (ângulo), dist2goal(distância até o objetivo)
    """
    #Quanto menor a distância, maior a pontuação, portanto, indica que o robô está mais próximo do objetivo
    goal_theta = math.atan2(goal[0] - x[0], goal[1] - x[1])
    #Diferença de ângulo entre a orientação do robô e do objetivo 
    heading = abs(x[2] - goal_theta)
    #DistÂncia atual até a distãncia do objetivo 
    dist2goal = abs(math.sqrt((goal[0] - x[0]) ** 2 + (goal[1] - x[1]) ** 2))
    return heading, dist2goal

#===========Calcula a pontuação para velocidades linear e angular===========#
def calc_score_v_w(alpha, ro, vt, wt, robot_prop):
    """

    :param alpha: Diferença de ângulo entre a orientação do robô e do objetivo
    :param ro: O quanto o robô se desviou do curso 
    :param vt: Velocidade linear desejada pelo robô 
    :param wt: Velocidade angular desejada pelo robô
    :param robot_prop: Prâmetros do robô
    :return: Pontuações de vel linear e angular 
    """
    # 计算线速度得分
    vi = robot_prop.kv * robot_prop.maxV * math.cos(alpha) * math.tanh(ro / robot_prop.kro)
    # vel desejada - vel linear (Quanto menor a diferença, maior a pontuação) 
    score_v = 1 - abs(vt - vi) / (2 * robot_prop.maxV)

    # 计算角速度得分
    wi = robot_prop.kalpha * alpha + vi * math.sin(alpha) / ro
    #vel desejada - vel angular (Quanto menor a diferença, maior a pontuação)
    score_w = 1 - abs(wt - wi) / (2 * robot_prop.maxW)
    '''A pontuação rsultante dirá o quão bem o robõ está seguindo o caminho'''
    return score_v, score_w

#=========Calcula a pontuação entre a distância do robô até um objeto=========#
def calc_score_dis2obs(traj, obs, obs_r):
    """
    障碍物距离评价函数  （机器人在当前轨迹上与最近的障碍物之间的距离，如果没有障碍物则设定一个常数）
    :param traj: Matriz das coordenadas do trajeto calculado 
    :param obs: Matriz que contém as coordenadas do obstáculos 
    :param obs_r: Zona de influência do obstáculo 
    :return: distância do robô até o obstáculo  
    """
    dis2obs = float("inf")

    # 提取轨迹上的机器人x y坐标
    robotx = traj[:, 0:2]

    for it in range(0, len(robotx[:, 1])):
        for io in range(0, len(obs[:, 0])):
            dx = obs[io, 0] - robotx[it, 0]
            dy = obs[io, 1] - robotx[it, 1]
            disttemp = math.sqrt(dx ** 2 + dy ** 2) - obs_r

            if disttemp < dis2obs:
                dis2obs = disttemp

    # Se não houver obstáculo ao longo do percurso
    if dis2obs >= 1.5 * obs_r:
        dis2obs = 1.5 * obs_r

    return dis2obs

#==========Calcula a distância de parada==========# 
def calc_breaking_dist(vt, robot_prop):
    """

    :param vt: velocidade que se pretende avaliar 
    :param robot_prop: Parâmetros do robô 
    :return: distância de parada
    """
    stopdist = vt ** 2 / (2 * robot_prop.accV)
    return stopdist

#==========Avaliar as potneciais trajetórias==========#
def evaluation(x, vr, goal, obs, obs_r, robot_prop):
    """
    评价函数 内部负责产生可用轨迹
    :param x: Vetor posição
    :param vr: Janela Dinâmica
    :param goal: Posição do objetivo
    :param obs: Matriz das coordenadas dos obstáculos
    :param obs_r: Raio dos obstáculos
    :param robot_prop: Parâmetros dos robôs 
    :return: Pontuaçãoe se trajetórias dos robôs 
    """
    # robot_score = np.array([0, 0, 0, 0, 0])
    robot_score = []
    robot_trajectory = []
    for vt in np.arange(vr[0], vr[1], robot_prop.resolV):
        for wt in np.arange(vr[2], vr[3], robot_prop.resolW):

            # Calcula a trajetória do robô 
            xt, traj = calc_traj(x, vt, wt, robot_prop)

            # Ângulo entre o robô e o objetivo 
            alpha, ro = calc_heading(xt, goal)

            # Calcula as pontuações para velocidades linear e angular
            score_v, score_w = calc_score_v_w(alpha, ro, vt, wt, robot_prop)

            # Calcula pontuações de distância para os obstáculos
            score_dis2obs = calc_score_dis2obs(traj, obs, obs_r)

            # Calcula a distãncia de frenagem
            stopdist = calc_breaking_dist(abs(vt), robot_prop)

            if score_dis2obs > stopdist:
                robot_score.append([vt, wt, score_v, score_w, score_dis2obs])
                # robot_score = np.vstack((robot_score, [vt, wt, score_v, score_w, score_dis2obs]))
                robot_trajectory.append(np.transpose(traj))

    robot_score = np.array(robot_score)
    return robot_score, robot_trajectory

#===========Normalizar as pontuações das trajetórias===========#
def normalization(score):
    """
    归一化处理
    :param score: Pontuação total 
    :return: Nova pontuação
    """
    #Pontuação da vel linear
    if sum(score[:, 2]) != 0:
        score[:, 2] = score[:, 2] / sum(score[:, 2])
    #Pontuação da vel angular
    if sum(score[:, 3]) != 0:
        score[:, 3] = score[:, 3] / sum(score[:, 3])
    #Pontuação da distãncia entre os obstáculos
    if sum(score[:, 4]) != 0:
        score[:, 4] = score[:, 4] / sum(score[:, 4])

    return score

#============Implementar o algoritmo DWA============#
def dwa_control(x, goal, obs, obs_r, robot_prop):
    """
    DWA算法实现
    :param x: Estado atual do robô 
    :param goal: Coordenadas do objetivo 
    :param obs: coordenads do obstáculo
    :param obs_r: raio do obstáculo  
    :param robot_prop: Parãmetros do robô
    :return: u (velocidaes) e traj (trajetória)
    """
    score_list = []
    # Dynamic Window: Vr = [vmin, vmax, wmin, wmax] 最小速度 最大速度 最小角速度 最大角速度速度
    # Calculando o intervalo dinâmico 
    vr = calc_dynamic_window(x, robot_prop)
    # Lista de trajetórias possíveis, calculando pontuações para cada trajetória
    robot_score, robot_trajectory = evaluation(x, vr, goal, obs, obs_r, robot_prop)
    #Normalizar as pontuações e realiza a soma ponderada das pontuações 
    if len(robot_score) == 0:
        print('no path to goal')
        u = np.transpose([0, 0])
    else:
        score = normalization(robot_score)
	#Seleciona a trajetória com maior pontuação e a melhor ação 
        for ii in range(0, len(score[:, 0])):
            weights = np.mat([robot_prop.weightV, robot_prop.weightW, robot_prop.weightObs]) #média dos pesos
            scoretemp = weights * (np.mat(score[ii, 2:5])).T
            score_list.append(scoretemp)

        max_score_id = np.argmax(score_list)
        u = score[max_score_id, 0:2]
        trajectory = robot_trajectory[int(max_score_id)]
        trajectory = np.array(trajectory)
        trajectory = np.transpose(trajectory)

    return u, trajectory

def publish_velocity(pub, x):
    twist = Twist()
    twist.linear.x = x[3]
    twist.angular.z = x[4]
    pub.publish(twist)

def odom_callback(self, msg, robot_prop):
    # Extracting position information from Odometry message
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y

    #orientation_q = msg.pose.pose.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
    # Update current position
    robot_prop.current_position = np.array([pos_x, pos_y])

def main():
    robot_prop = RobotProp()
#========Inicialização de variáveis========#
    #odom_sub = rospy.Subscscriber('/odom', Odometry , odom_callback)
    #odom_callback(self,msg,robot_prop)
    #position_x  = robot_prop.current_position[0]
    #position_y = robot_prop.current_position[1]
    x = np.array([0.0,0.0, math.pi*2, 0.0, 0.0])
    #Coordenadas do robô, na posição (0,0 . 0,0) ,orientação de 90 g e vel =0
    goal_x = float(input('Enter the target x coordinate: '))
    goal_y = float(input('Enter the target y coordinate: '))
    goal = np.array([goal_x, goal_y])
    #Ponto de chegada definido como o ponto(05,10)
    #Parâmetros do robô
    #global mapx, mapy, framex, framey, obs
#========Definição do Ambiente========#
    #mapx = [-1, 11]
    #Dimensão x do mapa
    #mapy = [-1, 11]
    #Dimensão y do mapa
    #framex = 200
    #Resolução x do frame
    #framey = 200
    #Resolução y do frame
    sensor_reading = 0
    obs = np.mat([[5.0, 5.0],     #Leitura do sensor)
                  [4.0, 4.0]])
    #Matriz de coordenada dos obstáculos do ambiente
    obs_r = 0.5
    robot_r = 0.1
    sum_r = obs_r + robot_r
    #Somatório dos raios

    history_x = np.array(x)

    #Criação do gráfico 
    #global root
    #root = tk.Tk()
    #matplotlib.use('TkAgg')
    #root.title("DWA 测试")

    # 创建图形
    #f = plt.figure(1, figsize=(4, 4), dpi=100)
    #a = f.add_subplot(111)

    # 把绘制的图形显示到tkinter窗口上
    #canvas = FigureCanvasTkAgg(f, master=root)
    #canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # 把matplotlib绘制图形的导航工具栏显示到tkinter窗口上
    #toolbar = NavigationToolbar2Tk(canvas, root)
    #toolbar.update()
    #canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # 按钮
    #button = tk.Button(master=root, text='Quit', command=_quit)
    #button.pack(side=tk.BOTTOM)

    #frame = tk.Frame(root, width=framex, height=framey, bg='green')
    #frame.bind(sequence="<Button-1>", func=callback)
    #frame.pack(side=tk.BOTTOM)

    rospy.init_node('cmd_vel_override')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # Publicar a cada 10 Hz


#=========Loop inicial que itera 5000 vezes=========#
    for i in range(100): #5000
    # Calcula o próximo controle do robô 
        u, trajectory = dwa_control(x, goal, obs, sum_r, robot_prop)

        # Atualiza a poisção do robô com base na função de movimento
        x = motion(x, u, robot_prop)


        # Posição atual
        history_x = np.vstack((history_x, x))

        while not rospy.is_shutdown ():
            publish_velocity(pub, x)
            rospy.loginfo(f'x ={x[2]}')
            rospy.loginfo(f'y ={x[4]}')
        
            rate.sleep()
    #==========Atualizando a Iteração gráfica==========#

            # Verifica se o robô chegou ao objetivo 
        if math.sqrt((x[0] - goal[0]) ** 2 + (x[1] - goal[1]) ** 2) <= 0.5:
            print('Arrive Goal!!!')
            #break
            break
#========Fim da simulação========#
    print("Done")
    plt.cla()
    plt.close()

'''
def callback(event):
    global obs, mapx, mapy, framex, framey
    #Imprime as coordenadas x,y aonde o mouse foi clicado
    print(event.x, event.y)
    #Calcula a nova posição do obstáculo com base na posição do clique do mouse 
    new_obs_x = event.x / framex * (mapx[1] - mapx[0]) + mapx[0]
    new_obs_y = (framey - event.y) / framey * (mapy[1] - mapy[0]) + mapy[0]
    #Adiciona às novas coordenadas x e y à matriz
    obs = np.vstack((obs, np.array([new_obs_x, new_obs_y])))
'''

# 按钮单击事件处理函数
def _quit():
    # 结束事件主循环，并销毁应用程序窗口
    global root
    #Encerra o programa
    root.quit()
    #Encerra completamente a aplicação
    root.destroy()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        #tk.mainloop()
