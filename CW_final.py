import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2
from clover.srv import SetLEDEffect
import time                            #Импорт библиотек

bridge = CvBridge()

rospy.init_node('flight') #Иницилизация полёта

img = [] #Создание массива для снимка

def id_color(img): #Функция определения цвета
    col = 4 #инициализация переменной цвета
    R = 0 #инициализация переменных, хранящих число пикселей
    Y = 0
    B = 0
    G = 0
    for i in range(len(img)): #Подсчёт пикселей определённого цвета
        for j in range(len(img[0])):
            if (img[i, j][2] >= 150) and (img[i, j][1] <= 115) and (img[i, j][0] <= 115):
                R = R + 1   
            if (img[i, j][2] <= 70) and (img[i, j][1] >= 90) and (img[i, j][0] <= 70):
                G = G + 1
            if (img[i, j][2] >= 200) and (img[i, j][1] >= 200) and (img[i, j][0] <= 140):
                Y = Y + 1
            if (img[i, j][2] <= 75) and (img[i, j][1] <= 140) and (img[i, j][0] >= 140):
                B = B + 1
    h = [R, G, B, Y] #Поиск цвета с максимальным кол-вом пикселей
    if R == max(h):
        col = 3
    if G == max(h):
        col = 1
    if B == max(h):
        col = 2
    if Y == max(h):
        col = 0
    if max(h) < 20:
        col = 4
    return col         
#Вывод и кадрирование снимка
def image_callback(data): 
    global img
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    img = img[70:170, 110:210]
#Функция подлёта к точке
def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
#Функция включения светодиодной ленты
def lad(god):
    if god == 0:
        set_effect(r=255, g=255, b=0)
    if god == 1:
        set_effect(r=0, g=255, b=0)
    if god == 2:
        set_effect(r=0, g=0, b=255)
    if god == 3:
        set_effect(r=255, g=0, b=0)
    

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)#Получение телеметрии 
navigate = rospy.ServiceProxy('navigate', srv.Navigate) 
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

navigate_wait(z=2, frame_id='body', auto_arm=True) #Взлёт


A = [[0.45, 5.4], [1.35, 5.4], [2.25, 5.4], [3.15, 5.4], [3.6, 4.95], [3.15, 4.95], [2.7, 4.95], [2.25, 4.95],
    [1.8, 4.95], [1.35, 4.95], [0.9, 4.95], [0.45, 4.95], [0, 4.95], [0.45, 4.5], [1.35, 4.5], [2.25, 4.5], [3.15, 4.5]] #Список точек склада
p = id_color(img) #Инициализация функции распознавания цветных маркеров
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1) #Получение снимка

B = [0, 0, 0, 0, 0] #Массив грузов
for i in range(len(A)): #Полёты по точкам склада
    navigate_wait(x=A[i][0], y=A[i][1], z=0.75, frame_id='aruco_map')#Полёт на точку склада
    rospy.sleep(3)#Задержка для стабилизации
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)#Получение снимка
    d = id_color(img)#Сохранение цвета метки
    B[d] = B[d] + 1

print('Balance', B[0]+B[1]+B[2]+B[3], 'cargo')#Вывод типов грузов и их количества
print('Type 0:', B[0], 'cargo')
print('Type 1:', B[1], 'cargo')
print('Type 2:', B[2], 'cargo')
print('Type 3:', B[3], 'cargo') 


for i in range(5): #Полёт по точкам поля в поисках дронпоинта
    for j in range(5):
        navigate_wait(x=i*0.9, y=j*0.9, z=1.75, frame_id='aruco_map')#Полёт на точку поля
        rospy.sleep(2)#Задержка для стабилизации
        image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)#Получение снимка
        o = id_color(img)#Распознавание цвета цифры
        if o == 1: #Если в кадре что-то зелёное, то это дронпоинт
            lad(1) #Включение зелёной индикации (распознование цифры не получилось)
            rospy.sleep(5.0)#Задержка для стабилизации
            land()#Посадка
            print('D1_delivered to',B[1],'cargo')#Вывод сообщения о доставке
            rospy.sleep(2.0)#Задержка на дронпоинте
            set_effect(r=0, g=0, b=0)#Отключение подсветки
            navigate_wait(z=1, frame_id='body', auto_arm=True)#Взлёт
        o = 0

navigate_wait(x=0, y=0, z=2, frame_id='aruco_map')#Полёт на исходную точку и посадка
navigate_wait(x=0, y=0, z=0, frame_id='aruco_map')
land()
