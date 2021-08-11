#coding:utf-8
import cv2
import numpy as np
import json
import math
#pose
num_of_cloud=216      #点云总数
cloud_one=215    #机器人1点云数，   从0开始计数
cloud_two=12    #机器人2

#两幅地图初始不对齐参数:第二幅地图相对第一幅地图
change_x=0
change_y=0
change_z=0
rotate_x=0              #角度制
rotate_y=0
rotate_z=0

x_origin=0           #均值
y_origin=0
z_origin=0
x_ro_origin=0
y_ro_origin=0
z_ro_origin=0
ro_origin=0

#读入点云姿态
pose_txt = open("poses_new_no_sem.txt","r") 
data = pose_txt.readlines() 
save_pose=data[:]

j=0
for i in save_pose:            #清空
    i=[]
    save_pose[j]=i
    j=j+1
#print(save_pose)

num=0
line_num=0
for i in data:
    
    if line_num%5==0:
        num=num+1
    else:
        data0,data1,data2,data3=str(i).split()   #type of data0 is 'str'
        #print(data0,data1,data2,data3)   
        save_pose[num-1].append(float(data0))
        save_pose[num-1].append(float(data1))
        save_pose[num-1].append(float(data2))
        save_pose[num-1].append(float(data3))
    line_num=line_num+1
num_of_cloud=num
print("num is:",num)

trans=np.array([[0,0,0,1],[0.00035558,0.99990535,0.01375345,0.26461072],[0.41828482, -0.12231229 ,0.90004306, -0.61531781], [0.0,0.0,0.0, 1.0]])

#残差和
residual_x=0       #平移量
residual_y=0
residual_z=0
residual_trans=0   #平移量的残差
residual_ro_x=0 #旋转量
residual_ro_y=0
residual_ro_z=0
residual_ro=0         #旋转量的残差

#均方根误差
RMSE_x=0
RMSE_y=0
RMSE_z=0
RMSE_trans=0  #平移量的RMSE
RMSE_ro_x=0
RMSE_ro_y=0
RMSE_ro_z=0
RMSE_rotate=0 #旋转量的RMSE

#计算 均值
first=0
for i in save_pose:
    if i==[]:
        break
    trans[0][0]=save_pose[first][0]
    trans[0][1]=save_pose[first][1]
    trans[0][2]=save_pose[first][2]
    trans[0][3]=save_pose[first][3]

    trans[1][0]=save_pose[first][4]
    trans[1][1]=save_pose[first][5]
    trans[1][2]=save_pose[first][6]
    trans[1][3]=save_pose[first][7]

    trans[2][0]=save_pose[first][8]
    trans[2][1]=save_pose[first][9]
    trans[2][2]=save_pose[first][10]
    trans[2][3]=save_pose[first][11]

    trans[3][0]=0
    trans[3][1]=0
    trans[3][2]=0
    trans[3][3]=1
    rotate=trans[0:3,0:3]                       #求旋转矩阵
    a, j = cv2.Rodrigues(rotate)          #求旋转向量

    if(first<cloud_one):         #点云地图1
        x_origin=x_origin+save_pose[first][3]    #计算 均值
        y_origin=y_origin+save_pose[first][7]
        z_origin=z_origin+save_pose[first][11]
        x_ro_origin=x_ro_origin+math.atan2(save_pose[first][9],save_pose[first][10])/math.pi*180          #角度
        y_ro_origin=y_ro_origin+math.atan2(-save_pose[first][8],((save_pose[first][8])**2+(save_pose[first][10])**2)**0.5)/math.pi*180
        z_ro_origin=z_ro_origin+math.atan2(save_pose[first][4],save_pose[first][0])/math.pi*180
        ro_origin=ro_origin+np.linalg.norm(a)/math.pi*180
    if(first>cloud_one or first==cloud_one):     #点云地图2
        x_origin=x_origin+save_pose[first][3]+change_x    #计算 均值
        y_origin=y_origin+save_pose[first][7]+change_y
        z_origin=z_origin+save_pose[first][11]+change_z
        x_ro_origin=x_ro_origin+math.atan2(save_pose[first][9],save_pose[first][10])/math.pi*180 +rotate_x         #角度
        y_ro_origin=y_ro_origin+math.atan2(-save_pose[first][8],((save_pose[first][8])**2+(save_pose[first][10])**2)**0.5)/math.pi*180 +rotate_y
        z_ro_origin=z_ro_origin+math.atan2(save_pose[first][4],save_pose[first][0])/math.pi*180 +rotate_z
        ro_origin=ro_origin+np.linalg.norm(a)/math.pi*180
    first=first+1

x_origin=x_origin/num_of_cloud
y_origin=y_origin/num_of_cloud
z_origin=z_origin/num_of_cloud
x_ro_origin=x_ro_origin/num_of_cloud
y_ro_origin=y_ro_origin/num_of_cloud
z_ro_origin=z_ro_origin/num_of_cloud
ro_origin=ro_origin/num_of_cloud

#计算残差和
first=0
for i in save_pose:
    if i==[]:
        break
    trans[0][0]=save_pose[first][0]
    trans[0][1]=save_pose[first][1]
    trans[0][2]=save_pose[first][2]
    trans[0][3]=save_pose[first][3]

    trans[1][0]=save_pose[first][4]
    trans[1][1]=save_pose[first][5]
    trans[1][2]=save_pose[first][6]
    trans[1][3]=save_pose[first][7]

    trans[2][0]=save_pose[first][8]
    trans[2][1]=save_pose[first][9]
    trans[2][2]=save_pose[first][10]
    trans[2][3]=save_pose[first][11]

    trans[3][0]=0
    trans[3][1]=0
    trans[3][2]=0
    trans[3][3]=1
    rotate=trans[0:3,0:3]                       #求旋转矩阵
    a, j = cv2.Rodrigues(rotate)          #求旋转向量
    print("rotate",rotate)
    print("xuanzhuanjiao",np.linalg.norm(a)/math.pi*180)

    if(first<cloud_one):         #点云地图1的残差计算

        residual_x=residual_x+(x_origin-trans[0][3])**2
        print((x_origin-trans[0][3]))

        residual_y=residual_y+(y_origin-trans[1][3])**2
        #print((y_origin-trans[1][3])**2)

        residual_z=residual_z+(z_origin-trans[2][3])**2
        #print((z_origin-trans[2][3])**2)
        

        #旋转量的残差和
        residual_ro_x=residual_ro_x+(x_ro_origin-math.atan2(save_pose[first][9],save_pose[first][10])/math.pi*180)**2
        residual_ro_y=residual_ro_y+(y_ro_origin-math.atan2(-save_pose[first][8],((save_pose[first][8])**2+(save_pose[first][10])**2)**0.5)/math.pi*180)**2
        residual_ro_z=residual_ro_z+(z_ro_origin-math.atan2(save_pose[first][4],save_pose[first][0])/math.pi*180)**2
        residual_ro=residual_ro+(ro_origin-np.linalg.norm(a)/math.pi*180)**2

    if(first>cloud_one or first==cloud_one):

        residual_x=residual_x+((x_origin-change_x)-trans[0][3])**2
        print((x_origin-change_x)-trans[0][3])

        residual_y=residual_y+((y_origin-change_y)-trans[1][3])**2
        #print((y_origin-trans[1][3])**2)

        residual_z=residual_z+((z_origin-change_z)-trans[2][3])**2
        #print((z_origin-trans[2][3])**2)

        #旋转量的残差和
        residual_ro_x=residual_ro_x+((x_ro_origin-rotate_x)- math.atan2(save_pose[first][9],save_pose[first][10])/math.pi*180 )**2
        residual_ro_y=residual_ro_y+((y_ro_origin-rotate_y)- math.atan2(-save_pose[first][8],((save_pose[first][8])**2+(save_pose[first][10])**2)**0.5)/math.pi*180 )**2
        residual_ro_z=residual_ro_z+((z_ro_origin-rotate_z)- math.atan2(save_pose[first][4],save_pose[first][0])/math.pi*180 )**2
        residual_ro=residual_ro+(ro_origin-np.linalg.norm(a)/math.pi*180)**2
    first=first+1

residual_trans=residual_x+residual_y+residual_z     #平移量的残差
#residual_ro=residual_ro_x+residual_ro_y+residual_ro_z     #旋转量的残差

#计算均方根误差
RMSE_x=(residual_x/num_of_cloud)**0.5
print("RMSE_x:",RMSE_x,'m')
RMSE_y=(residual_y/num_of_cloud)**0.5
print("RMSE_y:",RMSE_y,'m')
RMSE_z=(residual_z/num_of_cloud)**0.5
print("RMSE_z",RMSE_z,'m')
RMSE_trans=(residual_trans/num_of_cloud)**0.5
print("RMSE_trans",RMSE_trans,'m')
RMSE_ro_x=(residual_ro_x/num_of_cloud)**0.5
print("RMSE_ro_x:",RMSE_ro_x,'°')
RMSE_ro_y=(residual_ro_y/num_of_cloud)**0.5
print("RMSE_ro_y:",RMSE_ro_y,'°')
RMSE_ro_z=(residual_ro_z/num_of_cloud)**0.5
print("RMSE_ro_z",RMSE_ro_z,'°')
RMSE_rotate=(residual_ro/num_of_cloud)**0.5
print("RMSE_rotate",RMSE_rotate,'°')

pose_txt.close()



