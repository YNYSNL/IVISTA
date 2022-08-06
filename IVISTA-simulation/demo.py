# -*- coding: utf-8 -*-
# from re import M
import sim_pb2
from carlink_api import CarlinkAPI
# import algorithmsA  as alg #导入算法\
import sys
# import algorithmsB  as alg #导入算法\
import algorithmsC  as alg #导入算法\


# 初始化API，参数是LOG文件的路径
api = CarlinkAPI("")

#回调函数定义
def Data_Process_Function(rdb_str, scene_str, msg_type):
    print("Data_Process_Function : ", msg_type, rdb_str)
    msg_type = int(msg_type)
    #从rdb_str中解析sim_rdb_info
    #print(msg_type)
    if msg_type == 5:
        print("pythonif")
        rdb = sim_pb2.sim_rdb_info()
        print("113",rdb_str)
        rdb.ParseFromString(rdb_str)
        print(msg_type)
        tc =  sim_pb2.Traffic_driveControll()
        drive_control = alg.alg_1(rdb)
        tc.acceleration_target = float(drive_control[0])
        tc.steering_wheel = float(drive_control[1])
        tc.steering_speed = float(drive_control[2])
        #将tc序列化为字符串返回
        strret = tc.SerializeToString()
        print("tc",drive_control[0],drive_control[1],drive_control[2])
        print("python str return ", strret)
        return strret

def main(taskId):
    
    api.Set_Data_Process_Function(Data_Process_Function)
    api.Connect_Carlink_Start_Simulation(taskId,10)
    api.Set_Start_Position_And_Simulation_Time(1900, -1, 30)#s,laneid,simulation_time
    api.Wait_Simulation_Stop()
    api.Disconnect_Carlink()
    print('python exit')
    

if __name__ == '__main__':
    taskId = "784883207661879297"
    main(taskId)#在这里直接写死任务id，也可以不用写，上传后平台会自动给出。
