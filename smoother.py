
import numpy as np
import math
import osqp




    


class wayPoint:
    def __init__(self,x, y, z=0 ,roll=0, pitch=0, yaw=0) -> None:
        self.__x=x
        self.__y=y
        self.__z=z
        # self.__vel_x=vel_x
        # self.__vel_y=vel_y
        # self.__vel_z=vel_z

        self.__roll = roll   ##  rotate along the x-axis
        self.__pitch = pitch    ##  rotate along the y-axis
        self.__yaw =yaw    ##  rotate along the z-axis

    def x(self):
        return self.__x
    
    def y(self):
        return self.__y
    
    def y(self):
        return self.__y

    def yaw(self):
        return self.__yaw

def getDistance(from_point:wayPoint,to_point:wayPoint):
    return math.sqrt(
        (from_point.x()-to_point.x())**2+(from_point.y()-to_point.y())**2
    )


#==================================================================
class  smoother:
    def __init__(self) -> None:
        self.w_vel_smooth=1
        self.w_acc_smooth=0.1
        self.w_obstacle=0.25
        
        self.w_s=1
        self.w_ds=0.5
        self.w_ds2=0.25
        self.w_ds3=1


        self.delta_t=1


    def processPath2LongitudinalSeedProfile(self,path:list):

        path_size = len(path)
        ref_x_ = np.array([])

        ## apend   s
        ref_x_=np.append(ref_x_, [0])
        for index in range(path_size-1):
            p1=path[index]
            p2=path[index+1]
            dis=getDistance(p1,p2)
            ref_x_ = np.append(ref_x_, [dis])

        ## apend   ds
        ref_x_=np.append(ref_x_, [0])
        for index in range(path_size-1):
            p1=path[index]
            p2=path[index+1]
            vel=getDistance(p1,p2)/self.delta_t
            ref_x_ = np.append(ref_x_, [vel])

        ## apend   dds
        ref_x_=np.append(ref_x_, [0])
        for index in range(path_size-1):
            v1= ref_x_[path_size+index]
            v2= ref_x_[path_size+index+1]
            acc=v2-v1/self.delta_t
            ref_x_ = np.append(ref_x_, [acc])


        ref_x_ = ref_x_.reshape(-1, 1)
        # print(ref_x_)

        return ref_x_


    def piece_wise_speed_optimize(self,path:list):
        
        delta_t=0.5
        path_size = len(path)

        ref_x_=self.processPath2LongitudinalSeedProfile(path)

        A_n_n_ = np.diag([self.w_s] * path_size)    ##  build a diagonal matrix
        B_n_n_ = np.diag([self.w_ds] * path_size)  
        C_n_n_ = np.diag([  self.w_ds2+self.w_ds3/(delta_t**2) ] * path_size)    
        C_n_n_[range(path_size - 1), range(1, path_size)] = -2*self.w_ds3/(delta_t**2)
        # 使用 numpy.block 沿对角线拼接这些矩阵
        Q_3n_3n_= np.block([[A_n_n_, np.zeros_like(A_n_n_), np.zeros_like(A_n_n_)],
                                [np.zeros_like(A_n_n_), B_n_n_, np.zeros_like(A_n_n_)],
                                [np.zeros_like(A_n_n_), np.zeros_like(A_n_n_), C_n_n_]])
        # print(Q_3n_3n_)


        q_2n_1_=ref_x_[:3*path_size]
        q_2n_1_[:path_size]=q_2n_1_[:path_size] *(-2*self.w_ds)
        q_2n_1_[path_size: 2*path_size]=q_2n_1_[path_size: 2*path_size] *(-2*self.w_ds2)
        q_2n_1_[2*path_size:3*path_size]=0
        
        ##  inequality constraint 
        A_I_3n_3n_=np.diag([1]*path_size*3)

        A_I_n_n_ = np.zeros((path_size, path_size))
        np.fill_diagonal(A_I_n_n_, -1)        # 设置对角线元素为 -1
        for i in range(1, path_size):         
            A_I_n_n_[i-1, i ] = 1        # 设置对角线上方的元素为 1

        A_dt_n_n_=np.diag([ -self.delta_t]*path_size)   # 设置对角线元素为 -dt
        A_dt2_n_n_=np.diag([ -1/2 * self.delta_t**2 ]*path_size)   # 设置对角线元素为 -dt

        Q_5n_3n_= np.block([[A_I_3n_3n_],
                                                     [A_I_n_n_,     A_dt_n_n_,  A_dt2_n_n_],
                                                     [np.zeros_like(A_I_n_n_),A_I_n_n_,A_dt_n_n_]])
        
        # print(A_I_3n_3n_)
        # print(A_I_n_n_)
        # print(A_dt_n_n_)
        # print(A_dt2_n_n_)
        # print(Q_5n_3n_)



    def solveQPproblem(self):
        pass
        # P = np.array([[2.0, 0.0], [0.0, 2.0]])  # 目标函数的二次项系数矩阵
        # q = np.array([0.0, 0.0])  # 目标函数的线性项系数
        # A = np.array([[1.0, -1.0], [-1.0, 1.0]])  # 约束矩阵
        # l = np.array([1.0, 0.0])  # 不等式约束下限向量
        # u = np.array([2.0, 2.0])  # 不等式约束上限向量

        # # 创建OSQP问题实例
        # prob = osqp.OSQP()

        # # 设置问题参数
        # prob.setup(P=P, q=q, A=A, l=l, u=u)

        # # 求解问题
        # results = prob.solve()

        # # 打印结果
        # print("Optimal solution:")
        # print(results.x)  # 优化变量的值
        # print("Optimal objective value:")
        # print(results.info.obj_val)  # 优化目标函数值




def main():
    test_smoother=smoother()
    # path=[wayPoint(1,1), wayPoint(1.2,3.5), wayPoint(3.2,3.5), wayPoint(3.4,2.9), wayPoint(4.5,3.5)]
    path=[wayPoint(1,1), wayPoint(1.2,3.5), wayPoint(3.2,3.5)]

    test_smoother.piece_wise_speed_optimize(path)




if __name__ == "__main__":
    main()



    # # 创建示例矩阵 A、B 和 C
    #     A = np.array([[1, 2], [3, 4]])
    #     B = np.array([[5, 6], [7, 8]])
    #     C = np.array([[9, 10], [11, 12]])

    #     # 使用 numpy.block 沿对角线拼接这些矩阵
    #     result = np.block([[A, np.zeros_like(A), np.zeros_like(A)],
    #                     [np.zeros_like(A), B, np.zeros_like(A)],
    #                     [np.zeros_like(A), np.zeros_like(A), C]])

    #     print(A)
    #     print(B)
    #     print(C)
    #     print(result)
# def smooth_path(path, configuration_space, w_acc_smooth, w_vel_smooth, w_obstacle, alpha=0.1, max_iterations=100):
#     iterations = 0
#     path_length = len(path)
#     total_weight = w_acc_smooth + w_vel_smooth + w_obstacle
#     temp_path = path[:]

#     while iterations < max_iterations:
#         for i in range(3, path_length - 3):
#             xim3 = np.array([path[i - 3].x, path[i - 3].y, path[i - 3].z])
#             xim2 = np.array([path[i - 2].x, path[i - 2].y, path[i - 2].z])
#             xim1 = np.array([path[i - 1].x, path[i - 1].y, path[i - 1].z])
#             xi = np.array([path[i].x, path[i].y, path[i].z])
#             xip1 = np.array([path[i + 1].x, path[i + 1].y, path[i + 1].z])
#             xip2 = np.array([path[i + 2].x, path[i + 2].y, path[i + 2].z])
#             xip3 = np.array([path[i + 3].x, path[i + 3].y, path[i + 3].z])
#             correction = np.zeros(3)

#             correction += smoothness_term(xim3, xim2, xim1, xi, xip1, xip2, xip3)
#             correction += obstacle_term(xi, configuration_space)

#             xi -= alpha * correction / total_weight

#             temp_path[i].x = xi[0]
#             temp_path[i].y = xi[1]
#             temp_path[i].z = xi[2]

#         path = temp_path
#         iterations += 1

#     for i in range(3, path_length - 3):
#         xi = np.array([path[i].x, path[i].y, path[i].z])
#         xip1 = np.array([path[i + 1].x, path[i + 1].y, path[i + 1].z])
#         Dxi = xip1 - xi
#         path[i].t = np.arctan2(Dxi[1], Dxi[0])

#     return path

# def smoothness_term(xim3, xim2, xim1, xi, xip1, xip2, xip3):
#     # 根据你的平滑度代价函数来实现这个函数
#     # 返回一个3维向量作为修正
#     pass

# def obstacle_term(xi, configuration_space):
#     # 根据障碍物代价函数来实现这个函数
#     # 返回一个3维向量作为修正
#     pass
