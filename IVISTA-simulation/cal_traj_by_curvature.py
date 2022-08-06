import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import splrep, splev

def calculate_curvature(x_value: np.ndarray, y_value: np.ndarray):
    """计算曲率"""
    x_t = np.gradient(x_value)
    y_t = np.gradient(y_value)
    xx_t = np.gradient(x_t)
    yy_t = np.gradient(y_t)
    curvature_val = np.abs(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t) ** 1.5
    norm1 = y_t / (x_t * x_t + y_t * y_t) ** 0.5
    norm2 = -x_t / (x_t * x_t + y_t * y_t) ** 0.5
    return curvature_val, [norm1, norm2]  ## 曲率 曲率圆心


## 平滑函数
def smooth_ployline(cv_init, point_num=1000):
    cv = cv_init
    list_x = cv[:, 0]
    list_y = cv[:, 1]
    if type(cv) is not np.ndarray:
        cv = np.array(cv)
    delta_cv = cv[1:, ] - cv[:-1, ]
    s_cv = np.linalg.norm(delta_cv, axis=1)

    s_cv = np.array([0] + list(s_cv))
    s_cv = np.cumsum(s_cv)
    bspl_x = splrep(s_cv, list_x, s=0.1)
    bspl_y = splrep(s_cv, list_y, s=0.1)
    # values for the x axis
    s_smooth = np.linspace(0, max(s_cv), point_num)
    # get y values from interpolated curve
    x_smooth = splev(s_smooth, bspl_x)
    y_smooth = splev(s_smooth, bspl_y)
    new_cv = np.array([x_smooth, y_smooth]).T
    delta_new_cv = new_cv[1:, ] - new_cv[:-1, ]
    s_accumulated = np.cumsum(np.linalg.norm(delta_new_cv, axis=1))
    s_accumulated = np.concatenate(([0], s_accumulated), axis=0)
    return new_cv, s_accumulated

def get_central_vertices(pts):
    cv_init = np.array([pts[0], pts[1], pts[2], pts[3], pts[4]])
    cv_smoothed, s_accumulated = smooth_ployline(cv_init)
    return cv_smoothed, s_accumulated


path_data = np.loadtxt(r'C:\Users\房世玉\PycharmProjects\Lattice\roadMap_lzjSouth1.txt')  ### 参考轨迹数据集合 只需要x y
rx = path_data[:, 1]
ry = path_data[:, 2]
ka = []
x = []
y = []
for idx in range(0, 80):  ### 每隔10个点算一次曲率 每个点都算会导致曲率都比较小 差异不大
    idx = idx * 10
    x.append(rx[idx])
    y.append(ry[idx])
x = np.array(x)
y = np.array(y)
kappa, norm = calculate_curvature(x, y)
for i in range(len(kappa)):
    if abs(kappa[i]) <= 0.05:  ## 对于曲率<0.05(这个阈值需要适当调整)时认为就是0 即直线
        kappa[i] = 0
    ka.append(kappa[i])
print(ka)
plt.figure(figsize=(5, 5), dpi=120)
plt.plot(x[:], y[:], label='原轨迹（道路边界）')
plt.quiver(x[:], y[:], ka * norm[0], ka * norm[1])
plt.axis('equal')
#plt.show()

#####
### 现在我们假设有 曲率序列ka 起点[rx[0-10], ry[0-10] -2] 终点[rx[810] - 2, ry[810]] 重构参考轨迹  -2是考虑车道宽为4m
### 一共有起点2个 终点1个 以及弯道的曲率序列(这里我们用txt的轨迹求曲率 并且将txt的轨迹作为道路边界)  ### before line 84
### 首先如果曲率=0(这里是<=0.05， 可根据仿真环境调) 就认为还在直路上 沿着原来的方向开就行 开的长度是dis(两个起点的距离差)   ### line 84-88
### else 进入了弯道 先从np.linspace(0, pi/100, 200)生成一堆theta 用于缓慢的转弯 line 89-102
### 根据这些theta计算下一时刻可能到的两个位置scatter1，2 并根据现在位置和这两个位置 三个点可以计算曲率 line 103-121
### 挑选与实际曲率最接近的theta 如果按照这个theta转弯 得到的轨迹将最接近道路边界的曲率 所以我们就用这个theta移动dis后得到的位置作为规划轨迹 line 122-132
### 第一次规划轨迹序列后半段不准 目前认为最主要的原因是line91的dis txt中每几个散点的dis是不同的 需要看仿真环境的曲率序列 每两个曲率之间代表的道路边界点dis是不是相同 如果相同很可能就可以解决
### 但是不管准不准 我们都可以从生成的轨迹中挑几个散点加上初始位置 结束位置拟合一个轨迹也可以凑合用 也就是从我们第一次得到的规划轨迹中拿两个散点用于第二次规划轨迹 after line132
x_list = [rx[0], rx[10]]
y_list = [ry[0]-2, ry[10]-2]  ## 车道宽2m
pi = 3.14159
for i in range(len(ka)):
    if ka[i] == 0:  ## 如果曲率为0 说明是直线 沿着原来的两点延申即可
        traj_now = [x_list[-1] - x_list[-2] + x_list[-1], y_list[-1] - y_list[-2] + y_list[-1]]
        x_list.append(traj_now[0])
        y_list.append(traj_now[1])
    else:  ## 如果曲率不为0 那么先找曲率圆心 以曲率圆心再生成一个半径 r + 1/2 lane_width 的圆 同样沿着垂直线延申就能得到一个参考散点
        theta_list = np.linspace(0, pi/100, 200)  ## pi/100 可以调整 ****且这个取值将显著的影响结果****
        vector2 = np.array([x_list[-2], y_list[-2]])
        vector1 = np.array([x_list[-1], y_list[-1]])
        dis = np.sqrt(np.sum(np.square(vector2 - vector1)))
        theta_now = np.arccos((x_list[-1] - x_list[-2]) / dis)
        if ((y_list[-1] - y_list[-2]) / dis) < 0:  ## 如果在34象限(sin<0) 取负符号
            theta_now = - theta_now
        # print('theta now', theta_now)


        ka_4_theta = []  ## 记录下不同theta下的曲率和圆心
        no_4_theta = []
        for theta in theta_list:

            if ka[i] > 0:  ## 如果曲率大于0
                scatter1 = [x_list[-1] + np.cos(theta_now - theta) * dis, y_list[-1] + np.sin(theta_now - theta) * dis]
                scatter2 = [scatter1[0] + np.cos(theta_now - 2 * theta) * dis, scatter1[1] + np.sin(theta_now - 2 * theta) * dis]
                x = np.array([x_list[-1], scatter1[0], scatter2[0]])  ## 一共生成三个散点 用以计算曲率
                y = np.array([y_list[-1], scatter1[1], scatter2[1]])
                kappa, norm = calculate_curvature(x, y)  ## 针对这三个散点会生成三个kappa 但是后两个没有后续散点所以 np.gradient不准 只用第一个
                ka_4_theta.append(kappa[0])
                no_4_theta.append(norm[0])

            else:  ## 曲率小于0
                theta_now = theta_now + theta
                scatter1 = [x_list[-1] + np.cos(theta_now) * dis, y_list[-1] + np.sin(theta_now) * dis]
                scatter2 = [scatter1[0] + np.cos(theta_now) * dis, scatter1[1] + np.sin(theta_now) * dis]
                x = np.array([x_list[-1], scatter1[0], scatter2[1]])  ## 一共生成三个散点 用以计算曲率
                y = np.array([y_list[-1], scatter1[1], scatter2[1]])
                kappa, norm = calculate_curvature(x, y)
                ka_4_theta.append(kappa[0])
                no_4_theta.append(norm[0])

        index = np.argmin([abs(kappa-ka[i]) for kappa in ka_4_theta])
        # print('与实际的曲率差距为', np.min([abs(kappa-ka[i]) for kappa in ka_4_theta]))
        theta = theta_list[index]
        # print(theta)
        ## 已知theta再找回这几个散点
        scatter1 = [x_list[-1] + np.cos(theta_now - theta) * dis, y_list[-1] + np.sin(theta_now - theta) * dis]
        x_list.append(scatter1[0])
        y_list.append(scatter1[1])

plt.plot(x_list[:], y_list[:], label='第一次规划轨迹（曲率圆生成）')
### 从这个曲率原生成的轨迹中取几个散点拟合参考轨迹
length = len(x_list)
pts = []  ## 找几个散点作为第二次规划轨迹的散点 由于后半部分第一次规划轨迹不准 所以我们取的散点基本是前半段
pts.append([rx[0], ry[0] - 2])
pts.append([x_list[int(0.25*length)], y_list[int(0.25*length)]])
pts.append([x_list[int(0.3*length)], y_list[int(0.3*length)]])
pts.append([x_list[int(0.6*length)], y_list[int(0.6*length)]])
pts.append([rx[810] -2 , ry[810]])
cv,_ = get_central_vertices(pts)  ## 随便用了个拟合 可以更改
plt.plot(cv[:,0], cv[:,1], label='第二次规划轨迹（散点生成）')
plt.rcParams['font.sans-serif'] = ['KaiTi', 'SimHei', 'FangSong']  # 汉字字体,优先使用楷体，如果找不到楷体，则使用黑体
plt.rcParams['axes.unicode_minus'] = False
plt.legend()
plt.show()



