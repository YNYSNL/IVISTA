# IVISTA链接：
#### 2022/8/8 20：00 更改算法C中部分内容 主要是面对障碍物lattice会做出更合理的反应 效果可查看网盘中视频
# https://pan.baidu.com/s/1na0RI1zDuudY-z_Q3i4lXQ 
# 提取码：1234
# 同时增加brake_dist, obs_lat_range参数（与障碍物距离为sight_range时进入wait模式， 距离小于brake_dist进入brake，obs与参考轨迹横向距离小于obs_lat_range才会被ego纳入考虑）

#### 2022/8/9 17：00 增加partial_rl文件
# 其中包含所有场景的参考轨迹（场景7可能有问题，其他场景我猜能凑合用）
# 场景之间的连接线除了同时变化xy的部分也有，可以运行下看效果
