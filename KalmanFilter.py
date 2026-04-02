import numpy as np
import matplotlib.pyplot as plt

t=np.linspace(1,100,100)

# 初始条件
u=0.6   #加速度值，匀加速直线模型
v0=5    #理论初速度
s0=0    #理论初位置
X_true=np.array([[s0],[v0]])
size=t.shape[0]+1 #?
dims=2 #2维

# 超参数
Q=np.array([[1e1,0],[0,1e1]]) #过程噪声协方差矩阵
R=np.array([[1e4,0],[0,1e4]]) #观测噪声协方差矩阵
P=np.array([[0.1,0],[0,0.1]]) #先验误差协方差矩阵，经验给出

# 初始化
X=np.array([[0],[0]])
# 线性变换矩阵
F=np.array([[1,1],[0,1]]) #状态转移矩阵
B=np.array([[1/2],[1]])   #控制矩阵
H=np.array([[1,0],[0,1]]) #观测矩阵

# 真实值
real_positions=np.array([0]*size)
real_speeds=np.array([0]*size)
real_positions[0]=s0
# 观测值（理论值加观测噪声）（初值=理论初值＋观测噪声）
measure_positions=np.array([0]*size)
measure_speeds=np.array([0]*size)
measure_positions[0]=real_positions[0]+np.random.normal(0,R[0][0]**0.5)
#  最有估计值
optim_positions = np.array([0] * size)
optim_positions[0] = measure_positions[0]
optim_speeds = np.array([0] * size)

for i in range(1,t.shape[0]+1):
    w = np.array([[np.random.normal(0, Q[0][0] ** 0.5)], [np.random.normal(0, Q[1][1] ** 0.5)]])
    X_true=F@X_true+B*u+w
    real_positions[i]=X_true[0]
    real_speeds[i] = X_true[1]
    v = np.array([[np.random.normal(0, R[0][0] ** 0.5)], [np.random.normal(0, R[1][1] ** 0.5)]])

    Z=H@X_true+v
    # 预测
    X_=F@X+B*u
    P_=F@P@F.T+Q
    # 更新
    K = P_ @ H.T @ np.linalg.inv(H @ P_ @ H.T + R)
    X = X_ + K @ (Z - H @ X_)
    P = (np.eye(2) - K @ H) @ P_
    # 记录结果
    optim_positions[i] = X[0][0]
    optim_speeds[i] = X[1][0]
    measure_positions[i] = Z[0]
    measure_speeds[i] = Z[1]

t = np.concatenate((np.array([0]), t))
plt.plot(t,real_positions,label='real positions')
plt.plot(t,measure_positions,label='measured positions')
plt.plot(t,optim_positions,label='kalman filtered positions')

plt.legend()
plt.show()

plt.plot(t,real_speeds,label='real speeds')
plt.plot(t,measure_speeds,label='measured speeds')
plt.plot(t,optim_speeds,label='kalman filtered speeds')

plt.legend()
plt.show()


