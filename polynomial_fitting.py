import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import Ridge, LinearRegression
import math
from operator import itemgetter, attrgetter

def plot_trajectories():
    # v_data = pd.read_csv('P07_v_data.csv')
    v_data = pd.read_csv('participants_vh_unbraced_data/P07_v_data.csv')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(50): # [1, 43]: #range(6,7):
        filter = np.where(v_data['interval_label'] == i)
        shoulder_x = np.array(v_data['v_Sh_x'])[filter]
        shoulder_y = np.array(v_data['v_Sh_y'])[filter]
        shoulder_z = np.array(v_data['v_Sh_z'])[filter]

        elbow_x = np.array(v_data['v_Elbow_x'])[filter]
        elbow_y = np.array(v_data['v_Elbow_y'])[filter]
        elbow_z = np.array(v_data['v_Elbow_z'])[filter]

        wrist_x = np.array(v_data['v_Wrist_x'])[filter]
        wrist_y = np.array(v_data['v_Wrist_y'])[filter]
        wrist_z = np.array(v_data['v_Wrist_z'])[filter]
        print(wrist_x)
        print(wrist_y)
        print(wrist_z)
        # for j in range(len(wrist_x)):
        #     if j % 10 == 0:
        #         print(wrist_x[j], wrist_y[j], wrist_z[j], np.array(v_data['interval_label'])[filter][j])
        #         ax.plot([elbow_x[j], wrist_x[j]], [elbow_y[j], wrist_y[j]], zs=[elbow_z[j], wrist_z[j]])
        #         ax.plot([elbow_x[j], shoulder_x[j]], [elbow_y[j], shoulder_y[j]], zs=[elbow_z[j], shoulder_z[j]])
        # sc = ax.scatter(elbow_x, elbow_y, elbow_z, c = np.arange(len(elbow_x)), s=2, cmap='viridis')
        sc = ax.scatter(wrist_x, wrist_y, wrist_z, c = np.arange(len(wrist_x)), s=2, cmap='viridis')
        # sc = ax.scatter(shoulder_x, shoulder_y, shoulder_z, c = np.arange(len(shoulder_x)), s=2, cmap='viridis')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.colorbar(sc, label='Time')
    plt.show()

def get_inds(goal_y, goal_z):
    data = pd.read_csv('P07_h_allintervals.csv')
    sort_list = []
    for i in range(1,50):
        filter = np.where(data['interval_label'] == i)
        wrist_x = np.array(data['Scaled_Wrist_x'])[filter]
        wrist_y = np.array(data['Scaled_Wrist_y'])[filter]
        wrist_z = np.array(data['Scaled_Wrist_z'])[filter]

        dist = math.dist([wrist_y[-1], wrist_z[-1]], [goal_y, goal_z])
        sort_list.append((dist, i))
    sorted_list = sorted(sort_list, key=itemgetter(0))

    inds = [sorted_list[0][1], sorted_list[1][1], sorted_list[2][1], sorted_list[3][1]]
    return inds

def fit_poly(inds=[1,7,43,49]):
    # v_data = pd.read_csv('participants_vh_unbraced_data/P07_v_data.csv')
    data = pd.read_csv('P07_h_allintervals.csv')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    trajs = []
    for i in inds:
        filter = np.where(data['interval_label'] == i)

        wrist_x = np.array(data['Scaled_Wrist_x'])[filter]
        wrist_y = np.array(data['Scaled_Wrist_y'])[filter]
        wrist_z = np.array(data['Scaled_Wrist_z'])[filter]
        wrist_angle = np.array(data['h_Wrist_Angle_y_rad'])[filter]

        X = wrist_x.reshape(len(wrist_x), 1)
        poly = PolynomialFeatures(12)
        X = poly.fit_transform(X)
        y = np.column_stack((wrist_y, wrist_z, wrist_angle))
        model = LinearRegression()
        model.fit(X, y)
        X_plot = wrist_x
        X_inter = poly.transform(X_plot.reshape(len(wrist_x), 1))
        y_plot = model.predict(X_inter)

        sc = ax.scatter(wrist_x, y_plot[:,0],y_plot[:,1],s=2)
        sc = ax.scatter(wrist_x, wrist_y, wrist_z, c = np.arange(len(wrist_x)), s=2, cmap='viridis')
        trajs.append((poly, model))
    plt.xlabel('x', fontsize=10)
    plt.ylabel('y', fontsize=10)
    plt.colorbar(sc, label='Time')

    return trajs, ax

def get_traj(trajs, ax, goal_y, goal_z):
    xs = np.linspace(0.03, 0.115, 200)

    ys = []
    for traj in trajs:
        poly = traj[0]
        model = traj[1]
        X_inter = poly.transform(xs.reshape(len(xs), 1))
        y_plot = model.predict(X_inter)
        ys.append(y_plot)

    # [goal_y, goal_z] = ys[-1,:] * w
    sol = np.linalg.lstsq(np.array(ys)[:,-1,:2].T, np.array([goal_y, goal_z]).T)[0]
    print("SOL", sol)
    means = []
    for i in range(np.array(ys).shape[1]):
        means.append(sol @ np.array(ys)[:,i,:])
        # print(sol @ np.array(ys)[:,i,:])
    means = np.array(means)
    print(means[-1,:2])
    sc = ax.scatter(xs, means[:,0], means[:,1],s=2)

    df = pd.DataFrame(data=np.column_stack((xs, means[:,0], means[:,1], means[:,2])), \
                      columns=['Scaled_Wrist_x', 'Scaled_Wrist_y', 'Scaled_Wrist_z', 'h_Wrist_Angle_y_rad'])
    df.to_csv('interpolation_goal_0.06_0.1_20.csv')

if __name__ == "__main__":
    plot_trajectories()
    # fit_poly(np.arange(1,20))
    goal_y = -0.05 # -0.05
    goal_z = 0.1
    inds = get_inds(goal_y, goal_z)
    print("inds", inds)
    trajs, ax = fit_poly(inds)
    get_traj(trajs, ax, goal_y, goal_z)
    plt.show()