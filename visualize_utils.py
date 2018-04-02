import matplotlib.pyplot as plt
import numpy as np
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D
import pickle


def visualize_planned_trajectory(traj, executed=None):
    traj = np.array(traj)
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    plt.title('Flight path').set_fontsize(20)
    ax.set_xlabel('NORTH')
    ax.set_ylabel('EAST')
    ax.set_zlabel('-DOWN')
    ax.invert_xaxis()
    ax.set_zlim([0, int(-np.min(traj[:, 2])) + 1])

    legend = ['Planned']
    ax.plot(traj[:, 0], traj[:, 1], -traj[:, 2])

    if executed is not None:
        executed = np.array(executed)
        ax.plot(executed[:, 0], executed[:, 1], -executed[:, 2], c='r')
        legend.append('Executed')

    plt.legend(legend, fontsize=14)
    plt.show()


def visualize_lateral_trajectory(traj, executed=None):
    traj = np.array(traj)
    fig = plt.figure()
    ax = fig.gca()
    plt.title('Flight path').set_fontsize(20)
    ax.set_xlabel('NORTH')
    ax.set_ylabel('EAST')

    legend = ['Planned']
    ax.plot(traj[:, 0], traj[:, 1], c='b')

    if executed is not None:
        executed = np.array(executed)
        ax.plot(executed[:, 0], executed[:, 1], c='r')
        legend.append('Executed')

    plt.legend(legend, fontsize=14)
    plt.show()


def visualize_altitude_trajectory(target, actual):
    target = np.array(target)
    actual = np.array(actual)
    fig = plt.figure()
    ax = fig.gca()
    plt.title('Flight path').set_fontsize(20)
    ax.set_xlabel('Time')
    ax.set_ylabel('-Down')

    legend = ['Target', 'Actual']
    ax.plot(np.arange(target.shape[0]), -target[:, 2], c='r')
    ax.plot(np.arange(actual.shape[0]), -actual[:, 2], c='g')

    plt.legend(legend, fontsize=14)
    plt.show()


if __name__ == '__main__':
    with open('flight_log', 'rb') as f:
        all_logs = pickle.load(f)
    target, actual = all_logs
    visualize_altitude_trajectory(target, actual)
