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


def visualize_axial_trajectory(t, target, actual, axis=2):
    fig = plt.figure()
    ax = fig.gca()
    plt.title('Flight path').set_fontsize(20)
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')

    legend = ['Target', 'Actual']
    t_start = np.min(t)
    ax.plot(np.array(t) - t_start, target[:, axis], c='r')
    ax.plot(np.array(t) - t_start, actual[:, axis], c='g')

    plt.legend(legend, fontsize=14)
    plt.show()


def visualize_axial_error(target, actual, axis=2):
    fig = plt.figure()
    ax = fig.gca()
    plt.title('Error').set_fontsize(20)
    ax.set_xlabel('Time')
    ax.set_ylabel('Error')

    legend = ['Target', 'Actual']
    t_start = np.min(target[:, -1])
    ax.plot(target[:, -1] - t_start, actual[:, axis] - target[:, axis], c='r')

    plt.legend(legend, fontsize=14)
    plt.show()


if __name__ == '__main__':
    with open('flight_log', 'rb') as f:
        all_logs = pickle.load(f)
    traj_t, target_traj, actual_traj, v_t, target_v, actual_v = all_logs
    visualize_axial_trajectory(
        traj_t,
        np.array(target_traj), np.array(actual_traj), axis=1)
    visualize_axial_trajectory(
        v_t,
        np.array(target_v), np.array(actual_v), axis=1)
