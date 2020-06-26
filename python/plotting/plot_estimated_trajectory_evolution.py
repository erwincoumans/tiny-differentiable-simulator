import glob
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, ImageMagickFileWriter

'''Plots an animation of trajectory evolution.

Run in the directory containing the estimated trajectory csvs.
'''

# def main():
#     data = dict()
#     truth = pd.read_table('./true_trajectory.csv', header=None)
#     for fn in glob.glob('./step_*_estimated_trajectory.csv'):
#         _, step, _, _ = fn.split('_')
#         data[int(step)] = pd.read_table(fn, header=None)
#
#     fig, ax = plt.subplots()
#     truth_ln1, = plt.plot(truth[0], truth[1], 'r-', label="True $\\mathbf{q}_0$")
#     truth_ln2, = plt.plot(truth[0], truth[2], 'b-', label="True $\\mathbf{q}_1$")
#     est_ln1, = plt.plot([], [], 'r--', label="Estimated $\\mathbf{q}_0$")
#     est_ln2, = plt.plot([], [], 'b--', label="Estimated $\\mathbf{q}_1$")
#     plt.legend()
#
#
#     def render_frame(frame):
#         est_ln1.set_data(data[frame][0], data[frame][1])
#         est_ln2.set_data(data[frame][0], data[frame][2])
#
#     ani = FuncAnimation(fig, render_frame, frames = range(len(data)))
#     writer = ImageMagickFileWriter()
#     ani.save('evolution.gif', writer=writer)
#     plt.show()


def main():
    truth = pd.read_table('./true_trajectory.csv', header=None)
    est = pd.read_table('./best_estimated_trajectory.csv', header=None)

    fig = plt.figure(figsize=(3.5, 4))
    fig.subplots()
    plt.plot(truth[0], truth[1], 'r-', label="True $\\mathbf{q}_0$")
    plt.plot(truth[0], truth[2], 'b-', label="True $\\mathbf{q}_1$")
    plt.plot(est[0], est[1], 'r--', label="Estimated $\\mathbf{q}_0$")
    plt.plot(est[0], est[2], 'b--', label="Estimated $\\mathbf{q}_1$")
    plt.legend()

    plt.grid()
    plt.xlabel("Time [s]")

    plt.savefig("ibm_pendulum_estimation.pdf")

    plt.show()


if __name__ == '__main__':
    main()
