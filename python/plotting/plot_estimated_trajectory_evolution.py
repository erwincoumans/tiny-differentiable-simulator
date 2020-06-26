import glob
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, ImageMagickFileWriter

'''Plots an animation of trajectory evolution.

Run in the directory containing the estimated trajectory csvs.
'''

def main():
    data = dict()
    truth = pd.read_table('./true_trajectory.csv', header=None)
    for fn in glob.glob('./step_*_estimated_trajectory.csv'):
        _, step, _, _ = fn.split('_')
        data[int(step)] = pd.read_table(fn, header=None)

    fig, ax = plt.subplots()
    truth_ln1, = plt.plot(truth[0], truth[1], 'r-')
    truth_ln2, = plt.plot(truth[0], truth[2], 'b-')
    est_ln1, = plt.plot([], [], 'r--')
    est_ln2, = plt.plot([], [], 'b--')

    def render_frame(frame):
        est_ln1.set_data(data[frame][0], data[frame][1])
        est_ln2.set_data(data[frame][0], data[frame][2])

    ani = FuncAnimation(fig, render_frame, frames = range(len(data)))
    writer = ImageMagickFileWriter()
    ani.save('evolution.gif', writer=writer)
    plt.show()


if __name__ == '__main__':
    main()
