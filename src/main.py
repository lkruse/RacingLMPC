import matplotlib.pyplot as plt
from plot import plotTrajectory, animation_xy
from controller import PID
from simulator import Simulator
from track import Map
import numpy as np

def main():

    x0 = [np.array([0.5, 0, 0, 0, 0, 0, 0, 0, 0])]

    map = Map()       # Initialize map
    vt = 0.8          # target velocity

    # Init simulators
    simulator     = Simulator(map, multiLap=False)

    print("Starting PID")
    PIDController = PID(vt)
    X, U = simulator.sim(x0, PIDController)
    print("===== PID terminated")


    print("===== Start Plotting")
    plotTrajectory(map, X, 'PID')
    animation_xy(map, X[::4])
    plt.show()

if __name__== "__main__":
  main()