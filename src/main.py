import matplotlib.pyplot as plt
from plot import plotTrajectory, animation_xy
from controller import PID
from simulator import Simulator
from track import Map
import numpy as np

def main():
    # ======================================================================================================================
    # ============================================= Initialize parameters  =================================================
    # ======================================================================================================================
    x0 = np.array([0.5, 0, 0, 0, 0, 0])       # Initial condition
    xS = [x0, x0]

    map = Map(0.4)                            # Initialize map
    vt = 0.8                                  # target vevlocity

    # Init simulators
    simulator     = Simulator(map, multiLap=False)

    # ======================================================================================================================
    # ======================================= PID path following ===========================================================
    # ======================================================================================================================
    print("Starting PID")
    # Initialize pid and run sim
    PIDController = PID(vt)
    xPID_cl, uPID_cl, xPID_cl_glob, _ = simulator.sim(xS, PIDController)
    print("===== PID terminated")

    # # ======================================================================================================================
    # # ========================================= PLOT TRACK =================================================================
    # # ======================================================================================================================
    print("===== Start Plotting")
    plotTrajectory(map, xPID_cl_glob, 'PID')
    animation_xy(map, xPID_cl_glob[::4])
    plt.show()

if __name__== "__main__":
  main()