import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plotTrajectory(map, X, stringTitle):
    Points = int(np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4])))
    Points1 = np.zeros((Points+1, 2))
    Points2 = np.zeros((Points+1, 2))
    Points0 = np.zeros((Points+1, 2))
    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)
    Points1[-1, :] = map.getGlobalPosition(0 * 0.1, map.halfWidth)
    Points2[-1, :] = map.getGlobalPosition(0 * 0.1, -map.halfWidth)
    Points0[-1, :] = map.getGlobalPosition(0 * 0.1, 0)

    vels = np.linalg.norm(np.vstack([X[:,0], X[:,1]]),axis=0)
    norm = plt.Normalize(vels.min(), vels.max())
    plt.figure()
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    
    sc = plt.scatter(X[:, 7], X[:, 8], c=vels, cmap='inferno', norm=norm, s = 10)
    cbar = plt.colorbar(sc)
    cbar.set_label('Velocity')
    plt.title(stringTitle)


def animation_xy(map, x_glob):
    Points = int(np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4])))
    Points1 = np.zeros((Points+1, 2))
    Points2 = np.zeros((Points+1, 2))
    Points0 = np.zeros((Points+1, 2))

    for i in range(0, int(Points)):
        Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
        Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
        Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)
    Points1[-1, :] = map.getGlobalPosition(0 * 0.1, map.halfWidth)
    Points2[-1, :] = map.getGlobalPosition(0 * 0.1, -map.halfWidth)
    Points0[-1, :] = map.getGlobalPosition(0 * 0.1, 0)

    plt.figure()
    plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
    plt.plot(Points0[:, 0], Points0[:, 1], '--')
    plt.plot(Points1[:, 0], Points1[:, 1], '-b')
    plt.plot(Points2[:, 0], Points2[:, 1], '-b')
    #plt.plot(x_glob[:, 4], x_glob[:, 5], 'k', label="Closed-loop trajectory",zorder=-1)

    ax = plt.gca()
    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]])
    rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
    ax.add_patch(rec)

    for i in range(0, x_glob.shape[0]):

        x = x_glob[i, 7]
        y = x_glob[i, 8]
        psi = x_glob[i, 6]
        l = 0.4; w = 0.2
        car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l*np.cos(psi) + w * np.sin(psi),
                    x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
        car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
                    y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]

        rec.set_xy(np.array([car_x, car_y]).T)

        plt.savefig("figs/{:04d}.png".format(i))
