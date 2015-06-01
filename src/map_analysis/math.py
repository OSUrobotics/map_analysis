import numpy as np
import scipy.spatial
from itertools import product
# import numexpr as ne

def ssd_mat(mat):
    return np.sum(np.diff(mat, axis=0)**2, axis=1)


def ssd(a, b):
    mat = type(a)([a, b])
    return ssd_mat(mat)


# from
# http://stackoverflow.com/questions/13692801/distance-matrix-of-curves-in-python
def distanceBetweenCurves(C1, C2):
    D = scipy.spatial.distance.cdist(C1, C2, 'euclidean')

    # non symmetric Hausdorff distances
    H1 = np.max(np.min(D, axis=1))
    H2 = np.max(np.min(D, axis=0))

    return (H1 + H2) / 2.


def distanceMatrixOfCurves(Curves):
    numC = len(Curves)

    D = np.zeros((numC, numC))
    for i in range(0, numC - 1):
        for j in range(i + 1, numC):
            D[i, j] = D[j, i] = distanceBetweenCurves(Curves[i], Curves[j])

    return D

from fractions import Fraction

def linePoints((x0, y0), (x1, y1)):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x, y))
    return points


def mnd(x, mu, sigma):
    k = np.shape(sigma)[0]

    # make mu and x the right shape in case they're single points
    x = np.atleast_3d(x)
    mu = np.atleast_3d(mu)

    scale = 1.0 / (np.sqrt((2 * np.pi)**k) * np.linalg.det(sigma))

    x_minus_mu = x - mu
    left = np.array(np.mat(x_minus_mu) * sigma.I)
    right = x_minus_mu.squeeze()
    # do the vector multiplication by hand since
    # I can't figure out a way to get numpy to
    # efficiently multiply arrays of vectors

    exponent = -0.5 * np.sum(left * right, axis=1)
    # exponent = -0.5 * ne.evaluate('sum(left * right, axis=1)')
    return np.asarray(scale * np.exp(exponent)).squeeze()


if __name__ == '__main__':
    cc = np.matrix([[31.94067409, 3.86519414],
                    [3.86519414, 20.17490579]])

    # xx, yy = np.meshgrid(range(-4, 5), range(-4, 5))
    coords = list(product(range(-4, 5), range(-4, 5)))
    pdf = np.zeros(len(coords))
    # pdf = dict()
    # for x y in zip(xx.flatten(), yy.flatten()):
    for i, xy in enumerate(coords):
        pdf[i] = mnd(xy, (0, 0), cc)
        # pdf[xy] = mnd(xy, (0, 0), cc)
    # print pdf[(0,4)]
    # print pdf.keys()

    x = zip(range(-4, 5), [0] * 9)
    # for xy in x:
    #     print mnd(xy, mu, sigma)

    # print mnd(np.array((0, 0)), cc, np.array((0, 0)))
