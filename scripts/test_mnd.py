#!/usr/bin/env python
import numpy as np
def mnd(x, mu, sigma):
    k = np.shape(sigma)[0]

    # make mu and x column vectors
    x = np.atleast_3d(x)
    # x = np.matrix(x).T
    mu = np.atleast_3d(mu)
    # mu = np.matrix(mu).T
    x_minus_mu = x - mu
    left = np.array(np.mat(x_minus_mu) * sigma.I)
    # exponent = -0.5 * np.sum(np.mat(x_minus_mu)*sigma.I*np.mat(x_minus_mu).T, axis=1)
    right = x_minus_mu.squeeze()
    exponent = -0.5 * np.sum(left * right, axis=1)
    return (1.0 / (np.sqrt((2 * np.pi)**k) * np.linalg.det(sigma)) * np.exp(exponent))
        # np.exp(-0.5 * (x - mu).T * sigma.I * (x - mu))

# def mnd(x, mu, sigma):
#     k = np.shape(sigma)[0]

#     # make mu and x column vectors
#     x = np.matrix(x).T
#     mu = np.matrix(mu).T
#     x_minus_mu = x - mu
#     left = np.array(x_minus_mu.T * sigma.I)
#     right = x_minus_mu.squeeze()
#     # do the vector multiply by hand since I can't figure out how to get
#     # numpy to do it
#     exponent = -0.5 * np.sum(left * right, axis=1)
#     return 1.0 / (np.sqrt((2 * np.pi)**k) * np.linalg.det(sigma)) * \
#         np.exp(exponent)
#         # np.exp(-0.5 * (x - mu).T * sigma.I * (x - mu))


def mnd_slow(x, mu, sigma):
    k = np.shape(sigma)[0]

    # make mu and x column vectors
    x = np.matrix(x).T
    mu = np.matrix(mu).T
    # print x, np.shape(x), np.shape(mu), np.shape(sigma)
    # return (2 * np.pi)**(-k / 2.0) * np.linalg.det(sigma)**(1 / 2.0) * \
    exponent = -0.5 * (x - mu).T * np.linalg.inv(sigma) * (x - mu)
    return 1.0 / (np.sqrt((2 * np.pi)**k) * np.linalg.det(sigma)) * \
        np.exp(exponent)

x = np.load(open('/home/lazewatd/Dropbox/research/map_analysis_ws/src/map_analysis/scripts/kernel.pkl', 'r'))

sigma = np.matrix([[ 0.02433246, -0.00095878],
                   [-0.00095878,  0.02763094]])

mu = x[840]

# def main():
# pdf = mnd(x, mu, sigma)
pdf_batch = mnd(x, mu, sigma)
pdf_fast = np.zeros(41*41)
exp_fast = np.zeros(41*41)
pdf_slow = np.zeros(41*41)
exp_slow = np.zeros(41*41)

for i, px in enumerate(x):
    fast = mnd(px, mu, sigma)
    pdf_fast[i] = np.squeeze(np.asarray(fast))
    slow = mnd_slow(px, mu, sigma)
    pdf_slow[i] = np.squeeze(np.asarray(slow))
    print fast, slow

print pdf_fast.shape
print pdf_fast
import matplotlib.pyplot as plt
plt.imshow(pdf_batch.reshape(41,41))
plt.show()

# if __name__ == '__main__':
#     main()

    # left row vector (1,2)
    # right col vector (2,1)