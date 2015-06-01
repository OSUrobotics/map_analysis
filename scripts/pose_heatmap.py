#!/usr/bin/env rosh
from privacy_zones.map_geometry import MapGeometry
import numpy as np
import matplotlib.pyplot as plt
from scipy.misc import imread
from map_analysis.math import linePoints, mnd
import time

# b = Bag('/media/lazewatd/data/wheelchair_logs/04-30/poses.bag')
b = Bag('/media/lazewatd/data/wheelchair_logs/05-31/poses.bag')

for _, map_metadata, _ in b.topics.map_metadata:
    break 

mg = MapGeometry(map_metadata=map_metadata)
res = mg.map_info['resolution']
poses = []
cov = []
for _, p, _ in b.topics.amcl_pose:
    poses.append((p.pose.pose.position.x, p.pose.pose.position.y))
    cov.append(np.multiply(p.pose.covariance, res))

pixels = mg.world_coords_to_px(poses).astype(int)
# pixels = [(100,100),(200,200)]

# map_img = imread('/home/lazewatd/Dropbox/research/ssalsr-maps/map0/map-edited1.pgm')
map_img = imread('/home/lazewatd/Dropbox/research/map_analysis_ws/src/map_analysis/map-edited1.pgm')
map_plot = plt.imshow(map_img, zorder=0)
map_plot.set_cmap('gray')

pixels_ext = []
cov_ext = []
# pixels_ext = np.array(sum([linePoints(a, b) for a, b in zip(pixels, pixels[1:])], []))

heatmap = np.zeros_like(map_img, dtype=np.float32)

sz = 20
kernel = np.vstack(np.mgrid[-sz:sz+1,-sz:sz+1]).reshape(2,-1).T

for (a, b), (cova, covb) in zip(zip(pixels, pixels[1:]), zip(cov, cov[1:])):
    cova = np.mat(cova).reshape(6,6)[:2,:2]
    covb = np.mat(covb).reshape(6,6)[:2,:2]
    if np.linalg.norm(a-b) >= 30:
        lp = [a] # this line just uses original points
    else:
        lp = linePoints(a,b) # this line connects the points
    # pixels_ext.extend(lp)
    for weight, p in zip(np.linspace(1, 0, len(lp)), lp):
        cov_out = weight * cova + (1 - weight) * covb
        cov_ext.append(cov_out)
        x = kernel + p
        pdf = mnd(x, p, cov_out)
        heatmap[zip(*np.fliplr(x))] += pdf


dims = mg.get_map_dims()
fig = plt.figure(1)
fig.suptitle('Location Frequencies')
axes = fig.axes[0]
axes.set_xticklabels(np.linspace(0,dims[0].round(decimals=1),11))
axes.set_yticklabels(np.linspace(0,dims[1].round(decimals=1),11))
axes.set_xlabel('x-postition (m)')
axes.set_ylabel('y-postition (m)')

plt.axis('image')

heatmap_masked = np.ma.masked_where(heatmap==0, heatmap)

# plt.scatter(*pixels_ext.T)
    # heatmap_plot = plt.imshow(heatmap_masked)
    # heatmap_plot.set_cmap('Oranges')
    # plt.colorbar()
    # time.sleep(0.0000001)
    # plt.draw()

# mouseover for figure
axes.format_coord = lambda x, y: 'x=%s, y=%s: %s' % (x, y, heatmap[y,x])
heatmap_plot = plt.imshow(np.ma.log(heatmap_masked))
heatmap_plot.set_cmap('jet')
cb = plt.colorbar()
cb.set_label('Log frequency (arbitrary units)')
plt.show()