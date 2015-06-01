#!/usr/bin/env python
import yaml
import roslib
import os
import numpy as np
from itertools import izip, product, repeat

def make_name(vals):
    return 'config_'+str(hash('config_' + '_'.join(['%s%1.2f' % v for v in vals]))) + '.yaml'

yaml_dir = os.path.join(roslib.packages.get_pkg_dir('map_analysis'), 'laser_opt_configs')

template = yaml.load(open(os.path.join(yaml_dir, 'template.yaml'), 'r'))


all_vals = []

for param, (val_min, val_max, steps) in template['optimization_params'].iteritems():
    vals = np.linspace(val_min, val_max, num=steps, endpoint=True)
    all_vals.append(iter(izip(repeat(param), vals.tolist())))

for vals in product(*all_vals):
    new_dict = dict(template)
    del new_dict['optimization_params']
    new_dict.update(vals)
    yaml.dump(new_dict, open(os.path.join(yaml_dir, make_name(vals)), 'w'))