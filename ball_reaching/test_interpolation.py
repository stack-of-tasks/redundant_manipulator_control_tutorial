#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as pl

from dynamic_graph.sot.reaching import CubicInterpolationSE3

interpolation = CubicInterpolationSE3 ('interpolation')

init = ((1.,0.,0.,0.),
        (0.,1.,0.,1.),
        (0.,0.,1.,0.),
        (0.,0.,0.,1.))

goal = ((1.,0.,0.,0.),
        (0.,1.,0.,4.),
        (0.,0.,1.,0.),
        (0.,0.,0.,1.))

interpolation.init.value = init
interpolation.goal.value = goal

samplingPeriod = .01
interpolation.setSamplingPeriod (samplingPeriod)
interpolation.start (2.)

times = []
values = []
for t in range (220):
    interpolation.reference.recompute (t)
    reference = interpolation.reference.value
    interpolation.init.value = reference
    times.append (t*samplingPeriod)
    values.append (reference [1][3])

x = np.array (times)
y = np.array (values)

fig = pl.figure ()
ax = fig.add_subplot (111)
ax.plot (x, y)

pl.show ()
