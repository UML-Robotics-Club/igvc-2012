#!/usr/bin/env python
from pyproj import *
utm = Proj(proj='utm', zone='17', datum='WGS84')
ll  = Proj(proj='latlong', datum='WGS84')
print transform(ll, utm, -83.195592, 42.678555)
