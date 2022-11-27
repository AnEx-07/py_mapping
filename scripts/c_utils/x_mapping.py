#cython: language_level=3
# distutils: define_macros=NPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION

import cython
print("loading _mapping as",("python" if not cython.compiled else "cython"))

import numpy as np

from sensor_msgs.msg import LaserScan


c_none = cython.typedef(cython.void)
c_int  = cython.typedef(cython.int)
c_float = cython.typedef(cython.float)
np_array = cython.typedef(np.ndarray)

if not cython.compiled:
    from math import sin, cos
      
else:
    _round: round
    
    @cython.nogil
    @cython.cfunc
    def round(x: c_float) -> c_int:
        return cython.cast(c_int,_round(x))
    
    
    
            
    
    
@cython.cclass
class _MAP:
    
    
    
    @cython.boundscheck(False) 
    @cython.wraparound(False)
    @cython.ccall
    def on_data(self, data: LaserScan):
        """callback fn for laser scan topic.

        Args:
            data (LaserScan): data from topic in sensor_msgs/LaserScan format.
        """
        
        # ranges = cython.declare(c_float[:])
        
        min_dist: c_float = data.range_min
        max_dist: c_float = data.range_max
        min_ang: c_float = data.angle_min
        max_ang: c_float = data.angle_max
        ang_inc: c_float = data.angle_increment
        ranges: c_float[:] = np.array(data.ranges,dtype=np.float32)


        with cython.nogil:
            len_ranges: c_int = len(ranges)
            i: c_int
            dist: c_float
            ang: c_float
            
            for i in range(len_ranges):
                with cython.nonecheck(False) :
                    dist = ranges[i]
                    
                if (min_dist <= dist) & (dist <= max_dist):
                    ang = min_ang+(i*ang_inc)
                    # with cython.gil:
                    self._set_obs(dist, ang)
                    
        print("done...")

    @cython.nogil
    @cython.cfunc
    def _set_obs(self,dist: c_float, ang: c_float) -> c_none:
        obs_x: c_int = round(cos(ang)*dist)
        obs_y: c_int = round(sin(ang)*dist)
        
    
    @cython.ccall
    def set_obs(self,dist: c_float, ang: c_float):  
        self._set_obs(dist,ang)
        
        # print(obs_x,obs_y)