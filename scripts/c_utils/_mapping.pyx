#cython: language_level=3

# ;distutils: define_macros=NPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION

import cython
print("loading _mapping as",("python" if not cython.compiled else "cython"))

import numpy as np
cimport numpy as np
np.import_array()

from libc.stdio cimport printf 
from libc.math cimport sin, cos, abs, pi
from libc.math cimport round as _round

from sensor_msgs.msg import LaserScan


cdef struct _Point2D:
    float x
    float y

cdef struct _MapIdx:
    int h
    int w
    

cdef struct _Pose2D:
    _Point2D point
    float th

cdef int c_round(float x) nogil:
    return <int> _round(x)
    
cdef _Point2D tf_point_to_parent(_Point2D _p, _Pose2D _to) nogil:
    cdef:
        float _x
        float _y
        float cv
        float sv
        _Point2D ret
    
    cv = cos(-_to.th)
    sv = sin(-_to.th)

    _x = _p.x*cv + _p.y*sv
    _y = -_p.x*sv + _p.y*cv 

    ret.x = _x + _to.point.x
    ret.y = _y + _to.point.y

    return ret


cdef _Point2D tf_point_to_local(_Point2D _p, _Pose2D _to) nogil:
    cdef:
        float _x
        float _y
        float cv
        float sv
        _Point2D ret
    
    cv = cos(_to.th)
    sv = sin(_to.th)
    

    _x = _p.x - _to.point.x
    _y = _p.y - _to.point.y

    ret.x = _x*cv + _y*sv
    ret.y = -_x*sv + _y*cv 

    return ret




    
    
cdef class _MAP:

    cdef:
        readonly float pading
        readonly float obs_size
        readonly float _resulation
        readonly _Pose2D origin
        readonly _Pose2D location
        readonly _Pose2D laser_tf
        readonly np.ndarray grid
        
    @property
    def resulation(self):
        return self._resulation

    @resulation.setter
    def resulation(self,val):
        if val < 0.001:
            raise ValueError("resulation cant be 0! suggested (0.0001 < resulation)")

        self._resulation = val

    def map_to_origin(self):
        return _Point2D(self.origin.point.x,self.origin.point.y)
        
    cdef void _add_origin_offset(self, _Point2D _p) nogil:
        self.origin.point.x+= _p.x
        self.origin.point.y+= _p.y

    def __cinit__(self,resulation=0.01):
        self.origin = _Pose2D(_Point2D(0.,0.), 0.)
        print(self.origin)
        self.grid = np.zeros((1,1),dtype=np.int8)
        # self.location = _Pose2D(_Point2D(0.,0.),0.)
        self.laser_tf = _Pose2D(_Point2D(0.45,0.),0.)
        self.resulation = resulation
        self.obs_size = 0.05
        self.pading = 1.0
        assert self.resulation

    cpdef void on_odom(self,data):
        self.location.point.x = data.pose.pose.position.x
        self.location.point.y = data.pose.pose.position.y

    
    
    cpdef void on_data(self, data: LaserScan):
        """callback fn for laser scan topic.

        Args:
            data (LaserScan): data from topic in sensor_msgs/LaserScan format.
        """
        
        # ranges = cython.declare(c_float[:])
        
        cdef:
            float min_dist = data.range_min
            float max_dist = data.range_max
            float min_ang = data.angle_min
            float max_ang = data.angle_max
            float ang_inc = data.angle_increment
            float [:] ranges = np.array(data.ranges,dtype=np.float32)

        cdef:            
            int i
            float dist
            float ang
            int len_ranges = ranges.size


        for i in range(len_ranges):
            with cython.nonecheck(False) :
                dist = ranges[i]
                
            if (min_dist <= dist <= max_dist):
                ang = min_ang+(i*ang_inc)
                self._set_obs(dist, ang)
                    
        printf("done...\n")


    cdef void _set_obs(self, float dist, float ang) nogil : 
        cdef:
            _Point2D obs_p 
            _MapIdx obs_idx

        obs_p.x = cos(ang)*dist 
        obs_p.y = sin(ang)*dist

        obs_idx = self._obs_to_map_idx(obs_p)

        self._mark_idx(obs_idx)



    cpdef void set_obs(self, float dist, float ang):  
        self._set_obs(dist,ang)
        
    
    cdef void _mark_idx(self, _MapIdx _idx) nogil:

        cdef:
            int insert_size
            int[2] _shp
            _MapIdx _insert_idx
            int rad = c_round(self.obs_size/2/self._resulation)
            int extra = c_round((0 + self.pading)/self._resulation) # rad + self.pad


        with gil:
            _shp[0] = self.grid.shape[0]
            _shp[1] = self.grid.shape[1]

            if _idx.h - extra < 0 :
                insert_size = abs(_idx.h-extra)
                self.grid = np.insert(self.grid, 0, np.zeros((abs(insert_size), 1), dtype="int8"), axis=0)

                _insert_idx.w = 0
                _insert_idx.h = insert_size
                self._add_origin_offset(self._map_idx_to_point(_insert_idx)) 
                _idx.h += insert_size
                _shp[0] += insert_size

            if _idx.h + extra >= _shp[0]:
                insert_size = _idx.h + extra - (_shp[0] - 1)
                self.grid = np.insert(self.grid, _shp[0], np.zeros((insert_size, 1), dtype="int8"), axis=0)

                _shp[0] += insert_size

            if _idx.w - extra < 0:
                insert_size = abs(_idx.w - extra)

                self.grid = np.insert(self.grid, 0, np.zeros((insert_size, 1), dtype="int8"), axis=1)

                _insert_idx.h = 0
                _insert_idx.w = insert_size
                self._add_origin_offset(self._map_idx_to_point(_insert_idx)) 
                _idx.w += insert_size
                _shp[1] += insert_size

            if _idx.w + extra >= _shp[1]:
                insert_size = _idx.w + extra - (_shp[1] - 1)
                
                self.grid = np.insert(self.grid, _shp[1], np.zeros((insert_size, 1), dtype="int8"), axis=1)

                _shp[1] += insert_size

            self.grid[_idx.h-5:_idx.h+5,_idx.w-5:_idx.w+5] = 100 


        # # a[4:6, 4:6] = b
        
        
    

    cdef _Point2D _obs_to_map (self,_Point2D _p) nogil:
        cdef: 
            _Point2D _mp

        # to base
        _mp = tf_point_to_parent(_p,self.laser_tf)

        # to origin
        _mp = tf_point_to_parent(_mp, self.location)

        # to map
        _mp = tf_point_to_parent(_mp, self.origin)

        return _mp


    cdef _Point2D _map_idx_to_point (self, _MapIdx _idx ) nogil:
        cdef _Point2D ret

        ret.y = _idx.h * self._resulation
        ret.x = _idx.w * self._resulation

        return ret

    cdef _MapIdx _point_to_map_idx(self, _Point2D _p) nogil:
        cdef _MapIdx ret


        with cython.cdivision(True):
            ret.w= c_round(_p.x/self._resulation)
            ret.h = c_round(_p.y/self._resulation)
                
        return ret



    
    cdef _MapIdx _obs_to_map_idx(self, _Point2D _p) nogil:
        cdef:
            _MapIdx ret
            _Point2D _mp = self._obs_to_map(_p)

        return self._point_to_map_idx(_mp)
            
            
        



    cpdef _MapIdx obs_to_map_idx(self, float x, float y):
        cdef _Point2D _p

        _p.x = x
        _p.y = y
        return self._obs_to_map_idx(_p)
        
    
