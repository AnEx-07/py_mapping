import math
import rospy

import numpy as np

from numba import jit

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from scipy.interpolate import interp1d


class Map(OccupancyGrid, object):

    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self._grid = np.zeros((4000, 4000), dtype="int8")
        self.res = 0.01
        self.x = 0
        self.off_x = 1500
        self.y = 0
        self.off_y = 1500
        self.th = 0
        self.obs_size = 15
        self.pad = 5

    def on_scan(self, data):
        # data
        min_dist = data.range_min
        max_dist = data.range_max
        min_ang = data.angle_min
        max_ang = data.angle_max
        ang_inc = data.angle_increment

        ranges = data.ranges

        len_ranges = len(ranges)

        for i in range(len_ranges):
            dist = ranges[i]
            if min_dist <= dist <= max_dist:
                ang = min_ang+(i*ang_inc)

                self.set_obs(dist, ang)

        print(self._grid)

        sub.unregister()
        
        import matplotlib.pyplot as plt

        plt.imshow(map._grid)
        plt.show()

    
    def set_obs(self, dist, ang):
        # pos
        x = self.pos_x
        y = self.pos_y
        th = self.th

        ang += th

        print("ang:", ang)

        obs_x = (math.cos(ang)*dist)
        obs_y = (math.sin(ang)*dist)

        # print("obs setting...")
        shp = self._grid.shape
        rad = self.obs_size//2

        extra = rad + self.pad

        idx_x = round((obs_x/self.res) + x)
        idx_y = round((obs_y/self.res) + y)

        if idx_x-extra < 0:
            self._grid = np.insert(
                self._grid, 0, np.zeros((abs(idx_x-extra), 1), dtype="int8"), axis=0)

            self.off_x -= idx_x
            idx_x = 0 + extra

        elif idx_x + extra >= shp[0]:
            self._grid = np.insert(
                self._grid, shp[0], np.zeros((idx_x + extra + 1 -
                                              shp[0], 1), dtype="int8"), axis=0)

        shp = self._grid.shape

        if idx_y-extra < 0:
            self._grid = np.insert(
                self._grid, 0, np.zeros((abs(idx_y-extra), 1), dtype="int8"), axis=1)

            self.off_y -= idx_y
            idx_y = 0 + extra

        elif idx_y + extra >= shp[1]:

            # np.insert(a,(0,0),np.ones((a.shape[0],2)),axis=1)
            self._grid = np.insert(
                self._grid, shp[1], np.zeros((idx_y + extra + 1 - shp[1], 1), dtype="int8"), axis=1)


        # a[4:6, 4:6] = b
        
        self.mark_point(idx_x,idx_y,rad)

        ob = self.mark_point(idx_x,idx_y,rad)
        
        bx = []

        for i in ob[0]:
            for j in ob[1]:
                dx = i-x
                dy = j-y
                bx.append(((i,j), math.atan2(dy, dx)))

        bx.sort(key=lambda x: x[1])
        
        p1 = bx[0][0]
        p2 = bx[-1][0]
        p3 = round(x),round(y)
        
        
        self.clear_angle(p1,p2,p3)
        
        
    def mark_point(self,idx_x,idx_y, rad=1, val=1):
        
        ob = ((round(idx_x-rad), round(idx_x+rad)),
              (round(idx_y-rad), round(idx_y+rad)))

        self._grid[slice(*ob[0]), slice(*ob[1])
                   ] = np.ones((rad*2, rad*2), dtype="int8")
        
        return ob
        
        

          
          
      
    def clear_angle(self,*ps):
        assert len(ps) == 3
        
        sx = sorted(ps, key= lambda x: x[0])
        sp = sx[0]
        
        # interp1d
        xs = [i[0] for i in sx]
        ys = [i[1] for i in sx]
        ls = interp1d(xs[::2],ys[::2])
        os = interp1d(xs,ys)
        
        for _x in range(sp[0],sx[-1][0]):
         
            _y1 = ls(_x)
            _y2 = os(_x)
            
        
        
        
        return

    @ property
    def pos_x(self):
        return (self.x / self.res) + self.off_x

    @ property
    def pos_y(self):
        return (self.y / self.res) + self.off_y

    @ property
    def data(self):
        return self._grid

    @ data.setter
    def data(self, val):
        if not isinstance(val, np.ndarray):
            val = np.array(val, dtype="int8")

        if not val.dtype == "int8":
            val.as_dtype("int8")

        self._grid = val


if __name__ == "__main__":
    map = Map()
    rospy.init_node("mapping")
    sub = rospy.Subscriber("/scan", LaserScan, map.on_scan)
    # rospy.wait_for_message("/scan", LaserScan)
    
    map_pub = rospy.Publisher("/map", Map, queue_size=1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        rate.sleep()
