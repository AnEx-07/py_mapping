import rospy
from rospy import Time 

from utils.time_dec import monitor
from c_utils._mapping import _MAP


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid,MapMetaData, Odometry

from tf import TransformBroadcaster, TransformListener
from tf.transformations import euler_from_quaternion


class Map(_MAP):
    
    on_data = monitor(_MAP.on_data)
    
    def cb(self,data):
        sub.unregister()
        self.on_data(data)
        print(self.grid)

        
        
        import matplotlib.pyplot as plt

        plt.imshow(map.grid)
        plt.show()
        
def on_map(data):
    print(min(data.data))
    print(max(data.data))
    


if __name__ == "__main__":
    rospy.init_node("mapping")
    TransformListener()
    
    map = Map()
    # sub = rospy.Subscriber("/odom", Odometry, map.on_odom)
    sub = rospy.Subscriber("/scan", LaserScan, map.on_data)
    # sub = rospy.Subscriber("/map", OccupancyGrid, on_map)
    tfb = TransformBroadcaster()
    
    
    # rospy.wait_for_message("/scan", LaserScan)
    
    og = OccupancyGrid()
    og.info.resolution = map.resulation
    
    
    og.header.frame_id="map2"
    og.header.stamp = Time.now()
    
    
    
    meta_pub = rospy.Publisher("/map2_meta", MapMetaData, queue_size=10)
    map_pub = rospy.Publisher("/map2", OccupancyGrid, queue_size=10)

    rate = rospy.Rate(50)
    map_dt = 5

    while not rospy.is_shutdown():
        
        if (Time.now().to_time() - og.info.map_load_time.to_time()) > map_dt:
            og.info.map_load_time = Time.now()
            _s = map.grid.shape
            og.info.width = _s[1]
            og.info.height = _s[0]
            o = map.map_to_origin()
            og.info.origin.position.x = -o["x"]
            og.info.origin.position.y = -o["y"]
            og.info.origin.orientation.w = 1.0
            

            og.data = map.grid.flatten()
            
            # print(og.info)
            meta_pub.publish(og.info)
            map_pub.publish(og)
        
                
        translation = (0.0, 0.0, 0.0)
        rotation = (0.0, 0.0, 0.0, 1.0)
    
        tfb.sendTransform(translation, rotation, Time.now(), 'odom', '/map2')
        

        rate.sleep()
