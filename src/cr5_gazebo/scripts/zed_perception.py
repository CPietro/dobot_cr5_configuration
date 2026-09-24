#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation
import sensor_msgs_py.point_cloud2 as pc2

class ZedPerceptionNode(Node):
    def __init__(self):
        super().__init__('zed_perception_node')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zed/camera/points',
            self.pointcloud_callback,
            10)

    def pointcloud_callback(self, msg):
        sensor_frame = msg.header.frame_id
        
        target_frame = 'world'

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                sensor_frame, 
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2)
            )
            
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            self.get_logger().info(f"La ZED si trova a X:{trans.x:.2f}, Y:{trans.y:.2f}, Z:{trans.z:.2f} rispetto al mondo.")

            punti_array = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            
            if len(punti_array) == 0:
                return 

            primo_punto_local = punti_array[0]

            p_local = np.array([float(primo_punto_local['x']), float(primo_punto_local['y']), float(primo_punto_local['z'])])
            r = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]) # scipy rotation

            matrice_rotazione = r.as_matrix()

            vettore_traslazione = np.array([trans.x, trans.y, trans.z])

            p_world = matrice_rotazione.dot(p_local) + vettore_traslazione

            self.get_logger().info("\n--- TEST SINGOLO PUNTO ---")
            self.get_logger().info(f"Visto dalla ZED:   X={p_local[0]:.3f}, Y={p_local[1]:.3f}, Z={p_local[2]:.3f}")
            self.get_logger().info(f"Posizione world:   X={p_world[0]:.3f}, Y={p_world[1]:.3f}, Z={p_world[2]:.3f}")
            self.get_logger().info("--------------------------\n")

            # TO DO
            pass

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Impossibile ottenere la TF tra {target_frame} e {sensor_frame}: {ex}')
            return

def main(args=None):
    rclpy.init(args=args)
    node = ZedPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()