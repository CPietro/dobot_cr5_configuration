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
            # Lookup TF usando Time() (Time 0 = ultima trasformazione disponibile)
            # per evitare problemi di estrapolazione o discrepanze temporali
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                sensor_frame, 
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2)
            )
            
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Ottimizzazione: invece di deserializzare tutti i 307.200 punti a 15 Hz,
            # leggiamo prima il pixel centrale tramite 'uvs'.
            if msg.height > 1 and msg.width > 1:
                center_idx = (msg.height // 2) * msg.width + (msg.width // 2)
            else:
                center_idx = 0

            p_local = None
            if hasattr(pc2, 'read_points_numpy'):
                # Accesso diretto a NumPy array (N, 3), evitando l'overhead dei structured array
                pts_center = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=False, uvs=[center_idx])
                if len(pts_center) > 0 and not np.isnan(pts_center[0]).any():
                    p_local = pts_center[0].astype(float)
                else:
                    # Se il punto centrale è NaN (es. fuori range/occluso), trova il primo punto valido
                    pts_all = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
                    if len(pts_all) > 0:
                        p_local = pts_all[0].astype(float)
            else:
                pts_center = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False, uvs=[center_idx])
                if len(pts_center) > 0 and not (np.isnan(pts_center[0]['x']) or np.isnan(pts_center[0]['y']) or np.isnan(pts_center[0]['z'])):
                    p_local = np.array([float(pts_center[0]['x']), float(pts_center[0]['y']), float(pts_center[0]['z'])])
                else:
                    punti_array = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                    if len(punti_array) > 0:
                        p_local = np.array([float(punti_array[0]['x']), float(punti_array[0]['y']), float(punti_array[0]['z'])])

            if p_local is None:
                return

            r = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]) # scipy rotation
            matrice_rotazione = r.as_matrix()
            vettore_traslazione = np.array([trans.x, trans.y, trans.z])
            p_world = matrice_rotazione.dot(p_local) + vettore_traslazione

            # Output consolidato con throttle per evitare il flood della console a 15 Hz
            log_msg = (
                f"La ZED si trova a X:{trans.x:.2f}, Y:{trans.y:.2f}, Z:{trans.z:.2f} rispetto al mondo.\n"
                f"--- TEST SINGOLO PUNTO ---\n"
                f"Visto dalla ZED:   X={p_local[0]:.3f}, Y={p_local[1]:.3f}, Z={p_local[2]:.3f}\n"
                f"Posizione world:   X={p_world[0]:.3f}, Y={p_world[1]:.3f}, Z={p_world[2]:.3f}\n"
                f"--------------------------"
            )
            self.get_logger().info(log_msg, throttle_duration_sec=1.0)

            # TO DO
            pass

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(
                f'Impossibile ottenere la TF tra {target_frame} e {sensor_frame}: {ex}',
                throttle_duration_sec=1.0
            )
            return

def main(args=None):
    rclpy.init(args=args)
    node = ZedPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
