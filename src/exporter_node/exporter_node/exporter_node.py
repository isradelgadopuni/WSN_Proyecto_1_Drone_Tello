# ros2_tello_prom_exporter.py
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32, UInt8
from geometry_msgs.msg import Vector3
from prometheus_client import Gauge, start_http_server

# ===== Prometheus metrics =====
G_RED_CNT    = Gauge('objects_red_count',   'Red objects count from ROS',   ['drone_id'])
G_BLACK_CNT  = Gauge('objects_black_count', 'Black objects count from ROS', ['drone_id'])
G_BATTERY    = Gauge('tello_battery_percent', 'Tello battery level (percent)', ['drone_id'])
G_HEIGHT     = Gauge('tello_height_raw', 'Tello height (raw Int32 units from topic)', ['drone_id'])
G_VX         = Gauge('tello_velocity_x_mps', 'Tello velocity X (m/s)', ['drone_id'])
G_VY         = Gauge('tello_velocity_y_mps', 'Tello velocity Y (m/s)', ['drone_id'])
G_VZ         = Gauge('tello_velocity_z_mps', 'Tello velocity Z (m/s)', ['drone_id'])
G_VSPEED     = Gauge('tello_speed_norm_mps', 'Tello speed norm |v| (m/s)', ['drone_id'])

class ExporterNode(Node):
    def __init__(self):
        super().__init__('tello_prom_exporter')

        # --- params ---
        self.drone_id = self.declare_parameter('drone_id', 'tello_1') \
                            .get_parameter_value().string_value

        qos_mode = self.declare_parameter('qos_mode', 'best_effort') \
                            .get_parameter_value().string_value.lower()

        topic_red   = self.declare_parameter('topic_red_count',   '/objects/red_count') \
                            .get_parameter_value().string_value
        topic_black = self.declare_parameter('topic_black_count', '/objects/black_count') \
                            .get_parameter_value().string_value
        topic_batt  = self.declare_parameter('topic_battery',     '/tello/battery') \
                            .get_parameter_value().string_value
        topic_h     = self.declare_parameter('topic_height',      '/tello/height') \
                            .get_parameter_value().string_value
        topic_vel   = self.declare_parameter('topic_velocity',    '/tello/velocity') \
                            .get_parameter_value().string_value

        # --- QoS profile ---
        if qos_mode == 'reliable':
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        else:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        # --- metric samples with label ---
        self.red      = G_RED_CNT.labels(drone_id=self.drone_id);     self.red.set(float('nan'))
        self.black    = G_BLACK_CNT.labels(drone_id=self.drone_id);   self.black.set(float('nan'))
        self.battery  = G_BATTERY.labels(drone_id=self.drone_id);     self.battery.set(float('nan'))
        self.height   = G_HEIGHT.labels(drone_id=self.drone_id);      self.height.set(float('nan'))
        self.vx       = G_VX.labels(drone_id=self.drone_id);          self.vx.set(float('nan'))
        self.vy       = G_VY.labels(drone_id=self.drone_id);          self.vy.set(float('nan'))
        self.vz       = G_VZ.labels(drone_id=self.drone_id);          self.vz.set(float('nan'))
        self.vspeed   = G_VSPEED.labels(drone_id=self.drone_id);      self.vspeed.set(float('nan'))

        # --- subscriptions ---
        self.create_subscription(Int32, topic_red,   self.on_red,   qos)
        self.create_subscription(Int32, topic_black, self.on_black, qos)
        self.create_subscription(UInt8, topic_batt,  self.on_batt,  qos)
        self.create_subscription(Int32, topic_h,     self.on_h,     qos)
        self.create_subscription(Vector3, topic_vel, self.on_vel,   qos)

        self.get_logger().info(f"Exporter listo en :8000/metrics "
                               f"(QoS: {qos_mode.upper()}, drone_id={self.drone_id})")

    # --- callbacks ---
    def on_red(self, msg: Int32):
        self.red.set(float(msg.data))
    def on_black(self, msg: Int32):
        self.black.set(float(msg.data))
    def on_batt(self, msg: UInt8):
        self.battery.set(float(msg.data))
    def on_h(self, msg: Int32):
        # La unidad es la del tópico original (p.ej. cm si así lo publica tu nodo).
        self.height.set(float(msg.data))
    def on_vel(self, msg: Vector3):
        self.vx.set(float(msg.x))
        self.vy.set(float(msg.y))
        self.vz.set(float(msg.z))
        self.vspeed.set(math.sqrt(msg.x**2 + msg.y**2 + msg.z**2))

def main(args=None):
    rclpy.init(args=args)
    start_http_server(8000)
    node = ExporterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
