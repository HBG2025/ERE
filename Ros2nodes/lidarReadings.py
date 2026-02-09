#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import ydlidar
import os, csv
from datetime import datetime


qos_profile=QoSProfile(depth=10)
qos_profile.reliability=QoSReliabilityPolicy.BEST_EFFORT
# --- Init LiDAR global (como tenías) ---
ydlidar.os_init()
ports = ydlidar.lidarPortList()
port = next(iter(ports.values()), "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
laser.initialize()
laser.turnOn()

def wrap_to_pi(x: float) -> float:
    """Normaliza a (-π, π]."""
    return math.atan2(math.sin(x), math.cos(x))

class LidarLeftWithIMU(Node):
    def __init__(self):
        super().__init__('lidar_left_and_imu')

        #Parametros CSV
        self.declare_parameter('csv_dir', os.path.expanduser('~/CSV_ROBOT'))
        self.declare_parameter('LidarLeft_base', 'csv_LidarLeft')          # prefijo archivo DISTANCIA
        self.declare_parameter('ImuError_base', 'csv_ImuError')    # prefijo archivo IMU ERROR YAW
        self.declare_parameter('LecVal_base', 'csv_LecVal')    # prefijo archivo LECTURAS VALIDAS

        csv_dir      = self.get_parameter('csv_dir').value
        LidarLeft_base     = self.get_parameter('LidarLeft_base').value
        ImuError_base  = self.get_parameter('ImuError_base').value
        LecVal_base  = self.get_parameter('LecVal_base').value

        os.makedirs(csv_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # --- CSV DISTANCIA ---
        self.LidarLeft_path = os.path.join(csv_dir, f'{LidarLeft_base}_{ts}.csv')
        self.csv_LidarLeft = open(self.LidarLeft_path, 'w', newline='', buffering=1)
        self.writer_LidarLeft = csv.writer(self.csv_LidarLeft)
        self.writer_LidarLeft.writerow(['ros_time_s', 'Lidar_Left'])
        self.get_logger().info(f'LidarLeft → {self.LidarLeft_path}')

        # --- CSV IMU_ERROR_YAW ---
        self.ImuError_path = os.path.join(csv_dir, f'{ImuError_base}_{ts}.csv')
        self.csv_ImuError = open(self.ImuError_path, 'w', newline='', buffering=1)
        self.writer_ImuError = csv.writer(self.csv_ImuError)
        self.writer_ImuError.writerow(['ros_time_s', 'Imu_Error_yaw'])
        self.get_logger().info(f'Imu_Error_yaw → {self.ImuError_path}')
        
        # --- CSV LECTURAS VALIDAS ---
        self.LecVal_path = os.path.join(csv_dir, f'{LecVal_base}_{ts}.csv')
        self.csv_LecVal = open(self.LecVal_path, 'w', newline='', buffering=1)
        self.writer_LecVal = csv.writer(self.csv_LecVal)
        self.writer_LecVal.writerow(['ros_time_s', 'Lecturas_Validas'])
        self.get_logger().info(f'Lecturas_Validas → {self.LecVal_path}')
        
        # ── Parámetros configurables ─────────────────────────────────────────────
        self.declare_parameter('aperture_deg', 160.0)    # ancho total del cono (grados)
        self.declare_parameter('max_range', 8.0)         # m (rango LiDAR válido)
        self.declare_parameter('use_projection', True)   # usar proyección escalar
        self.declare_parameter('max_proj', 2.0)       # m (umbral de proyección) ANTES 0.85
        # ------------------------------------------------------------------------

        # IMU (radianes)
        self.initial_imu_angle = None
        self.imu_angle = 0.0
        self.eYaw = 0.0   # = initial - current   (≈ -θ)

        # Publishers
        self.pub_left = self.create_publisher(Float32, 'lidarLeft', 10)
        self.pub_scan = self.create_publisher(LaserScan, 'scan', qos_profile)
        self.pub_error_yaw = self.create_publisher(Float32, 'imu_error_yaw', 10)

        # Subscriber (guarda handle + QoS sensores)
        self.sub_imu = self.create_subscription(
            Float32, 'imu_heading', self.callback_imu, qos_profile_sensor_data
        )

        # Timers
        self.create_timer(0.10, self.process_scan)

        # Cache de parámetros
        self._update_params()

        # Watchdog básico
        self.last_imu_msg = self.get_clock().now()
        self.create_timer(1.0, self._watchdog)

    def _update_params(self):
        ap_deg = float(self.get_parameter('aperture_deg').value)
        self.conic_aperture = math.radians(ap_deg) / 2.0   # ±ap/2
        self.max_range = float(self.get_parameter('max_range').value)
        self.use_projection = bool(self.get_parameter('use_projection').value)
        self.max_proj = float(self.get_parameter('max_proj').value)

    def callback_imu(self, msg: Float32):
        # IMU llega en GRADOS → convertir a radianes
        current_rad = math.radians(float(msg.data))
        current_rad = wrap_to_pi(current_rad)

        if self.initial_imu_angle is None:
            self.initial_imu_angle = current_rad
            self.get_logger().info(f"📌 IMU referencia inicial: {self.initial_imu_angle:.3f} rad")

        self.imu_angle = current_rad
        # eYaw = initial - current  ⇒ si yaw = +θ (CCW), eYaw = -θ
        self.eYaw = wrap_to_pi(self.initial_imu_angle - self.imu_angle)

        self.pub_error_yaw.publish(Float32(data=self.eYaw))
        self.last_imu_msg = self.get_clock().now()
        # self.get_logger().debug(f"IMU: {self.imu_angle:.3f} rad | eYaw={self.eYaw:.3f}")

    def process_scan(self):
        scan = ydlidar.LaserScan()
        ok = laser.doProcessSimple(scan)
        if not ok or len(scan.points) == 0:
            self.get_logger().warn("LiDAR scan failed or empty")
            return

        # Centro del cono para la IZQUIERDA del robot:
        # base = -π/2 y compensamos orientación del robot (resta θ ⇒ suma eYaw)
        # con eYaw = -θ, entonces: center = -π/2 + eYaw = -π/2 - θ
        conic_center = wrap_to_pi(-math.pi/2 + self.eYaw)

        sum_val, count = 0.0, 0

        for pt in scan.points:
            a = pt.angle
            # Normaliza ángulo del LiDAR a (-π, π]
            if a > math.pi:
                a -= 2.0 * math.pi

            # Dentro del cono angular
            if (conic_center - self.conic_aperture) <= a <= (conic_center + self.conic_aperture):
                if 0.0 < pt.range <= self.max_range:
                    if self.use_projection:
                        # Proyección escalar sobre la normal del cono
                        angle_diff = a - conic_center
                        proj = pt.range * math.cos(angle_diff)
                        if 0.0 < proj <= self.max_proj:
                            sum_val += proj
                            count += 1
                    else:
                        sum_val += pt.range
                        count += 1

        if count > 0:
            avg = sum_val / count
            self.pub_left.publish(Float32(data=avg))
            self.get_logger().info(
                f"📏 Dist. lateral: {avg:.3f} m (validas={count}, centro={conic_center:.2f} rad, eYaw={self.eYaw:.2f})"
            )
            if self.csv_LidarLeft and not self.csv_LidarLeft.closed:
                t = self.get_clock().now().nanoseconds / 1e9
                self.writer_LidarLeft.writerow([f'{t:.9f}', f'{float(avg):.6f}'])
            if self.csv_ImuError and not self.csv_ImuError.closed:
                t = self.get_clock().now().nanoseconds / 1e9
                self.writer_ImuError.writerow([f'{t:.9f}', f'{float(self.eYaw):.6f}'])
            if self.csv_LecVal and not self.csv_LecVal.closed:
                t = self.get_clock().now().nanoseconds / 1e9
                self.writer_LecVal.writerow([f'{t:.9f}', f'{float(count):.6f}'])
        else:
            self.get_logger().warn("No se encontraron lecturas válidas en el cono.")
        if ok and len(scan.points) > 0:
            # Mensaje LaserScan
            msg = LaserScan()
            now = self.get_clock().now().to_msg()
            msg.header.stamp = now
            msg.header.frame_id = "laser_frame"   # importante para RViz
            msg.angle_min = -math.pi
            msg.angle_max = math.pi
            msg.angle_increment = (msg.angle_max - msg.angle_min) / len(scan.points)
            msg.time_increment = 0.0
            msg.scan_time = 0.05  # aprox (10 Hz)
            msg.range_min = 0.05
            msg.range_max = self.max_range

            ranges = []
            for pt in scan.points:
                r = pt.range if pt.range > 0.0 else float('inf')
                ranges.append(r)
            msg.ranges = ranges

            # publicar
            self.pub_scan.publish(msg)

        
    def _watchdog(self):
        if (self.get_clock().now() - self.last_imu_msg).nanoseconds > 2e9:
            self.get_logger().warn("⏱️ No llegan mensajes en /imu_heading desde hace >2 s.")

    def destroy_node(self):
        try:
            if self.csv_LidarLeft and not self.csv_LidarLeft.closed:
                self.csv_LidarLeft.flush(); self.csv_LidarLeft.close()
            if self.csv_ImuError and not self.csv_ImuError.closed:
                self.csv_ImuError.flush(); self.csv_ImuError.close()
            if self.csv_LecVal and not self.csv_LecVal.closed:
                self.csv_LecVal.flush(); self.csv_LecVal.close()
        except Exception as e:
            self.get_logger().warn(f'Error al cerrar archivos: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarLeftWithIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    laser.turnOff()
    laser.disconnecting()

if __name__ == '__main__':
    main()
