import rclpy
from rclpy.node import Node
import os, csv
from datetime import datetime
import serial
import threading
import time
from std_msgs.msg import Float32, String


class ESP32SerialBridge(Node):
    def __init__(self):
        super().__init__('esp32_serial_bridge')
        
        #Parametros CSV
        self.declare_parameter('csv_dir', os.path.expanduser('~/CSV_ROBOT'))
        self.declare_parameter('imu_base', 'csv_imu')          # prefijo archivo IMU
        self.declare_parameter('omega1_base', 'csv_omega1')    # prefijo archivo OMEGA1
        self.declare_parameter('omega2_base', 'csv_omega2')    # prefijo archivo OMEGA2
        self.declare_parameter('VelDes_base', 'csv_VelDes')    # prefijo archivo VELOCIDAD_DESEADA

        csv_dir      = self.get_parameter('csv_dir').value
        imu_base     = self.get_parameter('imu_base').value
        omega1_base  = self.get_parameter('omega1_base').value
        omega2_base  = self.get_parameter('omega2_base').value
        VelDes_base  = self.get_parameter('VelDes_base').value

        os.makedirs(csv_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # --- CSV IMU ---
        self.imu_path = os.path.join(csv_dir, f'{imu_base}_{ts}.csv')
        self.csv_imu = open(self.imu_path, 'w', newline='', buffering=1)
        self.writer_imu = csv.writer(self.csv_imu)
        self.writer_imu.writerow(['ros_time_s', 'imu_heading'])
        self.get_logger().info(f'IMU → {self.imu_path}')

        # --- CSV OMEGA1 ---
        self.omega1_path = os.path.join(csv_dir, f'{omega1_base}_{ts}.csv')
        self.csv_omega1 = open(self.omega1_path, 'w', newline='', buffering=1)
        self.writer_omega1 = csv.writer(self.csv_omega1)
        self.writer_omega1.writerow(['ros_time_s', 'omega_1'])
        self.get_logger().info(f'OMEGA1 → {self.omega1_path}')
        
        # --- CSV OMEGA2 ---
        self.omega2_path = os.path.join(csv_dir, f'{omega2_base}_{ts}.csv')
        self.csv_omega2 = open(self.omega2_path, 'w', newline='', buffering=1)
        self.writer_omega2 = csv.writer(self.csv_omega2)
        self.writer_omega2.writerow(['ros_time_s', 'omega_2'])
        self.get_logger().info(f'OMEGA2 → {self.omega2_path}')
        
        # --- CSV Velocidad deseada---
        self.VelDes_path = os.path.join(csv_dir, f'{VelDes_base}_{ts}.csv')
        self.csv_VelDes = open(self.VelDes_path, 'w', newline='', buffering=1)
        self.writer_VelDes = csv.writer(self.csv_VelDes)
        self.writer_VelDes.writerow(['ros_time_s', 'Vel_Des'])
        self.get_logger().info(f'Vel_Des → {self.VelDes_path}')
        
        # Parámetros configurables
        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud', 921600)  # más estable que 921600 en muchos cables

        self.port_name: str = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate: int = self.get_parameter('baud').get_parameter_value().integer_value

        self.serial_port = None
        self._open_serial()

        # Lock para escritura
        self._wlock = threading.Lock()

        # Subscripción a referencia "w1,w2"
        self.create_subscription(String, 'velocidad_deseada', self.write_serial_callback, 10)

        # Publicadores de telemetría
        self.omega1_pub = self.create_publisher(Float32, 'omega1', 10)
        self.omega2_pub = self.create_publisher(Float32, 'omega2', 10)
        self.imu_pub    = self.create_publisher(Float32,   'imu_heading', 10)
        self.imu=0.0

        # Hilo lector
        t = threading.Thread(target=self.serial_read_loop, daemon=True)
        t.start()

        self.get_logger().info(f"Bridge ESP32–ROS2 listo en {self.port_name} @ {self.baudrate}.")

    # ---------------- Serial helpers ----------------
    def _open_serial(self):
        while rclpy.ok():
            try:
                self.serial_port = serial.Serial(self.port_name, self.baudrate,
                                                 timeout=0.05, write_timeout=0.2)
                # Drenar basura de arranque (bootloader a 115200, etc.)
                self.serial_port.reset_input_buffer()
                t0 = time.time()
                while time.time() - t0 < 2.0:
                    self.serial_port.read(1024)
                self.get_logger().info("Puerto serial abierto y drenado.")
                return
            except Exception as e:
                self.get_logger().warn(f"No se pudo abrir {self.port_name}: {e}. Reintentando en 1s...")
                time.sleep(1.0)

    def _reopen_serial(self):
        try:
            if self.serial_port:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self._open_serial()
        except Exception as e:
            self.get_logger().error(f"Error reabriendo puerto: {e}")

    # ---------------- Callbacks ----------------
    def write_serial_callback(self, msg: String):
        data = (msg.data.strip() + '\n').encode('ascii', errors='ignore')
        if self.imu > 10.0 and self.imu<90.0:
            dato= "0.00,2.00"
            data = (dato + '\n').encode('ascii', errors='ignore')
            try:
                with self._wlock:
                    if self.serial_port and self.serial_port.writable():
                        self.serial_port.write(data)
                        # Opción: flush no bloqueante; normalmente no necesario con write_timeout
                        # self.serial_port.flush()
                self.get_logger().info(f"Enviado a ESP32: {dato}")
                if self.csv_VelDes and not self.csv_VelDes.closed:
                    t = self.get_clock().now().nanoseconds / 1e9
                    self.writer_VelDes.writerow([f'{t:.9f}', f'{(dato)}'])
            except Exception as e:
                self.get_logger().warn(f"Error enviando al ESP32: {e}. Reabriendo puerto...")
                self._reopen_serial()
        if self.imu < 350.0 and self.imu>270.0:
            dato= "2.00,0.00"
            data = (dato + '\n').encode('ascii', errors='ignore')
            try:
                with self._wlock:
                    if self.serial_port and self.serial_port.writable():
                        self.serial_port.write(data)
                        # Opción: flush no bloqueante; normalmente no necesario con write_timeout
                        # self.serial_port.flush()
                self.get_logger().info(f"Enviado a ESP32: {dato}")
                if self.csv_VelDes and not self.csv_VelDes.closed:
                    t = self.get_clock().now().nanoseconds / 1e9
                    self.writer_VelDes.writerow([f'{t:.9f}', f'{(dato)}'])
            except Exception as e:
                self.get_logger().warn(f"Error enviando al ESP32: {e}. Reabriendo puerto...")
                self._reopen_serial()
        else:
            try:
                with self._wlock:
                    if self.serial_port and self.serial_port.writable():
                        self.serial_port.write(data)
                        # Opción: flush no bloqueante; normalmente no necesario con write_timeout
                        # self.serial_port.flush()
                self.get_logger().info(f"Enviado a ESP32: {msg.data.strip()}")
                if self.csv_VelDes and not self.csv_VelDes.closed:
                    t = self.get_clock().now().nanoseconds / 1e9
                    self.writer_VelDes.writerow([f'{t:.9f}', f'{(msg.data.strip())}'])
            except Exception as e:
                self.get_logger().warn(f"Error enviando al ESP32: {e}. Reabriendo puerto...")
                self._reopen_serial()

    def serial_read_loop(self):
        while rclpy.ok():
            try:
                if not self.serial_port or not self.serial_port.readable():
                    self._reopen_serial()
                    continue

                raw = self.serial_port.read_until(b'\n')  # lee línea completa
                if not raw:
                    continue

                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                # Filtrar y publicar
                if line.startswith('>omega1:'):
                    try:
                        v = float(line.split(':', 1)[1])
                        if self.csv_omega1 and not self.csv_omega1.closed:
                            t = self.get_clock().now().nanoseconds / 1e9
                            self.writer_omega1.writerow([f'{t:.9f}', f'{float(v):.6f}'])
                        self.omega1_pub.publish(Float32(data=v))
                    except ValueError:
                        pass

                elif line.startswith('>omega2:'):
                    try:
                        v = float(line.split(':', 1)[1])
                        if self.csv_omega2 and not self.csv_omega2.closed:
                            t = self.get_clock().now().nanoseconds / 1e9
                            self.writer_omega2.writerow([f'{t:.9f}', f'{float(v):.6f}'])
                        self.omega2_pub.publish(Float32(data=v))
                    except ValueError:
                        pass

                elif line.startswith('>imu:'):
                    try:
                        v = float(line.split(':', 1)[1])
                        self.imu = v
                        if self.csv_imu and not self.csv_imu.closed:
                            t = self.get_clock().now().nanoseconds / 1e9
                            self.writer_imu.writerow([f'{t:.9f}', f'{float(v):.6f}'])
                        self.imu_pub.publish(Float32(data=v))
                    except ValueError:
                        pass

                # Si necesitas loguear de forma opcional:
                # self.get_logger().debug(f"RX: {line}")

            except serial.SerialException as e:
                self.get_logger().warn(f"SerialException: {e}. Reabriendo puerto...")
                time.sleep(0.2)
                self._reopen_serial()
            except Exception as e:
                # No lo truenes por bytes raros: sigue leyendo
                self.get_logger().warn(f"Error leyendo serial (ignorado): {e}")
                time.sleep(0.01)

    def destroy_node(self):
        try:
            if self.csv_imu and not self.csv_imu.closed:
                self.csv_imu.flush(); self.csv_imu.close()
            if self.csv_omega1 and not self.csv_omega1.closed:
                self.csv_omega1.flush(); self.csv_omega1.close()
            if self.csv_omega2 and not self.csv_omega2.closed:
                self.csv_omega2.flush(); self.csv_omega2.close()
            if self.csv_VelDes and not self.csv_VelDes.closed:
                self.csv_VelDes.flush(); self.csv_VelDes.close()
        except Exception as e:
            self.get_logger().warn(f'Error al cerrar archivos: {e}')
        super().destroy_node()
        
def main():
    rclpy.init()
    node = ESP32SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
