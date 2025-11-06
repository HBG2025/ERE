import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import os, csv
from datetime import datetime

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_fuzzy')
        
        #Parametros CSV
        self.declare_parameter('csv_dir', os.path.expanduser('~/CSV_ROBOT'))
        self.declare_parameter('ErrorDist_base', 'csv_ErrorDist')          # prefijo archivo ERROR DISTANCIA

        csv_dir      = self.get_parameter('csv_dir').value
        ErrorDist_base     = self.get_parameter('ErrorDist_base').value

        os.makedirs(csv_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # --- ERROR DE DISTANCIA CSV ---
        self.ErrorDist_path = os.path.join(csv_dir, f'{ErrorDist_base}_{ts}.csv')
        self.csv_ErrorDist = open(self.ErrorDist_path, 'w', newline='', buffering=1)
        self.writer_ErrorDist = csv.writer(self.csv_ErrorDist)
        self.writer_ErrorDist.writerow(['ros_time_s', 'Error_Distancia'])
        self.get_logger().info(f'Error_Distancia → {self.ErrorDist_path}')

        self.subscription = self.create_subscription(Float32, 'lidarLeft', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String, 'velocidad_deseada', 10)
        self.setpoint = 0.5 # metros

        self._crear_controlador_difuso()

    def _crear_controlador_difuso(self):
        error = ctrl.Antecedent(np.arange(-1, 0.401, 0.001), 'error')
        wl = ctrl.Consequent(np.arange(0.5, 3.001, 0.001), 'w_ld')
        wr = ctrl.Consequent(np.arange(0.5, 3.001, 0.001), 'w_rd')

        # Definición de funciones de pertenencia (igual que antes)
        error['NL'] = fuzz.trimf(error.universe, [-2, -0.01, -0.005])
        error['NS'] = fuzz.trimf(error.universe, [-0.01, -0.005, 0.0])
        error['Z']  = fuzz.trimf(error.universe, [-0.005, 0.0, 0.005])
        error['PS'] = fuzz.trimf(error.universe, [0.0, 0.005, 0.01])
        error['PL'] = fuzz.trimf(error.universe, [0.005, 0.01, 2])

        # Conjuntos con solapes suaves
        wl['slow']   = fuzz.trimf(wl.universe, [0.5, 0.5, 1.0])
        wl['medium'] = fuzz.trimf(wl.universe, [0.9, 2.0, 2.9])
        wl['fast']   = fuzz.trimf(wl.universe, [2.5, 3.0, 3.0])

        wr['slow']   = fuzz.trimf(wr.universe, [0.5, 0.5, 1.0])
        wr['medium'] = fuzz.trimf(wr.universe, [0.9, 2.0, 2.9])
        wr['fast']   = fuzz.trimf(wr.universe, [2.5, 3.0, 3.0])

        rules = [
            ctrl.Rule(error['NL'], consequent=[wl['fast'], wr['slow']]),
            ctrl.Rule(error['NS'], consequent=[wl['medium'], wr['slow']]),
            ctrl.Rule(error['Z'],  consequent=[wl['medium'], wr['medium']]),
            ctrl.Rule(error['PS'], consequent=[wl['slow'], wr['medium']]),
            ctrl.Rule(error['PL'], consequent=[wl['slow'], wr['fast']]),
        ]

        sistema_ctrl = ctrl.ControlSystem(rules)
        self.simulador = ctrl.ControlSystemSimulation(sistema_ctrl)

        # Guardamos referencia a variables para uso en callback
        self._error_var = error
        self._wl_var = wl
        self._wr_var = wr

    def listener_callback(self, msg: Float32):
        distancia = msg.data
        error_dist = self.setpoint - distancia

        self.simulador.input['error'] = error_dist
        self.simulador.compute()

        w_ld = self.simulador.output['w_ld']
        w_rd = self.simulador.output['w_rd']

        # calcular membresía del error en cada etiqueta
        m = self._error_var
        memberships = {label: fuzz.interp_membership(m.universe, m[label].mf, error_dist)
                       for label in ['NL','NS','Z','PS','PL']}

        self.get_logger().info(
            f"Error: {error_dist:.3f} m | Membresías – " +
            ", ".join(f"{lbl}:{val:.2f}" for lbl, val in memberships.items()) +
            f" | Velocidades → w_ld: {w_ld:.2f}, w_rd: {w_rd:.2f}"
        )
        if self.csv_ErrorDist and not self.csv_ErrorDist.closed:
            t = self.get_clock().now().nanoseconds / 1e9
            self.writer_ErrorDist.writerow([f'{t:.9f}', f'{float(error_dist):.6f}'])

        msg_out = String()
        msg_out.data = f"{w_ld:.2f},{w_rd:.2f}"
        self.publisher_.publish(msg_out)

    def destroy_node(self):
        try:
            if self.csv_ErrorDist and not self.csv_ErrorDist.closed:
                self.csv_ErrorDist.flush(); self.csv_ErrorDist.close()
        except Exception as e:
            self.get_logger().warn(f'Error al cerrar archivos: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()