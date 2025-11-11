#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Trigger


def qos_best_effort(depth=10):
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth
    )

def qos_reliable(depth=10):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth
    )

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_deg(err):
    # envuelve error de -180..+180
    while err > 180.0: err -= 360.0
    while err < -180.0: err += 360.0
    return err


class MissionPlannerNode(Node):
    """
    Secuencia fija con takeoff, ajuste de altura, avance, giro, avance final y aterrizaje.
    Durante los traslados mantiene retencion de altura, control lateral y sujecion de yaw.
    """

    def __init__(self):
        super().__init__("mission_planner")

        # ====== Parametros de trayectoria ======
        self.declare_parameter("target_height_m", 0.40)
        self.declare_parameter("forward_cm", 40)

        # ====== Movimiento base ======
        self.declare_parameter("move_speed_mps", 0.18)     # velocidad de referencia para cronometria
        self.declare_parameter("move_rc_scalar_xy", 0.20)  # proporcion de stick en eje x
        self.declare_parameter("pub_hz", 50.0)
        self.declare_parameter("ramp_prelude_s", 0.80)
        self.declare_parameter("segments", 2)              # numero de fragmentos por desplazamiento

        self.declare_parameter("yaw_rc_min", 0.10)           # stick minimo para romper zona muerta
        self.declare_parameter("yaw_slow_band_deg", 25.0)    # rango para atenuar el comando
        self.declare_parameter("yaw_timeout_s", 20.0)        # limite maximo del giro

        # Trim opcional por deriva sistematica en x
        self.declare_parameter("rc_xy_trim_x", 0.00)

        # ====== Control de altura ======
        self.declare_parameter("alt_tolerance_m", 0.02)
        self.declare_parameter("alt_slow_band_m", 0.15)
        self.declare_parameter("rc_z_fast", 0.10)
        self.declare_parameter("rc_z_slow", 0.05)
        self.declare_parameter("vgz_abs_max_cms", 2.0)
        self.declare_parameter("stability_window_s", 2.0)
        self.declare_parameter("alt_convergence_timeout_s", 25.0)  # reservado para compatibilidad
        self.declare_parameter("settle_after_takeoff_s", 2.0)
        self.declare_parameter("descend_timeout_s", 6.0)           # limite para el ajuste inicial

        # ====== Hold durante traslados ======
        self.declare_parameter("z_hold_kp_per_m", 0.30)         # rc porcentual por metro de error
        self.declare_parameter("z_hold_kd_per_mps", 0.20)        # rc porcentual por metro por segundo
        self.declare_parameter("z_hold_rc_max", 0.12)            # tope absoluto en eje z
        self.declare_parameter("y_damp_ky_per_cms", 0.004)       # factor para frenar deriva lateral
        self.declare_parameter("rc_y_max", 0.10)                 # tope lateral
        self.declare_parameter("yaw_hold_kp_per_deg", 0.035)      # rc porcentual por grado
        self.declare_parameter("rc_yaw_max", 0.4)               # tope de yaw

        # ====== Robustez ======
        self.declare_parameter("autoland_on_shutdown", True)
        self.declare_parameter("retry_backoff_s", 5.0)
        self.declare_parameter("takeoff_retries", 2)
        self.declare_parameter("takeoff_retry_delay_s", 1.0)
        self.declare_parameter("link_grace_s", 3.0)
        self.declare_parameter("land_retries", 3)
        self.declare_parameter("land_retry_delay_s", 1.0)
        self.declare_parameter("land_on_abort", True)
        self.declare_parameter("use_emergency_as_last_resort", False)
        self.declare_parameter("land_cooldown_s", 10.0)

        # Servicios remapeables
        self.declare_parameter("srv_ok_to_takeoff", "/battery_failsafe/ok_to_takeoff")
        self.declare_parameter("srv_sdk_command",  "/tello/sdk_command")
        self.declare_parameter("srv_takeoff",      "/tello/takeoff")
        self.declare_parameter("srv_land",         "/tello/land")

        # ====== Valores ======
        gp = self.get_parameter
        self.target_height = float(gp("target_height_m").value)
        self.forward_cm = int(gp("forward_cm").value)

        self.move_speed = max(0.05, float(gp("move_speed_mps").value))
        self.rc_xy = max(0.03, min(1.0, float(gp("move_rc_scalar_xy").value)))
        self.pub_hz = max(10.0, float(gp("pub_hz").value))
        self.ramp_prelude_s = max(0.0, float(gp("ramp_prelude_s").value))
        self.segments = max(1, int(gp("segments").value))
        self.rc_xy_trim_x = float(gp("rc_xy_trim_x").value)

        self.alt_tol = max(0.01, float(gp("alt_tolerance_m").value))
        self.alt_slow_band = max(self.alt_tol, float(gp("alt_slow_band_m").value))
        self.rc_z_fast = max(0.03, min(1.0, float(gp("rc_z_fast").value)))
        self.rc_z_slow = max(0.03, min(1.0, float(gp("rc_z_slow").value)))
        self.vgz_abs_max_cms = max(0.5, float(gp("vgz_abs_max_cms").value))
        self.stability_window_s = max(0.5, float(gp("stability_window_s").value))
        self.alt_conv_timeout = max(5.0, float(gp("alt_convergence_timeout_s").value))
        self.settle_after_takeoff_s = max(0.0, float(gp("settle_after_takeoff_s").value))
        self.descend_timeout_s = max(1.0, float(gp("descend_timeout_s").value))

        self.z_kp = float(gp("z_hold_kp_per_m").value)
        self.z_kd = float(gp("z_hold_kd_per_mps").value)
        self.z_rc_max = max(0.05, float(gp("z_hold_rc_max").value))

        self.y_ky = float(gp("y_damp_ky_per_cms").value)
        self.rc_y_max = max(0.05, float(gp("rc_y_max").value))

        self.yaw_kp = float(gp("yaw_hold_kp_per_deg").value)
        self.rc_yaw_max = max(0.05, float(gp("rc_yaw_max").value))
        self.yaw_rc_min = float(gp("yaw_rc_min").value)
        self.yaw_slow_band_deg = float(gp("yaw_slow_band_deg").value)
        self.yaw_timeout_s = float(gp("yaw_timeout_s").value)


        self.autoland_on_shutdown = bool(gp("autoland_on_shutdown").value)
        self.retry_backoff_s = float(gp("retry_backoff_s").value)
        self.takeoff_retries = max(0, int(gp("takeoff_retries").value))
        self.takeoff_retry_delay_s = max(0.0, float(gp("takeoff_retry_delay_s").value))
        self.link_grace_s = max(0.3, float(gp("link_grace_s").value))
        self.land_retries = max(0, int(gp("land_retries").value))
        self.land_retry_delay_s = max(0.0, float(gp("land_retry_delay_s").value))
        self.land_on_abort = bool(gp("land_on_abort").value)
        self.use_emergency_as_last_resort = bool(gp("use_emergency_as_last_resort").value)
        self.land_cooldown_s = max(0.0, float(gp("land_cooldown_s").value))

        # ====== Estado ======
        self._connected_raw = False
        self._last_connected_true_t = 0.0
        self._battery_critical = False
        self._stream_on = False
        self._height_m = 0.0
        self._vgx_cms = 0.0
        self._vgy_cms = 0.0
        self._vgz_cms = 0.0
        self._yaw_deg = 0.0
        self._mission_active = False
        self._permission_pending = False
        self._last_abort_t = 0.0
        self._last_land_t = 0.0
        self._in_air = False

        # ====== Interfaces ros ======
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_status = self.create_publisher(String, "/mission/status", latched_qos)
        self.pub_status.publish(String(data="idle"))

        self.pub_cmd_vel = self.create_publisher(Twist, "/tello/cmd_vel", qos_reliable(depth=10))

        self.create_subscription(Bool,   "/tello/connection_status", self._on_connection,      qos_reliable())
        self.create_subscription(Bool,   "/battery/critical",        self._on_battery_critical,qos_reliable())
        self.create_subscription(Int32,  "/tello/height",            self._on_height_int,      qos_best_effort())
        self.create_subscription(String, "/tello/stream/state",      self._on_stream_state_str,qos_best_effort())
        self.create_subscription(Vector3,"/tello/velocity",          self._on_velocity_vec,    qos_best_effort())
        self.create_subscription(Vector3,"/tello/attitude",          self._on_attitude_vec,    qos_best_effort())

        self.cli_ok_to_takeoff = self.create_client(Trigger, self.get_parameter("srv_ok_to_takeoff").get_parameter_value().string_value)
        self.cli_sdk_command  = self.create_client(Trigger, self.get_parameter("srv_sdk_command").get_parameter_value().string_value)
        self.cli_takeoff = self.create_client(Trigger, self.get_parameter("srv_takeoff").get_parameter_value().string_value)
        self.cli_land = self.create_client(Trigger, self.get_parameter("srv_land").get_parameter_value().string_value)
        self.cli_emergency = self.create_client(Trigger, "/tello/emergency")

        self.create_timer(1.0, self._check_start_conditions)
        rclpy.get_default_context().on_shutdown(self._on_shutdown)

    # ====== Funciones de callback ======
    def _on_connection(self, msg: Bool):
        self._connected_raw = bool(msg.data)
        if self._connected_raw:
            self._last_connected_true_t = time.time()

    def _is_connected(self) -> bool:
        return (time.time() - self._last_connected_true_t) <= self.link_grace_s

    def _on_battery_critical(self, msg: Bool):
        self._battery_critical = bool(msg.data)
        if self._battery_critical and self._mission_active:
            self._abort("battery_critical")

    def _on_height_int(self, msg: Int32):
        self._height_m = float(int(msg.data)) / 100.0
        self._in_air = (self._height_m > 0.2)

    def _on_velocity_vec(self, msg: Vector3):
        self._vgx_cms = float(msg.x)
        self._vgy_cms = float(msg.y)
        self._vgz_cms = float(msg.z)

    def _on_attitude_vec(self, msg: Vector3):
        self._yaw_deg = float(msg.z)

    def _on_stream_state_str(self, msg: String):
        txt = (msg.data or "").strip().lower()
        if "stream:off" in txt:
            self._stream_on = False
        elif "stream:on" in txt or "video_fps" in txt or "probe:ok" in txt:
            self._stream_on = True

    # ====== Arranque ======
    def _check_start_conditions(self):
        if self._mission_active or self._permission_pending:
            return
        now = time.time()
        if (now - self._last_land_t) < self.land_cooldown_s:
            self._publish_status("idle_cooldown_after_land"); return
        if (now - self._last_abort_t) < self.retry_backoff_s:
            return
        if self._in_air:
            self._publish_status("idle_in_air"); return
        if not self._is_connected():
            self._publish_status("idle_blocked_link"); return
        if not self._stream_on:
            self._publish_status("idle_blocked_stream_off"); return
        if self._battery_critical:
            self._publish_status("idle_blocked_battery_critical"); return

        if not self.cli_ok_to_takeoff.service_is_ready():
            self.cli_ok_to_takeoff.wait_for_service(timeout_sec=1.0)
        if not self.cli_ok_to_takeoff.service_is_ready():
            self._publish_status("idle_blocked_no_srv"); return

        self._permission_pending = True
        fut = self.cli_ok_to_takeoff.call_async(Trigger.Request())
        fut.add_done_callback(self._on_takeoff_permission)

    def _on_takeoff_permission(self, future):
        self._permission_pending = False
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"[mission_planner] Permiso de despegue: {res.message}")
                self._start_mission_thread()
            else:
                self.get_logger().warn(f"[mission_planner] Despegue bloqueado: {res.message}")
                self._publish_status("idle_blocked_low_battery")
                self._last_abort_t = time.time()
        except Exception as e:
            self.get_logger().warn(f"[mission_planner] Error consultando ok_to_takeoff: {e}")
            self._publish_status("idle_blocked_error")
            self._last_abort_t = time.time()

    # ====== Secuencia de vuelo ======
    def _start_mission_thread(self):
        self._mission_active = True
        threading.Thread(target=self._mission_sequence, daemon=True).start()

    def _mission_sequence(self):
        try:
            # Preparar sdk
            self._call_and_wait(self.cli_sdk_command, timeout=4.0)

            # Despegue con reintentos
            self._publish_status("takeoff")
            ok = False
            for i in range(1 + self.takeoff_retries):
                if self._call_and_wait(self.cli_takeoff):
                    ok = True; break
                self.get_logger().warn(f"[mission_planner] takeoff intento {i+1} fallo reintentando")
                time.sleep(self.takeoff_retry_delay_s)
            if not ok:
                self._abort("takeoff_failed"); return
            self._in_air = True

            # Ventana para amortiguar el tiron de despegue
            self._send_stop_for(self.settle_after_takeoff_s)

            # === Descenso hasta altura objetivo y espera de cinco segundos ===
            self._publish_status(f"descend_to_{int(self.target_height*100)}cm")
            self._descend_to_target_simple()
            self._pause_for(5.0, "hold_after_descend")

            # === Avance frontal y espera de cinco segundos ===
            if not self._move_x_with_holds(self.forward_cm, label="forward"):
                self._abort("forward_failed"); return
            self._pause_for(5.0, "pause_after_forward")

            # === Giro de ciento ochenta grados ===
            self._publish_status("yaw_180")
            if not self._turn_yaw_deg(180.0, timeout_s=8.0, tol_deg=5.0):
                self.get_logger().warn("[mission_planner] yaw 180 no entro en tolerancia continuo")

            # === Avance frontal de retorno y espera de cinco segundos ===
            if not self._move_x_with_holds(self.forward_cm, label="forward_return"):
                self._abort("forward_return_failed"); return
            self._pause_for(5.0, "pause_before_land")

            # Aterrizaje
            self._publish_status("landing")
            self._land_with_retries()
            self._publish_status("completed")

        finally:
            self._mission_active = False
            self._last_abort_t = time.time()
            if self._height_m < 0.2:
                self._in_air = False

    # ====== Descenso simple a target ======
    def _descend_to_target_simple(self):
        """
        Tras el despegue brusco del Tello, si quedo por encima del target,
        baja hasta aproximarse con un timeout. Si quedo por debajo no corrige.
        """
        dt = 1.0 / self.pub_hz
        t_end = time.time() + self.descend_timeout_s
        rc_down = self.rc_z_slow

        while time.time() < t_end:
            if self._check_abort_flags():
                return
            err = self.target_height - self._height_m
            if err >= -self.alt_tol:
                break  # objetivo alcanzado o por debajo
            t = Twist()
            t.linear.z = -rc_down
            if self.rc_xy_trim_x != 0.0:
                t.linear.x = self.rc_xy_trim_x
            self.pub_cmd_vel.publish(t)
            time.sleep(dt)

        self._send_stop_for(0.3)

    # ====== Giro en yaw por delta ======
    def _turn_yaw_deg(self, delta_deg: float, timeout_s: float = None, tol_deg: float = 5.0) -> bool:
        dt = 1.0 / self.pub_hz
        start_yaw = self._yaw_deg
        target = wrap_deg(start_yaw + delta_deg)
        t_end = time.time() + (timeout_s if timeout_s is not None else self.yaw_timeout_s)
        ok = False

        last_yaw = self._yaw_deg
        last_progress_t = time.time()

        while time.time() < t_end:
            if self._check_abort_flags():
                return False

            err = wrap_deg(target - self._yaw_deg)
            if abs(err) <= tol_deg:
                ok = True
                break

            # control proporcional con rampa cerca del objetivo
            rc_yaw = self.yaw_kp * err
            if abs(err) < self.yaw_slow_band_deg:
                scale = max(0.3, abs(err) / self.yaw_slow_band_deg)  # 0.3..1.0
                rc_yaw *= scale

            # nivel minimo para romper zona muerta
            mag = abs(rc_yaw)
            if mag > 0.0:
                mag = max(mag, self.yaw_rc_min)
            rc_yaw = math.copysign(mag, rc_yaw)

            # saturacion
            rc_yaw = clamp(rc_yaw, -self.rc_yaw_max, self.rc_yaw_max)

            # publicar comando
            tw = Twist()
            tw.angular.z = rc_yaw
            self.pub_cmd_vel.publish(tw)
            time.sleep(dt)

            # confirma que el giro progresa
            if abs(wrap_deg(self._yaw_deg - last_yaw)) >= 1.0:
                last_progress_t = time.time()
                last_yaw = self._yaw_deg
            elif (time.time() - last_progress_t) > 0.7:
                # refuerzo corto si no hay avance
                bump = 0.03
                rc_yaw_bumped = clamp(abs(rc_yaw) + bump, 0.0, self.rc_yaw_max)
                tw = Twist()
                tw.angular.z = math.copysign(rc_yaw_bumped, rc_yaw)
                self.pub_cmd_vel.publish(tw)
                time.sleep(dt)
                last_progress_t = time.time()

        self._send_stop_for(0.2)
        return ok



    # ====== Movimiento en eje x con controles z y yaw ======
    def _move_x_with_holds(self, cm: int, label: str, sign: int = +1) -> bool:
        if cm <= 0:
            return True

        dist_m = cm / 100.0
        total_duration = max(0.3, dist_m / self.move_speed)
        self._publish_status(f"{label}_{cm}")
        self.get_logger().info(f"[mission_planner] {label}_{cm} {dist_m:.2f} m @ ~{self.move_speed:.2f} m/s ({total_duration:.2f}s) rc={self.rc_xy:.2f}")

        # Detener antes de aplicar avance
        self._send_stop_for(0.25)

        # Rampa previa
        if self.ramp_prelude_s > 0.0:
            pre = Twist()
            pre.linear.x = self.rc_xy * sign * 0.6 + self.rc_xy_trim_x
            self._publish_rc_for_with_holds(pre, self.ramp_prelude_s, use_yaw_hold=True)

        # Dividir en segmentos para estabilizar
        segs = max(1, self.segments)
        seg_duration = total_duration / segs
        for _ in range(segs):
            # Capturar yaw de referencia al inicio de cada segmento
            yaw_ref = self._yaw_deg
            cmd = Twist()
            cmd.linear.x = self.rc_xy * sign + self.rc_xy_trim_x
            ok = self._publish_rc_for_with_holds(cmd, seg_duration, yaw_ref=yaw_ref, use_yaw_hold=True, use_alt_hold=True, use_y_damp=True)
            if not ok:
                self._send_stop_for(0.25)
                return False
            # Micro pausa entre segmentos
            self._send_stop_for(0.15)

        # Detener al terminar el avance
        self._send_stop_for(0.25)
        return True

    def _publish_rc_for_with_holds(self, base_twist: Twist, seconds: float,
                                   yaw_ref: float = None,
                                   use_yaw_hold: bool = False,
                                   use_alt_hold: bool = False,
                                   use_y_damp: bool = False) -> bool:
        dt = 1.0 / self.pub_hz
        t_end = time.time() + seconds
        while time.time() < t_end:
            if self._check_abort_flags():
                self._send_stop()
                return False

            t = Twist()
            # Base en eje x
            t.linear.x = base_twist.linear.x

            # ---- Control de altura Z ----
            if use_alt_hold:
                h_err = (self.target_height - self._height_m)             # m
                v_mps = self._vgz_cms / 100.0                             # m/s
                rc_z = self.z_kp * h_err + self.z_kd * (-v_mps)
                t.linear.z = clamp(rc_z, -self.z_rc_max, self.z_rc_max)

            # ---- Compensacion lateral por velocidad ----
            if use_y_damp:
                rc_y = - self.y_ky * self._vgy_cms
                t.linear.y = clamp(rc_y, -self.rc_y_max, self.rc_y_max)

            # ---- Control de yaw ----
            if use_yaw_hold and yaw_ref is not None:
                err_deg = wrap_deg(yaw_ref - self._yaw_deg)
                rc_yaw = self.yaw_kp * err_deg
                t.angular.z = clamp(rc_yaw, -self.rc_yaw_max, self.rc_yaw_max)

            self.pub_cmd_vel.publish(t)
            time.sleep(dt)
        return True

    # ====== Utilidades ======
    def _call_and_wait(self, client, timeout: float = 8.0) -> bool:
        try:
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=2.0)
            if not client.service_is_ready():
                self.get_logger().warn(f"Servicio no disponible: {client}")
                return False
            fut = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            res = fut.result()
            if not res:
                self.get_logger().warn(f"Timeout esperando respuesta del servicio {client}")
                return False
            self.get_logger().info(f"Comando -> success={res.success} msg='{res.message}'")
            return bool(res.success)
        except Exception as e:
            self.get_logger().warn(f"Error llamando servicio {client}: {e}")
            return False

    def _pause_for(self, seconds: float, msg="pause"):
        self.get_logger().info(f"[mission_planner] {msg} {seconds:.1f}s...")
        end = time.time() + max(0.0, seconds)
        while time.time() < end:
            if self._check_abort_flags():
                return
            time.sleep(0.1)

    def _check_abort_flags(self) -> bool:
        if not self._is_connected():
            self._abort("link_lost"); return True
        if self._battery_critical:
            self._abort("battery_critical"); return True
        return False

    def _abort(self, reason: str):
        self._publish_status(f"aborted_{reason}")
        self.get_logger().warn(f"[mission_planner] Mision abortada: {reason}")
        if self.land_on_abort and self._in_air:
            self.get_logger().warn("[mission_planner] abort: aterrizaje de cortesia")
            landed = self._land_with_retries()
            if not landed:
                threading.Thread(target=self._land_when_link_returns, daemon=True).start()
        self._mission_active = False
        self._last_abort_t = time.time()

    def _send_stop(self):
        self.pub_cmd_vel.publish(Twist())

    def _send_stop_for(self, seconds: float):
        dt = 1.0 / self.pub_hz
        t_end = time.time() + seconds
        zero = Twist()
        while time.time() < t_end:
            self.pub_cmd_vel.publish(zero)
            time.sleep(dt)

    def _land_with_retries(self) -> bool:
        self._send_stop_for(0.7)
        ok = False
        for i in range(1 + self.land_retries):
            if self._call_and_wait(self.cli_land, timeout=6.0):
                ok = True; break
            self.get_logger().warn(f"[mission_planner] land intento {i+1}/{1+self.land_retries} fallo reintentando")
            time.sleep(self.land_retry_delay_s)
        if not ok and self.use_emergency_as_last_resort and self.cli_emergency:
            self.get_logger().error("[mission_planner] land fallo usando EMERGENCY")
            try: self._call_and_wait(self.cli_emergency, timeout=3.0)
            except Exception: pass
        self._last_land_t = time.time()
        return ok

    def _land_when_link_returns(self):
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if self._is_connected():
                self.get_logger().warn("[mission_planner] link volvio intentando aterrizar")
                self._land_with_retries()
                return
            time.sleep(0.2)

    # ====== Cierre ======
    def _on_shutdown(self):
        try:
            if not self.autoland_on_shutdown: return
            self.get_logger().warn("[mission_planner] shutdown: ATERRIZAJE FORZADO...")
            self._land_with_retries()
        except Exception:
            pass

    def _publish_status(self, text: str):
        self.pub_status.publish(String(data=text))


def main():
    rclpy.init()
    node = MissionPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            if node.autoland_on_shutdown:
                node.get_logger().warn("[mission_planner] finalizacion: ATERRIZAJE FORZADO finalmente")
                node._land_with_retries()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

