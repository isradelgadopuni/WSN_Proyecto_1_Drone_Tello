# video_viewer.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, String, Int32MultiArray
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


def _ensure_runtime_dir():
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "/tmp/runtime-ros")
    try:
        os.makedirs(runtime_dir, exist_ok=True)
        os.chmod(runtime_dir, 0o700)
    except Exception:
        pass
    os.environ["XDG_RUNTIME_DIR"] = runtime_dir


def _opengl_cpu_safety():
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


class VideoViewer(Node):
    """Nodo de visualizacion que consume video JPEG y datos de deteccion."""

    def __init__(self):
        _opengl_cpu_safety()
        _ensure_runtime_dir()
        super().__init__('video_viewer')

        # Parametros de ventana y flujo de estados
        self.declare_parameter('status_topic', '/video_viewer/status')
        self.declare_parameter('status_rate_hz', 1.0)
        self.declare_parameter('frame_timeout_sec', 3.0)
        self.declare_parameter('window_name', 'Tello Camera Feed')
        self.declare_parameter('banner_size_hw', [480, 640])
        self.declare_parameter('display_fps_limit', 60.0)
        
        self.declare_parameter('boxes_timeout_sec', 0.6)  # tiempo sin boxes para limpiar overlay


        # Overlay
        self.declare_parameter('overlay_smooth_alpha', 0.6)
        self.declare_parameter('overlay_ttl_sec', 0.25)
        self.declare_parameter('overlay_fade', True)
        self.declare_parameter('overlay_thickness', 2)
        self.declare_parameter('color_red_bgr', [0, 0, 255])
        self.declare_parameter('color_black_bgr', [0, 0, 0])
        self.declare_parameter('show_debug_text', True)

        # Lectura de parametros declarados
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.status_period = 1.0 / max(0.1, float(self.get_parameter('status_rate_hz').value))
        self.frame_timeout = Duration(seconds=float(self.get_parameter('frame_timeout_sec').value))
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value
        banner_hw = self.get_parameter('banner_size_hw').get_parameter_value().integer_array_value
        self.banner_h = int(banner_hw[0] if len(banner_hw) > 0 else 480)
        self.banner_w = int(banner_hw[1] if len(banner_hw) > 1 else 640)
        self.display_min_period = 1.0 / max(1.0, float(self.get_parameter('display_fps_limit').value))

        self.overlay_alpha = float(self.get_parameter('overlay_smooth_alpha').value)
        self.overlay_ttl = float(self.get_parameter('overlay_ttl_sec').value)
        self.overlay_fade = bool(self.get_parameter('overlay_fade').value)
        self.overlay_thickness = int(self.get_parameter('overlay_thickness').value)
        cr = self.get_parameter('color_red_bgr').get_parameter_value().integer_array_value
        cb = self.get_parameter('color_black_bgr').get_parameter_value().integer_array_value
        self.color_red = (int(cr[0]), int(cr[1]), int(cr[2])) if len(cr) >= 3 else (0, 0, 255)
        self.color_black = (int(cb[0]), int(cb[1]), int(cb[2])) if len(cb) >= 3 else (0, 0, 0)
        self.show_debug_text = bool(self.get_parameter('show_debug_text').value)

        # Configuracion de perfiles QoS
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_status = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Creacion de publishers y suscripciones
        self.pub_status = self.create_publisher(String, self.status_topic, qos_status)
        self.bridge = CvBridge()

        # Suscripcion a video JPEG proveniente del driver
        self.sub_image_raw_comp = self.create_subscription(
            CompressedImage, '/tello/image_raw/compressed', self._cb_image_raw_comp, qos_img
        )

        # Suscripciones a cajas y conteos del detector
        self.sub_boxes_red = self.create_subscription(Int32MultiArray, '/objects/boxes_red', self._cb_boxes_red, qos_status)
        self.sub_boxes_black = self.create_subscription(Int32MultiArray, '/objects/boxes_black', self._cb_boxes_black, qos_status)
        self.sub_count_red = self.create_subscription(Int32, '/objects/red_count', self._cb_count_red, qos_status)
        self.sub_count_black = self.create_subscription(Int32, '/objects/black_count', self._cb_count_black, qos_status)
        
        self.boxes_timeout = float(self.get_parameter('boxes_timeout_sec').value)


        # Monitoreo de conexion y estado de stream
        self.sub_conn = self.create_subscription(Bool, '/tello/connection_status', self._cb_connection, qos_status)
        self.sub_stream = self.create_subscription(String, '/tello/stream/state', self._cb_stream_state, qos_img)

        # Buffers internos de imagen y banderas de estado
        self.connected = False
        self.last_stream_state_rx = None
        self.stream_summary = ""

        self._lock = threading.Lock()

        self.latest_raw = None           # (img, ts)
        self.latest_raw_ts: Time = Time()
        self.latest_raw_age_ms = None
        self.last_any_frame_rx = self.get_clock().now()

        # Seguimiento local por color
        self._tracks_red = []    # [{'sx','sy','sw','sh','last','vis'}]
        self._tracks_black = []
        self._last_boxes_rx = time.time()  

        # Conteos de objetivos por color
        self.count_red = 0
        self.count_black = 0

        self.current_status_str = "no_connection"

        # Lanzamiento del hilo de visualizacion
        self._stop_evt = threading.Event()
        self._viewer_thread = threading.Thread(target=self._viewer_loop, name='viewer_loop', daemon=True)
        self._viewer_thread.start()

        self.timer_status = self.create_timer(self.status_period, self._tick_status)

        try:
            cv2.setNumThreads(1)
        except Exception:
            pass

        self.get_logger().info("[video_viewer] listo (base=compressed, overlay boxes R/B)")

    # Callbacks de ROS para video y overlays
    def _img_stamp_or_now(self, msg):
        if getattr(msg, "header", None) and (msg.header.stamp.sec or msg.header.stamp.nanosec):
            return Time(seconds=float(msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9)
        return self.get_clock().now()

    def _cb_image_raw_comp(self, msg: CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='rgb8')
            ts = self._img_stamp_or_now(msg)
            with self._lock:
                self.latest_raw = (cv_img, ts)
                self.latest_raw_ts = ts
                self.latest_raw_age_ms = 0.0
                self.last_any_frame_rx = self.get_clock().now()
        except Exception as e:
            self.get_logger().warn(f"compressed decode: {e}")

    def _cb_boxes_red(self, msg: Int32MultiArray):
        boxes = []
        d = list(msg.data) if msg.data is not None else []
        for i in range(0, len(d), 4):
            try:
                x, y, w, h = int(d[i]), int(d[i+1]), int(d[i+2]), int(d[i+3])
                if w > 1 and h > 1:
                    boxes.append((x,y,w,h))
            except Exception:
                break
        now = time.time()
        with self._lock:
            self._update_tracks(self._tracks_red, boxes, now)
            self._last_boxes_rx = now


    def _cb_boxes_black(self, msg: Int32MultiArray):
        boxes = []
        d = list(msg.data) if msg.data is not None else []
        for i in range(0, len(d), 4):
            try:
                x, y, w, h = int(d[i]), int(d[i+1]), int(d[i+2]), int(d[i+3])
                if w > 1 and h > 1:
                    boxes.append((x,y,w,h))
            except Exception:
                break
        now = time.time()
        with self._lock:
            self._update_tracks(self._tracks_black, boxes, now)
            self._last_boxes_rx = now


    def _cb_count_red(self, msg: Int32):
        with self._lock:
            self.count_red = int(msg.data)

    def _cb_count_black(self, msg: Int32):
        with self._lock:
            self.count_black = int(msg.data)

    def _cb_connection(self, msg: Bool):
        self.connected = bool(msg.data)

    def _cb_stream_state(self, msg: String):
        self.last_stream_state_rx = time.time()
        s = msg.data.strip()
        self.stream_summary = s[:60] + ("…" if len(s) > 60 else "")

    # Suavizado de cajas entre actualizaciones
    def _update_tracks(self, tracks, boxes, now):
        alpha = max(0.0, min(1.0, float(self.get_parameter('overlay_smooth_alpha').value)))
        ttl = max(0.0, float(self.get_parameter('overlay_ttl_sec').value))
        fade = bool(self.get_parameter('overlay_fade').value)

        for t in tracks:
            t['updated'] = False

        def center(b): x,y,w,h=b; return (x+w*0.5, y+h*0.5)

        for b in boxes:
            cx, cy = center(b)
            best_i, best_d2 = -1, 1e18
            for i,t in enumerate(tracks):
                tcx, tcy = t['sx']+t['sw']*0.5, t['sy']+t['sh']*0.5
                d2 = (tcx-cx)**2 + (tcy-cy)**2
                if d2 < best_d2:
                    best_d2, best_i = d2, i
            if best_i >= 0 and best_d2 < 4000.0:
                t = tracks[best_i]
                t['sx'] = alpha*b[0] + (1-alpha)*t['sx']
                t['sy'] = alpha*b[1] + (1-alpha)*t['sy']
                t['sw'] = alpha*b[2] + (1-alpha)*t['sw']
                t['sh'] = alpha*b[3] + (1-alpha)*t['sh']
                t['last'] = now
                t['vis'] = 1.0
                t['updated'] = True
            else:
                tracks.append({'sx': float(b[0]), 'sy': float(b[1]),
                               'sw': float(b[2]), 'sh': float(b[3]),
                               'last': now, 'vis': 1.0, 'updated': True})

        keep = []
        for t in tracks:
            age = now - t['last']
            if age <= ttl:
                if fade:
                    t['vis'] = max(0.0, 1.0 - age/ttl)
                keep.append(t)
        tracks[:] = keep

    # Bucle principal de visualizacion
    def _viewer_loop(self):
        self.get_logger().info("Loop de video iniciado.")
        window_name = self.window_name
        last_display_t = 0.0

        while not self._stop_evt.is_set():
            now_mono = time.perf_counter()
            if (now_mono - last_display_t) < self.display_min_period:
                time.sleep(0.001); continue
            last_display_t = now_mono

            with self._lock:
                raw_tuple = self.latest_raw
                last_any = self.last_any_frame_rx
                now_ros = self.get_clock().now()
                now_sec = now_ros.nanoseconds * 1e-9
                if self.latest_raw_ts and (self.latest_raw_ts.nanoseconds > 0):
                    self.latest_raw_age_ms = max(0.0, (now_sec - (self.latest_raw_ts.nanoseconds*1e-9))*1000.0)

                tracks_red = [dict(t) for t in self._tracks_red]
                tracks_black = [dict(t) for t in self._tracks_black]
                cnt_r = self.count_red
                cnt_b = self.count_black
                last_boxes = self._last_boxes_rx
                
            # Reinicia overlays cuando se agota el tiempo sin cajas
            now_wall = time.time()
            if (now_wall - last_boxes) > self.boxes_timeout:
                # Limpieza de buffers locales usados en el frame actual
                tracks_red = []
                tracks_black = []
                cnt_r = 0
                cnt_b = 0
                # Limpieza del estado compartido para futuros frames
                with self._lock:
                    self._tracks_red.clear()
                    self._tracks_black.clear()
                    self.count_red = 0
                    self.count_black = 0



            if not self.connected:
                frame = self._make_banner((self.banner_h, self.banner_w), "Sin conexion")
                status_str = "no_connection"
            else:
                recent_any = (self.get_clock().now() - last_any) < self.frame_timeout
                if recent_any and raw_tuple is not None:
                    img, ts = raw_tuple
                    frame = img.copy()
                    status_str = "showing_video_compressed"
                elif recent_any:
                    frame = self._make_banner((self.banner_h, self.banner_w), "Sin senal de video")
                    status_str = "no_signal"
                else:
                    frame = self._make_banner((self.banner_h, self.banner_w), "Stream apagado")
                    status_str = "stream_off"

            # Dibujo de cajas detectadas
            if frame is not None and "showing_video" in status_str:
                self._draw_tracks(frame, tracks_red, self.color_red)
                self._draw_tracks(frame, tracks_black, self.color_black)

            # Overlay informativo con estado y conteos
            if frame is not None:
                self._draw_overlay(frame, status_str, cnt_r, cnt_b)
                try:
                    cv2.imshow(window_name, frame)
                except Exception as e:
                    self.get_logger().warn(f"imshow: {e}")

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                self.get_logger().info("Cerrando ventana de video por tecla 'q'.")
                self._stop_evt.set()
                break

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def _draw_tracks(self, frame, tracks, color):
        thickness_base = max(1, int(self.get_parameter('overlay_thickness').value))
        fade = bool(self.get_parameter('overlay_fade').value)
        for t in tracks:
            x = int(round(t['sx'])); y = int(round(t['sy']))
            w = int(round(t['sw'])); h = int(round(t['sh']))
            if w <= 1 or h <= 1: continue
            vis = float(t.get('vis', 1.0))
            th = thickness_base
            if fade and vis < 1.0:
                th = max(1, int(round(thickness_base * (0.5 + 0.5*vis))))
            try:
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, th)
            except Exception:
                pass

    # Funciones auxiliares para generar overlays
    def _make_banner(self, size_hw, text):
        h, w = size_hw
        img = np.zeros((h, w, 3), dtype=np.uint8)
        cv2.putText(img, text, (40, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2, cv2.LINE_AA)
        return img

    def _draw_overlay(self, frame, status_str, cnt_r, cnt_b):
        if not self.show_debug_text:
            return
        overlay = f"Conn:{self.connected} | R:{cnt_r} B:{cnt_b} | Status:{status_str}"
        cv2.putText(frame, overlay, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)
        raw_ms = f"{self.latest_raw_age_ms:.0f}ms" if self.latest_raw_age_ms is not None else "—"
        cv2.putText(frame, f"age video:{raw_ms}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1, cv2.LINE_AA)
        if self.last_stream_state_rx is not None:
            ago = time.time() - self.last_stream_state_rx
            hint = f"state[{ago:0.1f}s]: {self.stream_summary}"
            cv2.putText(frame, hint, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1, cv2.LINE_AA)

    # Temporizador que publica el estado del visor
    def _tick_status(self):
        recent = (self.get_clock().now() - self.last_any_frame_rx) < self.frame_timeout
        if not self.connected:
            s = "no_connection"
        elif not recent:
            s = "stream_off"
        else:
            with self._lock:
                has_video = self.latest_raw is not None
            s = "showing_video_compressed" if has_video else "no_signal"
        self.current_status_str = s
        try:
            self.pub_status.publish(String(data=s))
        except Exception:
            pass

    # Secuencia de apagado ordenada
    def destroy_node(self):
        self._stop_evt.set()
        try:
            if self._viewer_thread.is_alive():
                self._viewer_thread.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

