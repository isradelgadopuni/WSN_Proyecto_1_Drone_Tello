# object_detector.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
from typing import Optional, Tuple, List

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Int32, Int32MultiArray, MultiArrayDimension, Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


def _opengl_cpu_safety():
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "/tmp/runtime-ros")
    try:
        os.makedirs(runtime_dir, exist_ok=True)
        os.chmod(runtime_dir, 0o700)
    except Exception:
        pass
    os.environ["XDG_RUNTIME_DIR"] = runtime_dir


class ObjectDetector(Node):
    """Nodo de deteccion HSV que genera cajas y conteos para el visor."""

    def __init__(self):
        _opengl_cpu_safety()
        super().__init__('object_detector')

        # Parametros de configuracion general
        self.declare_parameter('stream_timeout_sec', 3.0)
        self.declare_parameter('frame_timeout_sec', 3.0)
        self.declare_parameter('max_fps', 30.0)
        self.declare_parameter('verbose', True)

        self.declare_parameter('publish_annotated', False)   # OFF: viewer dibuja boxes
        self.declare_parameter('display_overlay', True)      # solo afecta imagen anotada
        self.declare_parameter('input_color_order', 'rgb')

        self.declare_parameter('blur_ksize', 3)
        self.declare_parameter('morph_open_ksize', 5)
        self.declare_parameter('morph_close_ksize', 9)
        self.declare_parameter('open_iters', 1)
        self.declare_parameter('close_iters', 2)
        self.declare_parameter('min_area_px', 2000)

        # Rangos y robustez
        self.declare_parameter('lower_red1', [0, 100, 80])
        self.declare_parameter('upper_red1', [10, 255, 255])
        self.declare_parameter('lower_red2', [170, 100, 80])
        self.declare_parameter('upper_red2', [180, 255, 255])
        self.declare_parameter('lower_black', [0, 0, 0])
        self.declare_parameter('upper_black', [180, 60, 60])

        self.declare_parameter('red_h1_lo', 0)
        self.declare_parameter('red_h1_hi', 8)
        self.declare_parameter('red_h2_lo', 174)
        self.declare_parameter('red_h2_hi', 180)

        self.declare_parameter('exclude_orange_lo', 5)
        self.declare_parameter('exclude_orange_hi', 35)

        self.declare_parameter('black_excl_blue_lo', 90)
        self.declare_parameter('black_excl_blue_hi', 140)

        self.declare_parameter('use_clahe', True)
        self.declare_parameter('clahe_clip_limit', 2.0)
        self.declare_parameter('clahe_tile_grid', [8, 8])

        self.declare_parameter('red_s_min', 90)
        self.declare_parameter('red_v_min', 60)
        self.declare_parameter('black_s_max', 65)
        self.declare_parameter('black_v_max', 60)

        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.display_overlay = bool(self.get_parameter('display_overlay').value)

        self.input_color_order = self.get_parameter('input_color_order').get_parameter_value().string_value.strip().lower()
        if self.input_color_order not in ('bgr', 'rgb'):
            self.input_color_order = 'bgr'

        # Definicion de perfiles QoS para imagen y estado
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_status = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Creacion de publishers y suscripciones
        self.bridge = CvBridge()

        self.sub_image_in = self.create_subscription(
            CompressedImage,
            '/tello/image_raw/compressed',
            self._cb_image_comp,
            qos_img,
        )
        self.get_logger().info("[object_detector] usando /tello/image_raw/compressed (JPEG)")

        self.sub_conn = self.create_subscription(Bool, '/tello/connection_status', self._cb_connection, qos_status)
        self.sub_stream = self.create_subscription(String, '/tello/stream/state', self._cb_stream_state, qos_img)

        # Publicadores de cajas y conteos separados por color
        self.pub_boxes_red   = self.create_publisher(Int32MultiArray, '/objects/boxes_red', qos_status)
        self.pub_boxes_black = self.create_publisher(Int32MultiArray, '/objects/boxes_black', qos_status)
        self.pub_count_red   = self.create_publisher(Int32, '/objects/red_count', qos_status)
        self.pub_count_black = self.create_publisher(Int32, '/objects/black_count', qos_status)

        # Publicador de compatibilidad con total combinado
        self.pub_count_total = self.create_publisher(Int32, '/objects/red_black_count', qos_status)

        # Publicador opcional de imagen anotada
        self.pub_annot = None
        if self.publish_annotated:
            self.pub_annot = self.create_publisher(Image, '/tello/image_annotated', qos_img)

        # Variables de estado para control de tiempos
        self.connected: bool = False
        self.last_stream_heartbeat: Optional[float] = None
        self.stream_timeout = float(self.get_parameter('stream_timeout_sec').value)
        self.frame_timeout_sec = float(self.get_parameter('frame_timeout_sec').value)
        self.max_fps = float(self.get_parameter('max_fps').value)
        self.min_period = 1.0 / max(1e-3, self.max_fps)
        self.verbose = bool(self.get_parameter('verbose').value)

        # Throttle de logs
        self._last_log_times = {}
        self._log_default_period = 2.0

        # HSV/morf
        self.blur_ksize = int(self.get_parameter('blur_ksize').value)
        self.morph_open_ksize = int(self.get_parameter('morph_open_ksize').value)
        self.morph_close_ksize = int(self.get_parameter('morph_close_ksize').value)
        self.open_iters = int(self.get_parameter('open_iters').value)
        self.close_iters = int(self.get_parameter('close_iters').value)
        self.min_area_px = int(self.get_parameter('min_area_px').value)

        self.use_clahe = bool(self.get_parameter('use_clahe').value)
        self.clahe_clip = float(self.get_parameter('clahe_clip_limit').value)
        tg = self.get_parameter('clahe_tile_grid').get_parameter_value().integer_array_value
        self.clahe_tile = (int(tg[1] if len(tg) > 1 else 8), int(tg[0] if len(tg) > 0 else 8))

        red_s_min = int(self.get_parameter('red_s_min').value)
        red_v_min = int(self.get_parameter('red_v_min').value)
        black_s_max = int(self.get_parameter('black_s_max').value)
        black_v_max = int(self.get_parameter('black_v_max').value)

        red_h1_lo = int(self.get_parameter('red_h1_lo').value)
        red_h1_hi = int(self.get_parameter('red_h1_hi').value)
        red_h2_lo = int(self.get_parameter('red_h2_lo').value)
        red_h2_hi = int(self.get_parameter('red_h2_hi').value)

        exclude_orange_lo = int(self.get_parameter('exclude_orange_lo').value)
        exclude_orange_hi = int(self.get_parameter('exclude_orange_hi').value)
        blue_lo = int(self.get_parameter('black_excl_blue_lo').value)
        blue_hi = int(self.get_parameter('black_excl_blue_hi').value)

        # Rangos
        self.lower_red1  = np.array([red_h1_lo, red_s_min, red_v_min], dtype=np.uint8)
        self.upper_red1  = np.array([red_h1_hi, 255,      255],       dtype=np.uint8)
        self.lower_red2  = np.array([red_h2_lo, red_s_min, red_v_min], dtype=np.uint8)
        self.upper_red2  = np.array([red_h2_hi, 255,       255],       dtype=np.uint8)
        self.lower_black = np.array([0,   0,          0],       dtype=np.uint8)
        self.upper_black = np.array([180, black_s_max, black_v_max], dtype=np.uint8)
        self.orange_lo = np.array([exclude_orange_lo, 0,   0],   dtype=np.uint8)
        self.orange_hi = np.array([exclude_orange_hi, 255, 255], dtype=np.uint8)
        self.blue_lo = np.array([blue_lo, 0, 0], dtype=np.uint8)
        self.blue_hi = np.array([blue_hi, 255, 255], dtype=np.uint8)

        # Buffer frame
        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_header = None
        self._latest_stamp_sec: float = 0.0
        self._last_frame_rx_sec: float = 0.0

        # Lanzamiento del hilo de procesamiento
        self._stop_evt = threading.Event()
        self._worker = threading.Thread(target=self._process_loop, name='object_detector_loop', daemon=True)
        self._worker.start()

        try:
            cv2.setNumThreads(1)
        except Exception:
            pass

        self._publish_status("idle")
        self.get_logger().info(
            f"[object_detector] listo | max_fps={self.max_fps:.1f} | frame_timeout={self.frame_timeout_sec:.1f}s | "
            f"stream_timeout={self.stream_timeout:.1f}s | publish_annotated={self.publish_annotated}"
        )

    # Utilidades para limitar los mensajes de log
    def _log_throttled(self, level: str, key: str, msg: str, period: float = None):
        period = self._log_default_period if period is None else period
        now = time.time()
        last = self._last_log_times.get(key, 0.0)
        if (now - last) >= period:
            self._last_log_times[key] = now
            try:
                getattr(self.get_logger(), level)(msg)
            except Exception:
                self.get_logger().info(msg)

    # Callbacks de ROS para recibir datos
    def _cb_image_comp(self, msg: CompressedImage):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge compressed: {e}")
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp = getattr(msg, "header", None).stamp if getattr(msg, "header", None) else None
        stamp_sec = (float(stamp.sec) + stamp.nanosec * 1e-9) if (stamp and (stamp.sec or stamp.nanosec)) else 0.0
        with self._lock:
            self._latest_frame = img
            if getattr(msg, "header", None):
                self._latest_header = msg.header
            else:
                h = Header()
                h.stamp = self.get_clock().now().to_msg()
                h.frame_id = "tello_camera"
                self._latest_header = h
            self._latest_stamp_sec = stamp_sec
            self._last_frame_rx_sec = now_sec

    def _cb_connection(self, msg: Bool):
        self.connected = bool(msg.data)

    def _cb_stream_state(self, msg: String):
        self.last_stream_heartbeat = time.time()

    # Bucle principal de procesamiento de imagenes
    def _process_loop(self):
        last_pub_t = 0.0
        while not self._stop_evt.is_set() and rclpy.ok():
            now = time.perf_counter()
            if (now - last_pub_t) < self.min_period:
                time.sleep(0.001)
                continue
            last_pub_t = now

            if not self.connected:
                self._publish_status_once("no_connection")
                time.sleep(0.01)
                continue

            if (self.last_stream_heartbeat is None) or ((time.time() - self.last_stream_heartbeat) > self.stream_timeout):
                self._publish_status_once("stream_off")
                time.sleep(0.01)
                continue

            with self._lock:
                frame = self._latest_frame.copy() if self._latest_frame is not None else None
                header = self._latest_header
                latest_stamp_sec = self._latest_stamp_sec
                last_rx_sec = self._last_frame_rx_sec

            if frame is None or header is None:
                self._publish_status_once("no_signal")
                time.sleep(0.005)
                continue

            now_sec = self.get_clock().now().nanoseconds * 1e-9
            ref_sec = latest_stamp_sec if latest_stamp_sec > 0.0 else last_rx_sec
            if (now_sec - ref_sec) >= self.frame_timeout_sec:
                self._publish_status_once("no_signal")
                time.sleep(0.005)
                continue

            # Etapa de deteccion por color
            n_red, n_black, boxes_red, boxes_black, annotated = self._detect(frame)

            # Publicar conteos
            try:
                self.pub_count_red.publish(Int32(data=int(n_red)))
                self.pub_count_black.publish(Int32(data=int(n_black)))
                self.pub_count_total.publish(Int32(data=int(n_red + n_black)))
            except Exception as e:
                self.get_logger().warn(f"pub counts: {e}")

            # Publicar cajas (red)
            try:
                msg_r = Int32MultiArray()
                if len(boxes_red) > 0:
                    dimN = MultiArrayDimension(label='boxes_red', size=len(boxes_red), stride=len(boxes_red)*4)
                    dim4 = MultiArrayDimension(label='xywh',     size=4,            stride=4)
                    msg_r.layout.dim = [dimN, dim4]
                msg_r.data = [int(v) for b in boxes_red for v in b]
                self.pub_boxes_red.publish(msg_r)
            except Exception as e:
                self.get_logger().warn(f"pub boxes_red: {e}")

            # Publicar cajas (black)
            try:
                msg_b = Int32MultiArray()
                if len(boxes_black) > 0:
                    dimN = MultiArrayDimension(label='boxes_black', size=len(boxes_black), stride=len(boxes_black)*4)
                    dim4 = MultiArrayDimension(label='xywh',        size=4,               stride=4)
                    msg_b.layout.dim = [dimN, dim4]
                msg_b.data = [int(v) for b in boxes_black for v in b]
                self.pub_boxes_black.publish(msg_b)
            except Exception as e:
                self.get_logger().warn(f"pub boxes_black: {e}")

            # Publicacion de imagen anotada si esta habilitada
            if self.publish_annotated and (annotated is not None) and (self.pub_annot is not None):
                try:
                    out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                    out_msg.header.stamp = header.stamp
                    out_msg.header.frame_id = header.frame_id
                    self.pub_annot.publish(out_msg)
                except Exception as e:
                    self.get_logger().warn(f"pub annotated: {e}")

            self._publish_status_once("processing")

    # Procesamiento de mascaras y generacion de cajas
    def _detect(self, bgr: np.ndarray):
        img = bgr
        if self.blur_ksize >= 3 and (self.blur_ksize % 2 == 1):
            try:
                img = cv2.GaussianBlur(img, (self.blur_ksize, self.blur_ksize), 0)
            except Exception:
                pass

        # BGR/RGB -> HSV
        if self.input_color_order == 'rgb':
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        else:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # CLAHE en V (opcional)
        if self.use_clahe:
            h, s, v = cv2.split(hsv)
            try:
                clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=self.clahe_tile)
                v = clahe.apply(v)
                hsv = cv2.merge([h, s, v])
            except Exception:
                pass

        # Mascaras para objetivos rojos
        mask_r1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_r2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)
        mask_orange = cv2.inRange(hsv, self.orange_lo, self.orange_hi)
        mask_red = cv2.bitwise_and(mask_red, cv2.bitwise_not(mask_orange))

        # Mascara para objetivos negros
        mask_black = cv2.inRange(hsv, self.lower_black, self.upper_black)

        # Operaciones morfologicas para limpiar las mascaras
        if self.morph_open_ksize >= 3:
            kopen = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_open_ksize, self.morph_open_ksize))
            if self.open_iters > 0:
                mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_OPEN, kopen, iterations=self.open_iters)
                mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kopen, iterations=self.open_iters)

        if self.morph_close_ksize >= 3:
            kclose = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_close_ksize, self.morph_close_ksize))
            if self.close_iters > 0:
                mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_CLOSE, kclose, iterations=self.close_iters)
                mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kclose, iterations=self.close_iters)

        # Evitar que rojos oscuros entren como negro
        mask_black = cv2.bitwise_and(mask_black, cv2.bitwise_not(mask_red))

        # Cajas por color
        boxes_red: List[Tuple[int,int,int,int]] = []
        boxes_black: List[Tuple[int,int,int,int]] = []

        n_red,   boxes_red   = self._collect_boxes(mask_red, boxes_red)
        n_black, boxes_black = self._collect_boxes(mask_black, boxes_black)

        annotated = None
        if self.publish_annotated:
            annotated = img.copy()
            for (x,y,w,h) in boxes_red:
                cv2.rectangle(annotated, (x, y), (x+w, y+h), (0,0,255), 2)   # rojo (BGR)
            for (x,y,w,h) in boxes_black:
                cv2.rectangle(annotated, (x, y), (x+w, y+h), (0,0,0), 2)     # negro (BGR)
            if self.display_overlay:
                cv2.putText(annotated, f"R:{n_red} | B:{n_black}", (10, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

        return n_red, n_black, boxes_red, boxes_black, annotated

    def _collect_boxes(self, mask: np.ndarray, acc: List[Tuple[int,int,int,int]]):
        try:
            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        except Exception:
            try:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            except Exception:
                _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            count = 0
            for c in contours:
                if cv2.contourArea(c) < self.min_area_px:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                acc.append((int(x), int(y), int(w), int(h)))
                count += 1
            return count, acc

        count = 0
        for i in range(1, num_labels):
            x, y, w, h, area = stats[i, 0], stats[i, 1], stats[i, 2], stats[i, 3], stats[i, 4]
            if area < self.min_area_px:
                continue
            acc.append((int(x), int(y), int(w), int(h)))
            count += 1
        return count, acc

    # Publicacion del estado del detector
    def _publish_status(self, s: str):
        try:
            self.pub_status.publish(String(data=s))
        except Exception:
            pass
        if self.verbose:
            self._log_throttled('info', key=f'status:{s}', msg=f'[status] {s}', period=2.0)

    def _publish_status_once(self, s: str):
        self._publish_status(s)

    # Apagado ordenado del hilo de procesamiento
    def destroy_node(self):
        self._stop_evt.set()
        try:
            if self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

