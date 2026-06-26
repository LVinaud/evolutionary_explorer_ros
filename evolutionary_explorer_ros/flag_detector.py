#!/usr/bin/env python3
"""Nodo de PERCEPCAO: deteccao da bandeira inimiga por visao computacional.

Usa a camera de SEGMENTACAO SEMANTICA do robo (definida no URDF como sensor
``type="segmentation"``). O simulador publica dois mapas:

  * /robot_cam/labels_map  -> cada pixel guarda o ID da label do objeto.
  * /robot_cam/colored_map -> versao colorida (uma cor por label).

A arena padrao (arena_cilindros) atribui as labels:
    ground_plane=5, red_base=10, blue_base=15, red_flag=20, flag_deploy=28,
    blue_flag=25, center/obstaculos=30, paredes=35.

O robo nasce no lado VERMELHO, entao a bandeira INIMIGA e a AZUL (label 25).

Saidas (consumidas pela maquina de estados em mission_control):
  * /flag/detected   (std_msgs/Bool)     -> bandeira no campo de visao?
  * /flag/offset     (std_msgs/Float32)  -> desloc. horizontal normalizado
                                            em [-1, 1] (0 = centralizada,
                                            >0 = a direita da imagem).
  * /flag/area_ratio (std_msgs/Float32)  -> area do blob / area da imagem
                                            (proxy de "proximidade").
  * /flag/debug_image(sensor_msgs/Image) -> mascara/anotacao para depuracao.

Trabalhamos preferencialmente com o labels_map (modo "labels"), pois comparar
o ID da label e mais robusto do que casar uma cor exata. O modo "color"
permanece disponivel para quem quiser usar o colored_map (ver aulas).
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from cv_bridge import CvBridge
import cv2
import numpy as np


class FlagDetector(Node):

    def __init__(self):
        super().__init__('flag_detector')

        # ---- Parametros (ver config/mission_params.yaml) ----
        # Trabalho 2: a deteccao passou a ser por COR na camera RGB comum
        # (/rgb_cam). A bandeira azul inimiga tem a mascara RGB 0,73,227 dada no
        # enunciado. Em vez de casar a cor exata (fragil sob iluminacao), usamos
        # uma faixa em HSV em torno do azul, robusta a variacoes de brilho.
        self.declare_parameter('detection_mode', 'labels')     # "labels" | "color"
        self.declare_parameter('image_topic', '/rgb_cam')      # camera RGB do T2
        self.declare_parameter('target_label', 25)             # usado no modo labels
        # Faixa HSV (OpenCV: H 0-179) que isola o azul da bandeira. O azul do
        # mastro/painel cai em torno de H=110. A parede externa (RGB 0,154,114)
        # e esverdeada (H~82) e fica de fora desta faixa.
        self.declare_parameter('hue_min', 100)
        self.declare_parameter('hue_max', 130)
        self.declare_parameter('sat_min', 80)
        self.declare_parameter('val_min', 50)
        self.declare_parameter('min_blob_area', 10)
        self.declare_parameter('publish_debug_image', True)

        self.detection_mode = str(self.get_parameter('detection_mode').value).lower()
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.target_label = int(self.get_parameter('target_label').value)
        self.hsv_low = np.array([
            int(self.get_parameter('hue_min').value),
            int(self.get_parameter('sat_min').value),
            int(self.get_parameter('val_min').value),
        ], dtype=np.uint8)
        self.hsv_high = np.array([
            int(self.get_parameter('hue_max').value),
            255, 255,
        ], dtype=np.uint8)
        self.min_blob_area = int(self.get_parameter('min_blob_area').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.bridge = CvBridge()

        # ---- Publishers ----
        self.pub_detected = self.create_publisher(Bool, '/flag/detected', 10)
        self.pub_offset = self.create_publisher(Float32, '/flag/offset', 10)
        self.pub_area = self.create_publisher(Float32, '/flag/area_ratio', 10)
        self.pub_debug = self.create_publisher(Image, '/flag/debug_image', 1)

        # ---- Subscriber (topico conforme o modo escolhido) ----
        if self.detection_mode == 'color':
            topic = self.image_topic
        else:
            topic = '/robot_cam/labels_map'
        self.create_subscription(Image, topic, self.image_callback, 10)

        self.get_logger().info(
            f"flag_detector iniciado | modo='{self.detection_mode}' | "
            f"topico='{topic}' | HSV={self.hsv_low.tolist()}..{self.hsv_high.tolist()} | "
            f"target_label={self.target_label}")

    # ------------------------------------------------------------------ #
    def _build_mask(self, msg: Image):
        """Retorna (mask uint8 0/255, frame_bgr_para_debug)."""
        if self.detection_mode == 'color':
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
            # Limpa ruido e fecha pequenos buracos no blob da bandeira.
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            return mask, frame

        # modo "labels": cada pixel carrega o ID da label.
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if cv_img.ndim == 3:
            # labels_map costuma vir como o ID replicado nos 3 canais.
            label_img = cv_img[:, :, 0]
        else:
            label_img = cv_img
        mask = np.where(label_img == self.target_label, 255, 0).astype(np.uint8)
        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        return mask, debug

    # ------------------------------------------------------------------ #
    def image_callback(self, msg: Image):
        try:
            mask, debug_frame = self._build_mask(msg)
        except Exception as exc:  # falha de conversao/encoding
            self.get_logger().warn(f'Falha ao processar imagem: {exc}')
            return

        height, width = mask.shape[:2]
        image_area = float(width * height)

        # Maior blob com a label/cor da bandeira.
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        offset = 0.0
        area_ratio = 0.0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area >= self.min_blob_area:
                M = cv2.moments(largest)
                if M['m00'] != 0.0:
                    cx_centroid = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']

                    # O MASTRO da bandeira (world y=0) e o elemento mais a
                    # DIREITA do blob quando o robo se aproxima pelo lado
                    # vermelho (+x). O PAINEL (world y=0.16) se estende para
                    # a ESQUERDA na imagem, puxando o centroide para longe do
                    # mastro. Usamos a borda direita do bounding box para
                    # estimar a posicao do mastro em vez do centroide.
                    x_bb, _, w_bb, _ = cv2.boundingRect(largest)
                    cx_pole = float(x_bb + w_bb)  # borda direita = mastro
                    # offset horizontal normalizado: -1 (esq) .. +1 (dir)
                    offset = (cx_pole - width / 2.0) / (width / 2.0)
                    area_ratio = area / image_area
                    detected = True

                    if self.publish_debug:
                        cv2.drawContours(debug_frame, [largest], -1, (0, 255, 0), 2)
                        cv2.circle(debug_frame, (int(cx_centroid), int(cy)), 5,
                                   (0, 0, 255), -1)   # centroide (vermelho)
                        cv2.circle(debug_frame, (int(cx_pole), int(cy)), 5,
                                   (255, 0, 0), -1)   # estimativa do mastro (azul)

        # ---- Publica resultados ----
        self.pub_detected.publish(Bool(data=detected))
        self.pub_offset.publish(Float32(data=float(offset)))
        self.pub_area.publish(Float32(data=float(area_ratio)))

        if self.publish_debug:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debug_msg.header = msg.header
                self.pub_debug.publish(debug_msg)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = FlagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
