#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class VideoKaydedici(Node):
    def __init__(self):
        super().__init__('video_kaydedici_node')
        self.kopru = CvBridge()
        
        # Kayıt Yeri: ika_ws içinde otomatik bir klasör oluşturur
        self.kayit_klasoru = os.path.expanduser('~/ika_ws/veri_seti_videolari')
        os.makedirs(self.kayit_klasoru, exist_ok=True)
        
        # Her kayda o anın saat ve tarihini isim olarak verir (Üst üste binmesin diye)
        zaman_etiketi = time.strftime("%Y%m%d_%H%M%S")
        self.dosya_yolu = os.path.join(self.kayit_klasoru, f"tabela_kayit_{zaman_etiketi}.avi")
        
        self.video_yazici = None
        
        # Kamerayı dinle (Gazebo'daki topic adın /camera/image_raw değilse burayı düzelt!)
        self.abonelik = self.create_subscription(Image, '/camera/image_raw', self.kamera_callback, 10)
        
        self.get_logger().info(f"Kamera dinleniyor... Kayıt Yeri: {self.dosya_yolu}")
        self.get_logger().info("Durdurmak ve videoyu güvenle kaydetmek için terminalde CTRL+C yapın.")

    def kamera_callback(self, msg):
        try:
            kare = self.kopru.imgmsg_to_cv2(msg, "bgr8")
            yukseklik, genislik, _ = kare.shape
            
            # İlk kare geldiğinde VideoWriter'ı başlat (Kameranın çözünürlüğünü otomatik algılar)
            if self.video_yazici is None:
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_yazici = cv2.VideoWriter(self.dosya_yolu, fourcc, 15.0, (genislik, yukseklik))
                self.get_logger().info(f"Video kaydı başladı! Çözünürlük: {genislik}x{yukseklik}")

            # Gelen her kareyi videoya yaz
            self.video_yazici.write(kare)
            
            # Ekranda da canlı olarak görelim
            cv2.imshow("IKA Kamera Kayit (Durdurmak icin CTRL+C)", kare)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Kamera hatası: {e}")

    def durdur(self):
        # Program kapanırken videoyu düzgünce kapatıp mühürler (Dosya bozulmaz)
        if self.video_yazici is not None:
            self.video_yazici.release()
            self.get_logger().info(f"BAŞARILI: Video {self.dosya_yolu} konumuna kaydedildi!")
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    dugum = VideoKaydedici()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.durdur()
        dugum.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()