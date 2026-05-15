# 🦾 RoKiSim 6.1 | Industrial Control Pendant

![Version](https://img.shields.io/badge/Version-6.1%20Desktop-orange.svg)
![Build](https://img.shields.io/badge/Build-Passing-brightgreen.svg)
![License](https://img.shields.io/badge/License-MIT-blue.svg)

**RoKiSim 6.1**, 6 eksenli (6-DOF) endüstriyel robot kolları için geliştirilmiş, **Yapay Zeka (AI), 3D Dijital İkiz ve Ters Kinematik (IK)** algoritmalarını tek bir merkezde birleştiren masaüstü teleoperasyon ve simülasyon yazılımıdır. 

ROS 2 ve Gazebo altyapısıyla tam senkronize çalışan bu sistem; laboratuvar ortamından çıkıp, güvenlik kısıtlamalarını aşarak tam teşekküllü bir **"Teach Pendant"** (Endüstriyel El Kumandası) olarak tasarlanmıştır.

---

## ✨ Temel Özellikler

* **🤖 3D Digital Twin (Senkronize İkiz):** `Three.js` ile güçlendirilmiş, robotun fiziksel durumunu milisaniyeler içinde ekrana yansıtan yerel 3D motoru.
* **👁️ AI Computer Vision (Yapay Zeka Görüşü):** `Google MediaPipe` altyapısı ve Native WebRTC ile el iskeletini (21 eklem) gerçek zamanlı tarayıp, parmak hareketlerini doğrudan robot motorlarına eşleyen (Mapping) otonom sürüş modu.
* **🎯 Inverse Kinematics (Ters Kinematik):** Hedef X, Y, Z koordinatlarına gitmek için Gradient Descent algoritması kullanan, eklem katlanmalarını (Singularity) engelleyen yerleşik sayısal çözücü.
* **🎮 Analog Teleoperasyon:** Yumuşak geçişli (Low-Pass Filter) sanal joystick ile X-Y düzleminde güvenli manuel sürüş.
* **🛡️ Offline/Local Mode (Zarif Düşüş):** ROS veya Gazebo bağlantısı kopsa dahi çökmeyen; 3D İkiz ve AI özelliklerini **"SYS: LOCAL MODE"** altında lokal olarak çalıştırmaya devam eden endüstriyel güvenlik mimarisi.

---

## 🛠️ Kullanılan Teknolojiler (Tech Stack)

Sistem, maksimum performans ve donanım erişimi için **Electron.js** kabuğu ile paketlenmiştir.

* **Çekirdek:** JavaScript, HTML5, CSS3 (Dark Industrial UI)
* **Masaüstü Kabuğu:** Electron.js (Hardware Acceleration Disabled for Linux GPU stability)
* **3D & Fizik Motoru:** Three.js
* **Robotik Haberleşme:** ROSLib.js (WebSocket ws://localhost:9090)
* **Yapay Zeka / Görüntü İşleme:** MediaPipe Hands, HTML5 getUserMedia API

---

## � Bu Sürümde Yapılan Değişiklikler

* `src/rokisim_gazebo/web/app.js` içinde 3D Dijital İkiz sahnesi için başlatma ve animasyon sıralaması düzeltildi.
* `setupWorkspaceAndAxes()` fonksiyonu, sahne objesi oluşturulduktan sonra çağrılacak şekilde yeniden düzenlendi.
* `openGazeboWindow()` ile Gazebo butonu artık ayrı bir pencere veya sekme açmaya yönelik çalışacak şekilde ayarlandı.
* `src/rokisim_gazebo/web/styles.css` içinde 3D panel yüksekliği artırıldı ve alt siyah alan daha koyu, daha dolu görünmesi için güncellendi.
* UI panel düzeni iyileştirildi; 3D robot ekranı daha geniş görünür hale getirildi.

---

## �🚀 Kurulum ve Başlatma

Bu projeyi yerel makinenizde çalıştırmak için sisteminizde [Node.js](https://nodejs.org/) yüklü olması gerekmektedir. (ROS 2 / Gazebo kurulumu isteğe bağlıdır, yazılım yerel modda da çalışır).

### 1. Depoyu Klonlayın
```bash
git clone [https://github.com/imre-robotics/RoKiSim.git](https://github.com/imre-robotics/RoKiSim.git)
cd RoKiSim
```

### 2. Bağımlılıkları Kurun
```bash
npm install
```

### 3. Uygulamayı Başlatın
```bash
npm start
```

> Not: Bu sürüm `main.js` üzerinde güvenli Electron yapılandırması ile çalışır; `nodeIntegration` devre dışı bırakıldı ve `contextIsolation` etkinleştirildi.
