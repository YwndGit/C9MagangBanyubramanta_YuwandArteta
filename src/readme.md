
# Serial Communication ROS2 - ROV Control

ROS2 serial bridge untuk kontrol ROV dengan joystick ke STM32.

## 🚀 Cara Menjalankan

### Mode 1: Virtual Port (Testing)

**Terminal 1: Virtual Port**
```
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0,mode=666 pty,raw,echo=0,link=/tmp/ttyV1,mode=666
```
*Biarkan jalan, jangan ditutup*

**Terminal 2: Monitor Data**
```
cat /tmp/ttyV1
```
*Data serial akan muncul di sini*

**Terminal 3: Joy Node**
```
source ~/ros2ws/install/setup.bash
ros2 run joy joy_node
```

**Terminal 4: Controller Node**
```
source ~/ros2ws/install/setup.bash
ros2 run controller controller_node
```

**Terminal 5: Serial Bridge**
```
source ~/ros2ws/install/setup.bash
source ~/SerCom_banyu/install/setup.bash
ros2 run serial_to_cmdvel sercom_to_cmdvel
```

---

### Mode 2: Hardware STM32

#### 1. Colok STM32 & Cek Port
```
ls /dev/ttyACM*
```
*Harusnya muncul: `/dev/ttyACM0`*

#### 2. Edit File Code
```
code ~/SerCom_banyu/src/serial_to_cmdvel/src/sercom_to_cmdvel.cpp
```

**Ganti baris:**
```
serial_.open("/tmp/ttyV0");     // Hapus ini
serial_.open("/dev/ttyACM0");   // Ganti jadi ini
```

#### 3. Rebuild Package
```
cd ~/SerCom_banyu/
source ~/ros2ws/install/setup.bash
colcon build --packages-select serial_to_cmdvel
source install/setup.bash
```

#### 4. Berikan Permission Port
```
sudo chmod 666 /dev/ttyACM0
```

#### 5. Jalankan Node (3 Terminal)

**Terminal 1:**
```
source ~/ros2ws/install/setup.bash
ros2 run joy joy_node
```

**Terminal 2:**
```
source ~/ros2ws/install/setup.bash
ros2 run controller controller_node
```

**Terminal 3:**
```
source ~/ros2ws/install/setup.bash
source ~/SerCom_banyu/install/setup.bash
ros2 run serial_to_cmdvel sercom_to_cmdvel
```

---

```
# YOLO + OpenVINO + ROS 2

Deteksi objek underwater (baskom & flares) dengan YOLO dan ROS 2.

## Installation

```bash
cd ~/ros2ws
colcon build
source install/setup.bash
```

## How to Run

**Terminal 1:**
```bash
source ~/ros2ws/install/setup.bash
ros2 run opencv_masking masking_node
```

**Terminal 2:**
```bash
source ~/ros2ws/install/setup.bash
ros2 run yolo_openvino yolo_node
```

**Terminal 3:**
```bash
rqt
# Select: /detected_image
```

**Terminal 4:**
```bash
ros2 topic echo /object
```

## Topics

- `/raw_image` - Gambar asli
- `/mask_image` - Gambar masking
- `/detected_image` - Gambar dengan bounding box
- `/object` - Text deteksi

## Troubleshooting

```bash
cd ~/ros2ws
rm -rf build install log
colcon build
source install/setup.bash
```

***


# PROGRAM RUN DETEKSI OBJEK BOUNDING BOX

## 🚀 Cara Menjalankan

### masuk ke direktori 

**Opsi 1, Jalanin kode ini kalau mau hasilnya ga kesave **
```
python3 detect.py --weights yuwandbest.pt --source fourth.mp4 --view-img --nosave
```
*Biarkan jalan, jangan ditutup*


### **Opsi 2 — Menjalankan Deteksi dari Kamera (contoh lain)**

```bash
python3 detect.py --weights <path_ke_file.pt> --source 0
```

* `--source 0` berarti menggunakan webcam.
* Ganti `<path_ke_file.pt>` dengan lokasi file weight milikmu.

---

## 📌 Catatan

* Pastikan file `.pt` sesuai dengan model yang sudah kamu latih.
* Format source bisa berupa:

  * Path video (`.mp4`)
  * Folder gambar
  * Webcam (`0` atau index kamera lainnya)
* Jika ingin menyimpan hasil, cukup **hapus** argumen `--nosave`.
