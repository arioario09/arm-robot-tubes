# Robot Arm Controller

Aplikasi GUI berbasis Python Tkinter untuk mengontrol robot arm 3-axis dengan visualisasi 3D real-time.

## Fitur

✅ **Kontrol 3 Axis** - Kontrol Base (θ1), Shoulder (θ2), dan Elbow (θ3)
✅ **Visualisasi 3D Real-time** - Melihat pergerakan robot secara visual
✅ **Komunikasi Serial** - Kirim perintah ke Arduino/mikrokontroler
✅ **Tombol Kontrol** - Slider dan tombol increment untuk kontrol presisi
✅ **Forward Kinematics** - Menampilkan posisi end effector (X, Y, Z)
✅ **Multiple Port Support** - Deteksi otomatis port serial yang tersedia

## Screenshot

```
┌─────────────────────────────────────────────────────────────┐
│  Robot Control  │         Robot Visualization               │
│                 │                                            │
│  Serial Conn.   │         [3D Plot Robot Arm]               │
│  θ1 Control     │                                            │
│  θ2 Control     │         θ1=25°, θ2=40°, θ3=30°           │
│  θ3 Control     │                                            │
│  Position Info  │                                            │
└─────────────────────────────────────────────────────────────┘
```

## Instalasi

### 1. Install Dependencies Python

```bash
pip install tkinter matplotlib numpy pyserial
```

Atau install dari requirements:

```bash
pip install -r requirements.txt
```

### 2. Upload Kode Arduino

1. Buka `arduino_robot_arm.ino` dengan Arduino IDE
2. Sambungkan Arduino ke komputer
3. Pilih board dan port yang sesuai
4. Upload kode ke Arduino

### 3. Koneksi Hardware

**Servo Motor:**
- Servo 1 (Base) → Pin 9
- Servo 2 (Shoulder) → Pin 10
- Servo 3 (Elbow) → Pin 11
- VCC → 5V (atau power supply eksternal untuk servo besar)
- GND → GND

⚠️ **Catatan:** Untuk servo dengan torsi besar, gunakan power supply eksternal (5-6V) yang terpisah dari Arduino. Pastikan ground power supply terhubung dengan ground Arduino.

## Cara Penggunaan

### 1. Jalankan Aplikasi

```bash
python robot_arm_controller.py
```

### 2. Koneksi Serial

1. Klik tombol **"Refresh Ports"** untuk melihat port yang tersedia
2. Pilih port COM yang sesuai (contoh: COM3, /dev/ttyUSB0)
3. Pilih Baud Rate (default: 9600)
4. Klik **"Connect"**
5. Status akan berubah menjadi hijau "● Connected"

### 3. Kontrol Robot

**Menggunakan Slider:**
- Geser slider untuk mengubah sudut setiap axis
- Visualisasi akan update secara real-time

**Menggunakan Tombol:**
- `-5°` / `+5°` - Adjust sudut dengan increment 5 derajat
- `-1°` / `+1°` - Adjust sudut dengan increment 1 derajat (kontrol presisi)

**Kirim ke Robot:**
- Klik **"Send to Robot"** untuk mengirim posisi ke hardware
- Robot akan bergerak ke posisi yang ditampilkan di visualisasi

**Reset Position:**
- Klik **"Reset Position"** untuk kembali ke posisi home (90°, 90°, 90°)

### 4. Monitor Posisi

Panel **"End Effector Position"** menampilkan koordinat Cartesian (X, Y, Z) dari ujung robot arm berdasarkan forward kinematics.

## Protokol Komunikasi Serial

### Format Perintah (PC → Arduino)

```
A1:<angle>,A2:<angle>,A3:<angle>\n
```

**Contoh:**
```
A1:25,A2:40,A3:30\n
```

- `A1` = Sudut axis 1 (Base)
- `A2` = Sudut axis 2 (Shoulder)  
- `A3` = Sudut axis 3 (Elbow)
- Sudut dalam range 0-180 derajat
- Diakhiri dengan newline `\n`

### Format Response (Arduino → PC)

```
Position updated: A1=25, A2=40, A3=30
```

## Kustomisasi

### Mengubah Panjang Link Robot

Edit variabel di `robot_arm_controller.py`:

```python
self.L1 = 2.0  # Tinggi base (cm atau unit Anda)
self.L2 = 2.5  # Panjang upper arm
self.L3 = 2.0  # Panjang forearm
```

### Mengubah Batas Sudut

Edit range slider atau tambahkan limit di kode Arduino:

```python
# Di Python
ttk.Scale(theta1_frame, from_=0, to=180, ...)  # Ubah to=nilai_max

# Di Arduino
value = constrain(value, 0, 180);  // Ubah batas min/max
```

### Mengubah Kecepatan Gerakan

Edit delay di Arduino:

```cpp
void moveServoSmooth(Servo &servo, int currentAngle, int targetAngle) {
    // ...
    delay(15);  // Ubah nilai delay (ms) untuk kecepatan berbeda
}
```

## Troubleshooting

### ❌ Port tidak terdeteksi

- Pastikan driver USB-to-Serial sudah terinstall (CH340, FTDI, dll)
- Cek Device Manager (Windows) atau `ls /dev/tty*` (Linux/Mac)
- Coba port USB lain

### ❌ Koneksi gagal / Permission denied (Linux)

```bash
sudo chmod 666 /dev/ttyUSB0
# atau
sudo usermod -a -G dialout $USER
# logout dan login kembali
```

### ❌ Servo tidak bergerak

- Cek koneksi kabel servo (Signal, VCC, GND)
- Pastikan power supply cukup untuk servo
- Cek serial monitor Arduino untuk melihat data yang diterima
- Pastikan baud rate sama (9600)

### ❌ Gerakan servo patah-patah

- Kurangi kecepatan di fungsi `moveServoSmooth()`
- Gunakan power supply dengan ampere yang cukup
- Tambahkan kapasitor di jalur power servo

### ❌ Import error matplotlib

```bash
# Ubuntu/Debian
sudo apt-get install python3-tk python3-matplotlib

# macOS
brew install python-tk

# Windows - reinstall
pip uninstall matplotlib
pip install matplotlib
```

## Spesifikasi Teknis

- **Python Version:** 3.7+
- **Dependencies:** tkinter, matplotlib, numpy, pyserial
- **Serial:** 9600 baud (configurable)
- **Arduino:** Compatible dengan semua board (Uno, Mega, Nano, dll)
- **Servo:** Standard hobby servo (0-180°)

## Pengembangan Lebih Lanjut

Ide untuk pengembangan:

1. **Inverse Kinematics** - Input posisi X,Y,Z dan hitung sudut otomatis
2. **Path Planning** - Simpan dan replay sequence gerakan
3. **Gripper Control** - Tambah kontrol untuk end effector
4. **Sensor Feedback** - Baca posisi aktual dari encoder/potentiometer
5. **Multiple Robot** - Kontrol beberapa robot sekaligus
6. **Recording Mode** - Record dan replay gerakan
7. **Collision Detection** - Deteksi tabrakan antar link

## Lisensi

Free to use and modify for educational and commercial purposes.

## Kontributor

Dibuat dengan ❤️ menggunakan Python, Tkinter, dan Arduino

## Kontak & Support

Jika ada pertanyaan atau butuh bantuan, silakan buka issue di repository ini.

---

**Happy Coding! 🤖**
