#include <AccelStepper.h>
#include <Stepper.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================
const int DIR1  = 13, STEP1 = 12, LS1 = 34;
const int DIR2  = 14, STEP2 = 27, LS2 = 39;
const int DIR3  = 26, STEP3 = 25, LS3 = 36;

// --- PIN GRIPPER ---
const int GRIPPER_STEPS = 2048;
Stepper gripper(GRIPPER_STEPS, 17, 33, 16, 32);
long gripperPos = 0;
bool gripperIsOpen = false;

AccelStepper ax1(1, STEP1, DIR1);
AccelStepper ax2(1, STEP2, DIR2);
AccelStepper ax3(1, STEP3, DIR3);

// ============================================================
//  HOMING AXIS 1
// ============================================================
void homeAxis1() {
  pinMode(LS1, INPUT);
  ax1.setMaxSpeed(1000);
  ax1.setAcceleration(500);

  Serial.print("Axis1: Homing...");
  while (digitalRead(LS1) == HIGH) {
    ax1.setSpeed(-400);
    ax1.runSpeed();
  }
  ax1.setCurrentPosition(0);
  Serial.print(" switch hit!");

  ax1.moveTo(200);
  while (ax1.distanceToGo() != 0) ax1.run();

  ax1.setCurrentPosition(0);
  ax1.setMaxSpeed(2000);
  ax1.setAcceleration(1000);
  Serial.println(" done. Pos=0");
}

// ============================================================
//  HOMING AXIS 2 & 3 — PARALEL (Axis 2 jalan duluan, Axis 3 menyusul)
// ============================================================
void homeAxis2and3() {
  pinMode(LS2, INPUT_PULLUP);
  pinMode(LS3, INPUT);

  bool done2 = false, done3 = false;
  bool ax3Started = false;

  ax2.setMaxSpeed(1000);
  ax2.setAcceleration(500);
  ax2.setSpeed(400); // Axis 2 langsung jalan

  ax3.setMaxSpeed(1000);
  ax3.setAcceleration(500);

  Serial.println("Axis2: Homing dimulai...");

  unsigned long ax2StartTime = millis();
  unsigned long ax3DelayMs   = 500; // Axis 3 mulai 500ms setelah Axis 2

  // ---- FASE 1: Cari limit switch (paralel) ----
  while (!done2 || !done3) {

    // Mulai Axis 3 setelah delay
    if (!ax3Started && (millis() - ax2StartTime >= ax3DelayMs)) {
      ax3.setSpeed(-400);
      ax3Started = true;
      Serial.println("Axis3: Homing dimulai (menyusul)...");
    }

    if (!done2) {
      if (digitalRead(LS2) == LOW) {
        ax2.setSpeed(0);
        ax2.setCurrentPosition(0);
        done2 = true;
        Serial.println("Axis2: Switch hit!");
      } else {
        ax2.setSpeed(400);
        ax2.runSpeed();
      }
    }

    if (!done3 && ax3Started) {
      if (digitalRead(LS3) == LOW) {
        ax3.setSpeed(0);
        ax3.setCurrentPosition(0);
        done3 = true;
        Serial.println("Axis3: Switch hit!");
      } else {
        ax3.setSpeed(-400);
        ax3.runSpeed();
      }
    }
  }

  // ---- FASE 2: Mundur dari switch (paralel) ----
  ax2.moveTo(200);
  ax3.moveTo(200);

  while (ax2.distanceToGo() != 0 || ax3.distanceToGo() != 0) {
    ax2.run();
    ax3.run();
  }

  ax2.setCurrentPosition(0);
  ax3.setCurrentPosition(0);
  ax2.setMaxSpeed(2000); ax2.setAcceleration(1000);
  ax3.setMaxSpeed(2000); ax3.setAcceleration(1000);
  Serial.println("Axis2 & Axis3: Homing selesai. Pos=0");
}

// // ============================================================
// //  HOMING AXIS 2 — Diselesaikan terlebih dahulu
// // ============================================================
// void homeAxis2() {
//   pinMode(LS2, INPUT_PULLUP);
//   ax2.setMaxSpeed(1000);
//   ax2.setAcceleration(500);

//   Serial.print("Axis2: Homing...");
//   while (digitalRead(LS2) == HIGH) {
//     ax2.setSpeed(400);
//     ax2.runSpeed();
//   }
//   ax2.setCurrentPosition(0);
//   Serial.print(" switch hit!");

//   ax2.moveTo(200);
//   while (ax2.distanceToGo() != 0) {
//     ax2.run();
//   }

//   ax2.setCurrentPosition(0);
//   ax2.setMaxSpeed(2000);
//   ax2.setAcceleration(1000);
//   Serial.println(" Axis2 Done.");
// }

// // ============================================================
// //  HOMING AXIS 3 — Dilakukan setelah Axis 2 selesai
// // ============================================================
// void homeAxis3() {
//   pinMode(LS3, INPUT);
//   ax3.setMaxSpeed(1000);
//   ax3.setAcceleration(500);

//   Serial.print("Axis3: Homing...");
//   while (digitalRead(LS3) == HIGH) {
//     ax3.setSpeed(-400);
//     ax3.runSpeed();
//   }
//   ax3.setCurrentPosition(0);
//   Serial.print(" switch hit!");

//   ax3.moveTo(800);
//   while (ax3.distanceToGo() != 0) {
//     ax3.run();
//   }

//   ax3.setCurrentPosition(0);
//   ax3.setMaxSpeed(2000);
//   ax3.setAcceleration(1000);
//   Serial.println(" Axis3 Done.");
// }

// ============================================================
//  GRIPPER HELPERS
// ============================================================
void gripperMove(long steps) {
  Serial.print("[GRIP] Gerak "); Serial.print(steps); Serial.println(" langkah...");
  gripper.step(steps);
  gripperPos += steps;
  Serial.print("[GRIP] Posisi saat ini = "); Serial.println(gripperPos);
}

void gripperMoveDeg(float deg) {
  long steps = round((deg / 360.0) * GRIPPER_STEPS);
  gripperMove(steps);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== 3-Axis + 1-Gripper Stepper Controller ===");

  gripper.setSpeed(10);
  Serial.println("Gripper diinisialisasi (10 RPM).");

  Serial.println("Memulai homing...");
  // Di setup():
  homeAxis1();
  homeAxis2and3(); // Ganti homeAxis2() + homeAxis3()

// Di handleCommand — bagian HALL:
  

// Di handleCommand — bagian H2/H3:
  

  Serial.println("\nHoming selesai! Siap menerima perintah.");
  printHelp();
}

// ============================================================
//  LOOP
// ============================================================
String cmdBuffer = "";

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuffer.trim();
      if (cmdBuffer.length() > 0) {
        handleCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }

  ax1.run();
  ax2.run();
  ax3.run();
}

// ============================================================
//  COMMAND HANDLER
// ============================================================
void handleCommand(String cmd) {
  cmd.toUpperCase();

  if (cmd == "STOP") {
    ax1.stop(); ax2.stop(); ax3.stop();
    Serial.println("[STOP] Semua motor lengan dihentikan.");
    return;
  }

  if (cmd == "POS") {
    Serial.print("[POS] Axis1="); Serial.print(ax1.currentPosition());
    Serial.print("  Axis2=");     Serial.print(ax2.currentPosition());
    Serial.print("  Axis3=");     Serial.print(ax3.currentPosition());
    Serial.print("  Gripper=");   Serial.println(gripperPos);
    return;
  }

  if (cmd == "HALL") {
    Serial.println("[HALL] Homing semua axis...");
    homeAxis1();
    homeAxis2and3();
    Serial.println("[HALL] Selesai.");
    return;
  }

  if (cmd == "?" || cmd == "HELP") {
    printHelp();
    return;
  }

  if (cmd.startsWith("H") && cmd.length() == 2) {
    int axis = cmd.substring(1).toInt();
    if (axis == 1)                     { homeAxis1(); }
    else if (axis == 2 || axis == 3)   { homeAxis2and3(); }
    else Serial.println("[ERR] Axis tidak valid (1-3).");
    return;
  }

  if (cmd == "GOPEN") {
    if (gripperIsOpen) {
      Serial.println("[GRIP] Gripper sudah terbuka, perintah diabaikan.");
      return;
    }
    Serial.println("[GRIP] Membuka (-180°)");
    gripperMoveDeg(-180);
    gripperIsOpen = true;
    return;
  }

  if (cmd == "GCLOSE") {
    if (!gripperIsOpen) {
      Serial.println("[GRIP] Gripper sudah tertutup, perintah diabaikan.");
      return;
    }
    Serial.println("[GRIP] Menutup (+180°)");
    gripperMoveDeg(180);
    gripperIsOpen = false;
    return;
  }
  if (cmd == "GRESET") {
    gripperPos = 0;
    gripperIsOpen = false; // anggap posisi reset = tertutup
    Serial.println("[GRIP] Posisi direset ke 0. State = TERTUTUP.");
    return;
  }

  if (cmd.startsWith("GR ")) { gripperMove(cmd.substring(3).toInt()); return; }
  if (cmd.startsWith("GD ")) { gripperMoveDeg(cmd.substring(3).toFloat()); return; }
  if (cmd.startsWith("GSPD ")) {
    int rpm = constrain(cmd.substring(5).toInt(), 1, 15);
    gripper.setSpeed(rpm);
    Serial.print("[GRIP] Kecepatan disetel ke: "); Serial.print(rpm); Serial.println(" RPM");
    return;
  }

  if (cmd.startsWith("G ")) {
    String args = cmd.substring(2);
    args.trim();
    String tok[3];
    int count = 0, start = 0;
    for (int i = 0; i <= (int)args.length() && count < 3; i++) {
      if (i == (int)args.length() || args.charAt(i) == ' ') {
        tok[count] = args.substring(start, i);
        tok[count].trim();
        count++;
        start = i + 1;
      }
    }
    if (count < 3) {
      Serial.println("[ERR] Format: G <pos1> <pos2> <pos3>  (gunakan _ untuk skip)");
      return;
    }
    if (tok[0] != "_") ax1.moveTo(tok[0].toInt());
    if (tok[1] != "_") ax2.moveTo(tok[1].toInt());
    if (tok[2] != "_") ax3.moveTo(tok[2].toInt());
    Serial.print("[G] Target → Axis1="); Serial.print(tok[0]);
    Serial.print("  Axis2=");            Serial.print(tok[1]);
    Serial.print("  Axis3=");            Serial.println(tok[2]);
    return;
  }

  if (cmd.length() >= 3) {
    char type = cmd.charAt(0);
    int  axis = cmd.charAt(1) - '0';
    long val  = cmd.substring(3).toInt();

    AccelStepper *ax = nullptr;
    if      (axis == 1) ax = &ax1;
    else if (axis == 2) ax = &ax2;
    else if (axis == 3) ax = &ax3;

    if (ax == nullptr) { Serial.println("[ERR] Axis tidak valid."); return; }

    if      (type == 'M') { ax->moveTo(val); Serial.print("[M] Axis"); Serial.print(axis); Serial.print(" → "); Serial.println(val); }
    else if (type == 'R') { ax->move(val);   Serial.print("[R] Axis"); Serial.print(axis); Serial.print(" += "); Serial.println(val); }
    else if (type == 'S') { ax->setMaxSpeed((float)val); Serial.print("[S] Axis"); Serial.print(axis); Serial.print(" spd="); Serial.println(val); }
    else Serial.println("[ERR] Perintah tidak dikenal.");
    return;
  }

  Serial.println("[ERR] Perintah tidak dikenal. Ketik ? untuk bantuan.");
}

// ============================================================
//  HELP TEXT
// ============================================================
void printHelp() {
  Serial.println("\n--- Daftar Perintah Utama ---");
  Serial.println("[LENGAN ROBOT]");
  Serial.println("G <p1> <p2> <p3>  Gerak SEMUA axis paralel (ex: G 3200 -1600 800)");
  Serial.println("                  Gunakan _ untuk skip     (ex: G 3200 _ 800)");
  Serial.println("M<n> <pos>        Gerak absolut 1 axis     (ex: M1 3200)");
  Serial.println("R<n> <step>       Gerak relatif 1 axis     (ex: R2 -500)");
  Serial.println("S<n> <spd>        Set max speed 1 axis     (ex: S3 1500)");
  Serial.println("H1 / H2 / H3 / HALL  Homing perintah");
  Serial.println("POS               Tampilkan posisi seluruh motor (termasuk gripper)");
  Serial.println("STOP              Hentikan semua motor lengan");

  Serial.println("\n[GRIPPER]");
  Serial.println("GOPEN / GCLOSE    Buka / tutup gripper (+180° / -180°)");
  Serial.println("GD <deg>          Putar gripper sekian derajat (ex: GD 90.5 atau GD -45)");
  Serial.println("GR <steps>        Putar gripper sekian langkah (ex: GR 1024)");
  Serial.println("GSPD <rpm>        Set kecepatan RPM gripper (1-15, ex: GSPD 12)");
  Serial.println("GRESET            Reset titik nol (0) gripper");
  Serial.println("--------------------------------\n");
}