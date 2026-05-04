import time
import json
import ssl
import numpy as np
import threading
import paho.mqtt.client as mqtt
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType

current_actual = None
globalLockValue = threading.Lock()
# =============================================================================
# Configuration
# =============================================================================
DB1_IP = "192.168.2.6"   # IP ของแขนกล ตัวที่ 1
DB2_IP = "192.168.2.7"   # IP ของแขนกล ตัวที่ 2  ← แก้ให้ตรงกับเครือข่ายจริง

BROKER_URL    = "7301c4eb07c44d2ca436d360c487bcf8.s1.eu.hivemq.cloud"
TOPIC_JOINTS  = "dobot/mg400/joints"

# --- DO Port ---
DO_SUCTION_ON  = 9    # สั่งดูด  (เปิดลม/แม่เหล็กไฟฟ้า)
DO_SUCTION_OFF = 10   # สั่งปล่อย (blow-off)

# --- Timing ---
SUCTION_ENGAGE_DELAY  = 1.5   # วินาที รอให้สุญญากาศสร้างแรงดูด
SUCTION_RELEASE_DELAY = 3.0   # วินาที รอให้ชิ้นงานหล่นลง
MQTT_INTERVAL         = 0.1   # วินาที ส่งข้อมูล joint ที่ 10 Hz


# =============================================================================
# Waypoint Definitions  (Joint Space: J1, J2, J3, J4  หน่วย: องศา)
# =============================================================================
#
#  MovJ ใช้ joint-space ทำให้ไม่มีปัญหา IK no-solution
#  แต่ละตำแหน่งด้านล่างเป็น [J1, J2, J3, J4]
#
#  *** วิธีตั้งค่า: Jog หุ่นไปยังจุดที่ต้องการใน DobotStudio
#      แล้วอ่านค่า J1–J4 จาก Joint panel แล้วใส่ด้านล่าง ***
#
# ----- DB1 (แขนกล ตัวที่ 1) -----

# จุดพัก / จุดกลาง (Home)
# ตำแหน่งที่แขนยกสูงปลอดภัย ใช้เป็นจุดผ่านระหว่าง pick → place
# ควรเป็นตำแหน่งที่ไม่ชนกับอุปกรณ์อื่น
DB1_HOME_POINT      = [189.79,   182.00,  98.86,  51.42]   # ← ใส่ค่า J1–J4 จริง

# จุดหยิบชิ้นงาน (Pick Point)
# ตำแหน่งที่ปลายแขนลงมาแตะชิ้นงาน ณ จุดที่กล้องตรวจจับ
# ควร Jog ลงมาจนถึงระดับที่ซักเกอร์แตะผิวชิ้นงานพอดี
DB1_PICK_POINT      = [191.52,   295.58,  -48.23,  130.47]    # ← ใส่ค่า J1–J4 จริง

# จุดวางสินค้าดี → Conveyor Belt (Conveyor Drop Point)
# ตำแหน่งที่แขนวางชิ้นงานที่ผ่าน QC ลงบน conveyor belt
DB1_CONVEYOR_POINT  = [352.70,   18.94,  90.53,  88.25]   # ← ใส่ค่า J1–J4 จริง

# จุดวางของเสีย → ถังขยะ (Waste Drop Point)
# ตำแหน่งที่แขนวางชิ้นงานที่ไม่ผ่าน QC ลงถังของเสีย
DB1_WASTE_POINT     = [2.60,   -294.58,  -20.21,  9.38]   # ← ใส่ค่า J1–J4 จริง

# ----- DB2 (แขนกล ตัวที่ 2) -----
DB2_HOME_POINT      = [189.79,   182.00,  98.86,  51.42]   # ← ใส่ค่า J1–J4 จริง
DB2_PICK_POINT      = [191.52,   295.58,  -48.23,  130.47]    # ← ใส่ค่า J1–J4 จริง
DB2_CONVEYOR_POINT  = [352.70,   18.94,  90.53,  88.25]   # ← ใส่ค่า J1–J4 จริง
DB2_WASTE_POINT     = [2.60,   -294.58,  -20.21,  9.38]   # ← ใส่ค่า J1–J4 จริง


# =============================================================================
# RobotController
# =============================================================================
class RobotController:
    def __init__(self, ip: str, label: str,
                 home_point: list, pick_point: list,
                 conveyor_point: list, waste_point: list):
        self.ip    = ip
        self.label = label

        # --- API connections ---
        self.dashboard = DobotApiDashboard(ip, 29999)
        self.move      = DobotApiMove(ip, 30003)
        self.feedback  = DobotApi(ip, 30004)

        # --- Waypoints (joint space) ---
        self.home_point     = home_point      # จุดกลาง/พักแขน
        self.pick_point     = pick_point      # จุดหยิบชิ้นงาน
        self.conveyor_point = conveyor_point  # จุดวาง conveyor
        self.waste_point    = waste_point     # จุดวาง waste

        self.is_running = True

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------
    def start_up(self):
        print(f"🔄 [{self.label}] กำลังเตรียมความพร้อม...")
        self.dashboard.ClearError()
        time.sleep(0.5)
        self.dashboard.EnableRobot()
        time.sleep(3)
        self.dashboard.SpeedFactor(50)
        print(f"✅ [{self.label}] READY!")

    # ------------------------------------------------------------------
    # Feedback — Joint Reading
    # ------------------------------------------------------------------
    def get_joints(self) -> list[float] | None:
        """อ่านค่า J1–J4 จาก feedback port (30004) ด้วย MyType numpy dtype"""
        try:
            raw = self.feedback.socket_dobot.recv(1440)
            if len(raw) < 1440:
                return None
            data   = np.frombuffer(raw, dtype=MyType)
            joints = data["q_actual"][0]   # shape (6,) → ใช้แค่ 4 ตัวแรก
            return [round(float(j), 2) for j in joints[:4]]
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Digital Output helpers
    # ------------------------------------------------------------------
    def _do_set(self, index: int, status: int):
        """Log และเรียก dashboard.DO (queued digital output)"""
        print(f"  ⚡ [{self.label}] DO({index}) → {'ON' if status else 'OFF'}")
        self.dashboard.DO(index, status)

    def suction_pick(self):
        """เปิดดูด แล้วรอให้สุญญากาศสร้างแรงดูด"""
        self._do_set(DO_SUCTION_ON, 1)
        time.sleep(SUCTION_ENGAGE_DELAY)
        self._do_set(DO_SUCTION_ON,  0)

    def suction_release(self):
        """ปิดดูด + blow-off แล้วรอให้ชิ้นงานหล่น"""
        self._do_set(DO_SUCTION_OFF, 1)   # เป่าออก
        time.sleep(SUCTION_RELEASE_DELAY)
        self._do_set(DO_SUCTION_OFF, 0)   # หยุดเป่า

    # ------------------------------------------------------------------
    # Motion  (MovJ — joint space, ไม่มีปัญหา IK no-solution)
    # ------------------------------------------------------------------
    def _movj(self, point: list, description: str = ""):
        """MovJ ไปยัง joint point [J1, J2, J3, J4] พร้อม log"""
        j1, j2, j3, j4 = point
        print(f"  🦾 [{self.label}] MovJ → {description}  {point}")
        self.move.MovJ(j1, j2, j3, j4)

    def pick_and_place(self, to_conveyor: bool):
        """
        3-Step Pick-and-Place (Joint Space)

        ┌─ Step 1: Pick ─────────────────────────────────────────────┐
        │   Home  →  Pick Point  →  ดูด  →  Home                    │
        ├─ Step 2: Transit ──────────────────────────────────────────┤
        │   (แขนอยู่ที่ Home แล้ว พร้อมเคลื่อนไปฝั่งปลายทาง)        │
        ├─ Step 3: Place ─────────────────────────────────────────────┤
        │   Home  →  Drop Point  →  ปล่อย  →  Home                  │
        └─────────────────────────────────────────────────────────────┘
        """
        target = self.conveyor_point if to_conveyor else self.waste_point
        dest   = "Conveyor ✅" if to_conveyor else "Waste ❌"
        print(f"\n📦 [{self.label}] pick_and_place → {dest}")

        # ── Step 1: Pick ──────────────────────────────────────────────
        print(f"  [Step 1] Pick")
        self._movj(self.home_point,  "Home       (ยกแขนขึ้น ก่อนเข้าหยิบ)")
        self._movj(self.pick_point,  "Pick Point (ลงแตะชิ้นงาน)")
        self.suction_pick()                       # ดูดชิ้นงาน

        # ── Step 2: Transit ───────────────────────────────────────────
        print(f"  [Step 2] Transit")
        self._movj(self.home_point,  "Home       (ยกแขนขึ้น พร้อมเคลื่อนย้าย)")

        # ── Step 3: Place ─────────────────────────────────────────────
        print(f"  [Step 3] Place → {dest}")
        self._movj(target,           f"Drop Point ({dest})")
        self.suction_release()                    # ปล่อยชิ้นงาน
        self._movj(self.home_point,  "Home       (ยกแขนขึ้น เสร็จสิ้น)")


# =============================================================================
# MQTT
# =============================================================================
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set("Test1234", "Test1234")
mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED)


def make_mqtt_worker(robot: RobotController, robot_key: str):
    """สร้าง worker thread สำหรับแต่ละแขนกล ส่ง joint data ที่ 10 Hz"""
    def worker():
        print(f"📡 [{robot.label}] MQTT Worker Started")
        while robot.is_running:
            joints = robot.get_joints()
            if joints:
                payload = {
                    "timestamp": time.time(),
                    robot_key: {f"j{i+1}": joints[i] for i in range(4)},
                }
                mqtt_client.publish(TOPIC_JOINTS, json.dumps(payload))
            time.sleep(MQTT_INTERVAL)
    return worker

def WaitArrive(point_list):
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False
            if is_arrive :
                globalLockValue.release()
                return
        globalLockValue.release()  
        sleep(0.001)

# =============================================================================
# Entry Point  (ใช้งาน DB1 ตัวเดียว — DB2 พร้อมเปิดใช้เมื่อแก้การเชื่อมต่อได้แล้ว)
# =============================================================================
if __name__ == "__main__":

    # --- Init DB1 ---
    db1 = RobotController(
        ip             = DB1_IP,
        label          = "DB1",
        home_point     = DB1_HOME_POINT,
        pick_point     = DB1_PICK_POINT,
        conveyor_point = DB1_CONVEYOR_POINT,
        waste_point    = DB1_WASTE_POINT,
    )

    # --- DB2 ยังไม่ได้ใช้งาน (uncomment เมื่อพร้อม) ---
    db2 = RobotController(
        ip             = DB2_IP,
        label          = "DB2",
        home_point     = DB2_HOME_POINT,
        pick_point     = DB2_PICK_POINT,
        conveyor_point = DB2_CONVEYOR_POINT,
        waste_point    = DB2_WASTE_POINT,
    )

    try:
        # --- MQTT ---
        mqtt_client.connect(BROKER_URL, 8883, 60)
        mqtt_client.loop_start()

        # --- Startup DB1 ---
        db1.start_up()
        db2.start_up()

        # --- MQTT Worker thread สำหรับ DB1 ---
        t1 = threading.Thread(target=make_mqtt_worker(db1, "robot1"), daemon=True)
        t1.start()

        # --- MQTT Worker thread สำหรับ DB2 ---
        t2 = threading.Thread(target=make_mqtt_worker(db2, "robot2"), daemon=True)
        t2.start()

        # --- Main Control Loop ---
        print("\n" + "="*40)
        print(f"  ควบคุม [{db1.label}]")
        print("  '1' = Conveyor  '2' = Waste  'q' = ออก")
        print("="*40)

        while True:
            cmd = input(f"\n[{db1.label}] คำสั่ง → ").strip()

            if cmd == "1":
                db1.pick_and_place(to_conveyor=True)
                db2.pick_and_place(to_conveyor=True)

            elif cmd == "2":
                db1.pick_and_place(to_conveyor=False)
                db2.pick_and_place(to_conveyor=False)
            elif cmd == "q":
                break
            else:
                print("⚠️  พิมพ์ '1', '2' หรือ 'q' เท่านั้น")

    finally:
        db1.is_running = False
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("\n🔌 ปิดการทำงานเรียบร้อย")