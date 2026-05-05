import os
import time
import json
import ssl
import numpy as np
import threading
import paho.mqtt.client as mqtt
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType

# MQTT Configuration
MQTT_HOST = os.getenv("MQTT_HOST", "mqtt")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))

# Topics
TOPIC_CMD       = "phitt-f/db1/command"     # subscribe  QoS 2  — รับคำสั่ง  action: sweep | in | out
TOPIC_STATUS    = "phitt-f/db1/status"      # publish    QoS 2  retain — idle / working / halted
TOPIC_DETECTION = "phitt-f/detection/data"  # subscribe  QoS 0  — รับสัญญาณคนเข้า
TOPIC_CAM       = "phitt-f/cam/data"        # subscribe  QoS 0  — รับพิกัดจากกล้อง (relative)
TOPIC_SIM       = "phitt-f/sim/db1"         # publish    QoS 2  — J1–J4 simulation

# Robot Configuration
DB1_IP = "192.168.2.6"
LABEL  = "DB1"

DO_SUCTION_ON  = 9
DO_SUCTION_OFF = 10

SUCTION_ENGAGE_DELAY  = 0.5
SUCTION_RELEASE_DELAY = 1.0
SIM_INTERVAL          = 0.1   # 10 Hz

# Waypoints  (Cartesian: X, Y, Z, R)
HOME_POINT     = [189.79,   182.00,   98.86,  51.42]
CONVEYOR_POINT = [351.86,   -53.95,  158.03,  88.25]
WASTE_POINT    = [  2.60,  -294.58,  -20.21,   9.38]

# --- Base offset & workspace limits for camera relative coords ---
BASE_X, BASE_Y, BASE_Z, BASE_R = 149.6, 239.96, -60.03, 87.49
X_LIMITS = [149.0, 255.0]
Y_LIMITS = [239.0, 375.0]

# Robot API — module-level instances
dashboard = DobotApiDashboard(DB1_IP, 29999)
move      = DobotApiMove(DB1_IP, 30003)
feed      = DobotApi(DB1_IP, 30004)

# Global State
current_actual_point = None   # [X, Y, Z, R] จาก tool_vector_actual
current_joints       = None   # [J1, J2, J3, J4] จาก q_actual
globalLockValue      = threading.Lock()
is_running           = True
is_halted            = False  # True เมื่อ detection == 1

# --- Latest camera data (เก็บไว้รอคำสั่ง) ---
latest_cam = {"x": 0.0, "y": 0.0, "z": 0.0, "color": None}

# Startup
def start_up():
    print(f"🔄 [{LABEL}] กำลังเตรียมความพร้อม...")
    dashboard.ClearError()
    time.sleep(0.5)
    dashboard.EnableRobot()
    time.sleep(3)
    dashboard.SpeedFactor(50)
    print(f"✅ [{LABEL}] READY!")

# Feedback — background thread
def get_feed():
    global current_actual_point, current_joints
    while is_running:
        try:
            raw = feed.socket_dobot.recv(1440)
            if len(raw) >= 1440:
                data   = np.frombuffer(raw, dtype=MyType)[0]
                point  = [round(float(v), 2) for v in data["tool_vector_actual"][:4]]
                joints = [round(float(j), 2) for j in data["q_actual"][:4]]
                globalLockValue.acquire()
                current_actual_point = point
                current_joints       = joints
                globalLockValue.release()
        except Exception as e:
            print(f"  ⚠️ [{LABEL}] Feedback error: {e}")
        time.sleep(0.05)

# WaitArrive — Cartesian point only
def wait_arrive(target: list, tolerance: float = 5.0):
    print(f"  ⏳ [{LABEL}] Waiting to arrive at Point: {target}")
    start_wait = time.time()
    while True:
        globalLockValue.acquire()
        actual = current_actual_point
        globalLockValue.release()

        if actual is not None:
            if all(abs(actual[i] - target[i]) <= tolerance for i in range(4)):
                print(f"  ✨ [{LABEL}] Arrived at destination!")
                return
            if int(time.time() - start_wait) % 2 == 0 and time.time() - start_wait > 0.1:
                print(f"     [DEBUG] Still moving... Current Point: {actual}")
        else:
            print(f"  ⚠️ [{LABEL}] Warning: No feedback data yet...")

        time.sleep(0.1)

# Motion
def movj_wait(point: list, description: str = ""):
    x, y, z, r = point
    move.MovJ(x, y, z, r)
    wait_arrive(point)

# Suction
def suction_pick():
    dashboard.DO(DO_SUCTION_ON, 1)
    time.sleep(SUCTION_ENGAGE_DELAY)
    dashboard.DO(DO_SUCTION_ON, 0)

def suction_release():
    dashboard.DO(DO_SUCTION_OFF, 1)
    time.sleep(SUCTION_RELEASE_DELAY)
    dashboard.DO(DO_SUCTION_OFF, 0)

# Camera Coordinate Calculator
def calc_pick_point(cam_x: float, cam_y: float, cam_z: float) -> dict:
    """
    แปลงพิกัด relative จากกล้องเป็นพิกัด absolute ของหุ่นยนต์

    กล้องส่ง x, y, z เป็น relative offset จาก BASE
    ฟังก์ชันนี้:
      1. บวก BASE offset เข้าไป
      2. Clamp X และ Y ให้อยู่ใน workspace limits
      3. คืนค่า dict พร้อม pick_pos (ตำแหน่งหยิบ) และ approach_pos (ตำแหน่งเหนือวัตถุ)

    Returns:
        {
            "pick_pos"    : [X, Y, Z,    R]   ← ตำแหน่งสัมผัสวัตถุ
            "approach_pos": [X, Y, Z+20, R]   ← ตำแหน่งเหนือวัตถุ 20 mm
            "lower_pos"   : [X, Y, Z+10, R]   ← ตำแหน่งลงหยิบ (MovL)
        }
    """
    abs_x = BASE_X + cam_x
    abs_y = BASE_Y + cam_y
    abs_z = BASE_Z + cam_z
    abs_r = BASE_R  # rotation ไม่เปลี่ยนตามกล้อง

    # Clamp ให้อยู่ใน workspace
    abs_x = max(X_LIMITS[0], min(abs_x, X_LIMITS[1]))
    abs_y = max(Y_LIMITS[0], min(abs_y, Y_LIMITS[1]))

    print(f"  📐 [{LABEL}] Camera ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f})"
          f"  →  Robot ({abs_x:.2f}, {abs_y:.2f}, {abs_z:.2f}, {abs_r:.2f})")

    return {
        "pick_pos"    : [abs_x, abs_y, abs_z,      abs_r],
        "approach_pos": [abs_x, abs_y, abs_z + 20, abs_r],
        "lower_pos"   : [abs_x, abs_y, abs_z + 10, abs_r],
    }

def calc_sweep_point(cam_x: float, cam_y: float, cam_z: float) -> dict:
    """
    แปลงพิกัด relative จากกล้องเป็นพิกัด sweep ของหุ่นยนต์

    คำนวณทิศทาง sweep (Y_start → Y_end) โดยดูว่าวัตถุอยู่ใกล้ขอบไหน
    แล้ว sweep ผ่านทั้ง workspace ในแนว Y

    Returns:
        {
            "sweep_x"    : float        ← X คงที่ตลอด sweep (clamped)
            "sweep_z"    : float        ← Z ระหว่าง sweep
            "sweep_r"    : float        ← R คงที่ตลอด sweep
            "y_start"    : float        ← จุดเริ่มต้น sweep
            "y_end"      : float        ← จุดสิ้นสุด sweep
            "edge_pos"   : [X,Y,Z+25,R] ← ตำแหน่ง approach ก่อน sweep
        }
    """
    abs_x = BASE_X + cam_x
    abs_y = BASE_Y + cam_y
    abs_z = BASE_Z + cam_z
    abs_r = BASE_R

    sweep_x = max(X_LIMITS[0], min(abs_x, X_LIMITS[1]))

    # ถ้าวัตถุอยู่ใกล้ Y_MAX ให้ sweep จาก Y_MIN → Y_MAX และกลับกัน
    if abs(abs_y - Y_LIMITS[1]) > abs(abs_y - Y_LIMITS[0]):
        y_start, y_end = Y_LIMITS[0], Y_LIMITS[1]
    else:
        y_start, y_end = Y_LIMITS[1], Y_LIMITS[0]

    print(f"  📐 [{LABEL}] Sweep calc: X={sweep_x:.2f}  Y {y_start}→{y_end}  Z={abs_z:.2f}")

    return {
        "sweep_x" : sweep_x,
        "sweep_z" : abs_z,
        "sweep_r" : abs_r,
        "y_start" : y_start,
        "y_end"   : y_end,
        "edge_pos": [sweep_x, y_start, abs_z + 25, abs_r],
    }

# Tasks
def pick_and_place(to_conveyor: bool, cam_x: float, cam_y: float, cam_z: float):
    pts    = calc_pick_point(cam_x, cam_y, cam_z)
    target = CONVEYOR_POINT if to_conveyor else WASTE_POINT
    dest   = "Conveyor ✅" if to_conveyor else "Waste 🗑️"
    print(f"\n▶️ [{LABEL}] START: Pick and Place → {dest}")

    # Step 1: Approach
    movj_wait(HOME_POINT,          "Home           (เตรียมพร้อม)")
    movj_wait(pts["approach_pos"], "Above Object   (เหนือวัตถุ 20mm)")

    move.MovL(*pts["lower_pos"])
    suction_pick()

    # Step 2: Lift & Transit
    move.MovL(*pts["approach_pos"])
    movj_wait(HOME_POINT, "Home  (Transit)")

    # Step 3: Place
    movj_wait(target,     f"Drop Point ({dest})")
    suction_release()
    movj_wait(HOME_POINT, "Home  (Task complete)")


def edge_to_edge_sweep(cam_x: float, cam_y: float, cam_z: float):
    pts = calc_sweep_point(cam_x, cam_y, cam_z)
    sx, sz, sr       = pts["sweep_x"], pts["sweep_z"], pts["sweep_r"]
    y_start, y_end   = pts["y_start"], pts["y_end"]
    print(f"\n▶️ [{LABEL}] START: Sweep  Y {y_start} → {y_end}  at X={sx:.2f}")

    movj_wait(HOME_POINT,       "Home           (เตรียม sweep)")
    movj_wait(pts["edge_pos"],  "Sweep Edge     (ตำแหน่งเริ่ม)")

    print(f"  📉 [{LABEL}] Lowering brush...")
    move.MovL(sx, y_start, sz + 15, sr)
    time.sleep(0.2)

    print(f"  ➡️  [{LABEL}] SWEEPING...")
    move.MovL(sx, y_end, sz + 15, sr)
    time.sleep(0.2)

    move.MovL(sx, y_end, sz + 25, sr)
    movj_wait(HOME_POINT, "Home  (Sweep finished)")

# MQTT — publish status helper
def publish_status(status: str, cmd_id: int = None):
    payload = {"status": status}
    if cmd_id is not None:
        payload["id"] = cmd_id
    client.publish(TOPIC_STATUS, json.dumps(payload), qos=2, retain=True)
    print(f"  📢 [{LABEL}] Status → {status}", flush=True)

# MQTT — Callbacks
def on_connect(client, userdata, flags, reason_code, properties):
    print("CONNECT CALLED", flush=True)
    print(f"Connected to MQTT: {reason_code}", flush=True)

    topics_to_subscribe = [
        (TOPIC_CMD,       2),   # รับคำสั่ง
        (TOPIC_DETECTION, 0),   # รับสัญญาณ detection
        (TOPIC_CAM,       0),   # รับพิกัดจากกล้อง
    ]
    client.subscribe(topics_to_subscribe)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"[RECV] {msg.topic} -> {payload}", flush=True)

def handle_cam(client, userdata, msg):
    """
    phitt-f/cam/data   QoS 0
    Payload: {"x": 10.5, "y": 20.0, "z": 5.0, "color": "red"}
    อัปเดต latest_cam — พิกัดนี้จะถูกใช้เมื่อได้รับ command ถัดไป
    """
    global latest_cam
    data = json.loads(msg.payload.decode())
    latest_cam["x"]     = float(data.get("x",     0.0))
    latest_cam["y"]     = float(data.get("y",     0.0))
    latest_cam["z"]     = float(data.get("z",     0.0))
    latest_cam["color"] = data.get("color", None)
    print(f"  📷 [{LABEL}] Camera update → x={latest_cam['x']}  y={latest_cam['y']}"
          f"  z={latest_cam['z']}  color={latest_cam['color']}", flush=True)

def handle_command(client, userdata, msg):
    """
    phitt-f/db1/command   QoS 2
    Payload: {"id": 1, "x": 50, "y": 50, "action": "sweep" | "in" | "out"}

    พิกัด x, y ใน payload เป็น relative (เหมือนกับที่กล้องส่งมา)
    ถ้าไม่มี x,y ใน payload → ใช้ค่าล่าสุดจาก latest_cam แทน
    action:
      "in"    → pick_and_place → Conveyor
      "out"   → pick_and_place → Waste
      "sweep" → edge_to_edge_sweep
    """
    global is_halted
    data   = json.loads(msg.payload.decode())
    cmd_id = data.get("id", 0)
    action = data.get("action", "")

    # ใช้พิกัดจาก command ถ้ามี ไม่งั้นใช้จาก latest_cam
    cam_x = float(data.get("x", latest_cam["x"]))
    cam_y = float(data.get("y", latest_cam["y"]))
    cam_z = float(data.get("z", latest_cam["z"]))

    print(f"\n📩 [{LABEL}] Command → id={cmd_id}  action={action}"
          f"  x={cam_x}  y={cam_y}  z={cam_z}", flush=True)

    if is_halted:
        print(f"  🚫 [{LABEL}] HALTED (person detected). Command ignored.")
        return

    if action not in ("in", "out", "sweep"):
        print(f"  ⚠️ [{LABEL}] Unknown action: '{action}'. Supported: in | out | sweep")
        return

    publish_status("working", cmd_id)

    def run_task():
        if action == "in":
            pick_and_place(to_conveyor=True,  cam_x=cam_x, cam_y=cam_y, cam_z=cam_z)
        elif action == "out":
            pick_and_place(to_conveyor=False, cam_x=cam_x, cam_y=cam_y, cam_z=cam_z)
        elif action == "sweep":
            edge_to_edge_sweep(cam_x=cam_x, cam_y=cam_y, cam_z=cam_z)
        publish_status("idle", cmd_id)

    threading.Thread(target=run_task, daemon=True).start()

def handle_detection(client, userdata, msg):
    """
    phitt-f/detection/data   QoS 0
    Payload: {"detected": 0 | 1}
      1 → มีคน → Pause + publish "halted"
      0 → ปลอดภัย → Continue + publish "idle"
    """
    global is_halted
    data     = json.loads(msg.payload.decode())
    detected = int(data.get("detected", 0))

    if detected == 1 and not is_halted:
        print(f"\n🚨 [{LABEL}] Human detected → HALTING", flush=True)
        is_halted = True
        dashboard.PauseRobot()
        publish_status("halted")

    elif detected == 0 and is_halted:
        print(f"\n✅ [{LABEL}] Detection cleared → RESUMING", flush=True)
        is_halted = False
        dashboard.Continue()
        publish_status("idle")

# MQTT — Sim worker  (J1–J4 at 10 Hz → phitt-f/sim/db1)
def sim_worker():
    print(f"📡 [{LABEL}] Sim worker started → {TOPIC_SIM}", flush=True)
    while is_running:
        globalLockValue.acquire()
        joints = current_joints
        globalLockValue.release()
        if joints:
            payload = {"j1": joints[0], "j2": joints[1], "j3": joints[2], "j4": joints[3]}
            client.publish(TOPIC_SIM, json.dumps(payload), qos=2)
        time.sleep(SIM_INTERVAL)

# MQTT Client setup
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client.message_callback_add(TOPIC_CMD,       handle_command)
client.message_callback_add(TOPIC_DETECTION, handle_detection)
client.message_callback_add(TOPIC_CAM,       handle_cam)
client.on_connect = on_connect
client.on_message = on_message

# Entry Point
if __name__ == "__main__":
    start_up()

    feed_thread = threading.Thread(target=get_feed, daemon=True)
    feed_thread.start()

    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_start()

    threading.Thread(target=sim_worker, daemon=True).start()

    publish_status("idle")

    print("\n" + "="*55)
    print(f"  [{LABEL}] Listening on MQTT — waiting for commands...")
    print(f"  CMD    → {TOPIC_CMD}")
    print(f"  CAM    → {TOPIC_CAM}")
    print(f"  DETECT → {TOPIC_DETECTION}")
    print(f"  STATUS → {TOPIC_STATUS}  (retain)")
    print(f"  SIM    → {TOPIC_SIM}")
    print("  Ctrl+C to exit")
    print("="*55)

    try:
        while is_running:
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n⌨️  KeyboardInterrupt — shutting down...")
    finally:
        is_running = False
        client.loop_stop()
        client.disconnect()
        print("🔌 ปิดการทำงานเรียบร้อย")