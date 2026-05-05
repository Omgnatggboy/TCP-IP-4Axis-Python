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
TOPIC_CMD       = "phitt-f/db2/command"     # subscribe  QoS 2  — รับคำสั่ง
TOPIC_STATUS    = "phitt-f/db2/status"      # publish    QoS 2  retain — idle/working/halted
TOPIC_DETECTION = "phitt-f/detection/data"  # subscribe  QoS 0  — รับสัญญาณคนเข้า
TOPIC_SIM       = "phitt-f/sim/db2"         # publish    QoS 2  — J1–J4 simulation

# Robot Configuration
DB2_IP = "192.168.2.7"

DO_SUCTION_ON  = 9
DO_SUCTION_OFF = 10

SUCTION_ENGAGE_DELAY  = 1.5
SUCTION_RELEASE_DELAY = 3.0
SIM_INTERVAL          = 0.1   # 10 Hz

# Waypoints  (Cartesian: X, Y, Z, R)
HOME_POINT      = [189.79,   182.00,   98.86,   51.42]
PICK_POINT      = [191.52,   295.58,  -48.23,  130.47]
CONVEYOR_POINT  = [356.07,   -49.89,  165.05,   -5.02]
WASTE_POINT     = [  2.60,  -294.58,  -20.21,    9.38]

# Robot API — module-level instances
dashboard = DobotApiDashboard(DB2_IP, 29999)
move      = DobotApiMove(DB2_IP, 30003)
feed      = DobotApi(DB2_IP, 30004)

# Global State
current_actual_point = None      # [X, Y, Z, R] จาก tool_vector_actual
current_joints       = None      # [J1, J2, J3, J4] จาก q_actual
globalLockValue      = threading.Lock()
is_running           = True
is_halted            = False     # True เมื่อ detection == 1

# Startup
def start_up():
    print("🔄 [DB2] กำลังเตรียมความพร้อม...")
    dashboard.ClearError()
    time.sleep(0.5)
    dashboard.EnableRobot()
    time.sleep(3)
    dashboard.SpeedFactor(50)
    print("✅ [DB2] READY!")

# Feedback — background thread (mirrors GetFeed from main.py)
def get_feed():
    global current_actual_point, current_joints
    while is_running:
        try:
            raw = feed.socket_dobot.recv(1440)
            if len(raw) >= 1440:
                data = np.frombuffer(raw, dtype=MyType)[0]
                point  = [round(float(v), 2) for v in data["tool_vector_actual"][:4]]
                joints = [round(float(j), 2) for j in data["q_actual"][:4]]
                globalLockValue.acquire()
                current_actual_point = point
                current_joints       = joints
                globalLockValue.release()
        except Exception as e:
            print(f"  ⚠️ [DB2] Feedback error: {e}")
        time.sleep(0.05)

# WaitArrive — Cartesian point only (mirrors WaitArrive from main.py)
def wait_arrive(target: list, tolerance: float = 5.0):
    print(f"  ⏳ [DB2] Waiting to arrive at Point: {target}")
    start_wait = time.time()
    while True:
        globalLockValue.acquire()
        actual = current_actual_point
        globalLockValue.release()

        if actual is not None:
            if all(abs(actual[i] - target[i]) <= tolerance for i in range(4)):
                print("  ✨ [DB2] Arrived at destination!")
                return
            if int(time.time() - start_wait) % 2 == 0 and time.time() - start_wait > 0.1:
                print(f"     [DEBUG] Still moving... Current Point: {actual}")
        else:
            print("  ⚠️ [DB2] Warning: No feedback data yet...")

        time.sleep(0.1)

# Motion
def movj_wait(point: list, description: str = ""):
    x, y, z, r = point
    print(f"  🦾 [DB2] MovJ → {description}  {point}")
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

# Pick and Place
def pick_and_place(to_conveyor: bool):
    target = CONVEYOR_POINT if to_conveyor else WASTE_POINT
    dest   = "Conveyor ✅" if to_conveyor else "Waste 🗑️"
    print(f"\n📦 [DB2] pick_and_place → {dest}")

    print("  [Step 1] Pick")
    movj_wait(HOME_POINT, "Home       (ยกแขนขึ้น ก่อนเข้าหยิบ)")
    movj_wait(PICK_POINT, "Pick Point (ลงแตะชิ้นงาน)")
    suction_pick()

    print("  [Step 2] Transit")
    movj_wait(HOME_POINT, "Home       (ยกแขนขึ้น พร้อมเคลื่อนย้าย)")

    print(f"  [Step 3] Place → {dest}")
    movj_wait(target,     f"Drop Point ({dest})")
    suction_release()
    movj_wait(HOME_POINT, "Home       (ยกแขนขึ้น เสร็จสิ้น)")

# MQTT — publish status helper
def publish_status(status: str, cmd_id: int = None):
    payload = {"status": status}
    if cmd_id is not None:
        payload["id"] = cmd_id
    client.publish(TOPIC_STATUS, json.dumps(payload), qos=2, retain=True)
    print(f"  📢 [DB2] Status → {status}")

# MQTT — Callbacks  (same structure as example-mqqt.py)
def on_connect(client, userdata, flags, reason_code, properties):
    print("CONNECT CALLED", flush=True)
    print(f"Connected to MQTT: {reason_code}", flush=True)

    topics_to_subscribe = [
        (TOPIC_CMD,       2),   # รับคำสั่ง pick_and_place
        (TOPIC_DETECTION, 0),   # รับสัญญาณ detection
    ]
    client.subscribe(topics_to_subscribe)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"[RECV] {msg.topic} -> {payload}", flush=True)

def handle_command(client, userdata, msg):
    global is_halted
    data   = json.loads(msg.payload.decode())
    cmd_id = data.get("id", 0)
    action = data.get("action")
    print(f"\n📩 [DB2] Command received → id={cmd_id}  action={action}", flush=True)
    if is_halted:
        print("  🚫 [DB2] HALTED (person detected). Command ignored.")
        return
    if action not in (1, 2):
        print(f"  ⚠️ [DB2] Unknown action: {action}. Supported: 1=Conveyor  2=Waste")
        return
    publish_status("working", cmd_id)

    to_conveyor = (action == 1)
    def run_task():
        pick_and_place(to_conveyor=to_conveyor)
        publish_status("idle", cmd_id)
    threading.Thread(target=run_task, daemon=True).start()

def handle_detection(client, userdata, msg):
    global is_halted
    data     = json.loads(msg.payload.decode())
    detected = int(data.get("detected", 0))

    if detected == 1 and not is_halted:
        print("\n🚨 [DB2] Human detected → HALTING", flush=True)
        is_halted = True
        dashboard.PauseRobot()
        publish_status("halted")

    elif detected == 0 and is_halted:
        print("\n✅ [DB2] Detection cleared → RESUMING", flush=True)
        is_halted = False
        dashboard.Continue()
        publish_status("idle")

# MQTT — Sim worker  (J1–J4 at 10 Hz → phitt-f/sim/db2)
def sim_worker():
    print(f"📡 [DB2] Sim worker started → {TOPIC_SIM}", flush=True)
    while is_running:
        globalLockValue.acquire()
        joints = current_joints
        globalLockValue.release()
        if joints:
            payload = {"j1": joints[0], "j2": joints[1], "j3": joints[2], "j4": joints[3]}
            client.publish(TOPIC_SIM, json.dumps(payload), qos=2)
        time.sleep(SIM_INTERVAL)

# MQTT Client setup  (same as example-mqqt.py)
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client.message_callback_add(TOPIC_CMD,       handle_command)
client.message_callback_add(TOPIC_DETECTION, handle_detection)
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

    print("\n" + "="*50)
    print("  [DB2] Listening on MQTT — waiting for commands...")
    print(f"  CMD    → {TOPIC_CMD}")
    print(f"  DETECT → {TOPIC_DETECTION}")
    print(f"  STATUS → {TOPIC_STATUS}  (retain)")
    print(f"  SIM    → {TOPIC_SIM}")
    print("  Ctrl+C to exit")
    print("="*50)

    try:
        while is_running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n⌨️  KeyboardInterrupt — shutting down...")
    finally:
        is_running = False
        client.loop_stop()
        client.disconnect()
        print("🔌 ปิดการทำงานเรียบร้อย")