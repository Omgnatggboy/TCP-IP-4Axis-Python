import os
import time
import json
import ssl
import numpy as np
import threading
import paho.mqtt.client as mqtt
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType

# MQTT Configuration
MQTT_HOST = "10.35.120.100"
MQTT_PORT = 1883

# Topics
TOPIC_CMD       = "phitt-f/db2/command"     # subscribe  QoS 2  — รับคำสั่ง  action: in | out
TOPIC_STATUS    = "phitt-f/db2/status"      # publish    QoS 2  retain — idle / working / halted
TOPIC_DETECTION = "phitt-f/detection/data"  # subscribe  QoS 0  — รับสัญญาณคนเข้า
TOPIC_SIM       = "phitt-f/sim/db2"         # publish    QoS 2  — J1–J4 simulation
TOPIC_IR        = "phitt-f/ir/data"         # publish    QoS 0  — ส่งสัญญาณ IR sensor

# Robot Configuration
DB2_IP = "192.168.2.7"
LABEL  = "DB2"


DO_PUSHER = 1
DO_CONVEYOR    = 2
DO_SUCTION_ON  = 9
DO_SUCTION_OFF = 10


SUCTION_ENGAGE_DELAY  = 0.5
SUCTION_RELEASE_DELAY = 0.3
SIM_INTERVAL          = 0.1  # 10 Hz
IR_POLL_INTERVAL      = 0.05
DB2_MOCK_COMMAND      = False
MOCK_COMMAND_INTERVAL = float(os.getenv("DB2_MOCK_INTERVAL", "5.0"))

# Waypoints  (Cartesian: X, Y, Z, R)
HOME_POINT      = [189.79,   182.00,   115.86,   0]
PICK_POINT      = [208.47,   304.42,  -41.36, 0] #fix Z to be on object surface, adjust x,y,r as needed
CONVEYOR_POINT  = [353.12,   -47.81,  150.33,   0] #set to be above conveyor

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

# Arrival event — set by get_feed() whenever position data updates,
# so wait_arrive() can block efficiently instead of spinning with sleep().
_feed_event = threading.Event()

# Conveyor / IR pipeline state  — runs independently of robot arm and MQTT
conveyor_running = False   # True while DO_CONVEYOR is HIGH
ir_blocked       = False   # True while DI_IR_SENSOR reads HIGH (object present)

# Task synchronization — prevent concurrent command execution
_task_lock = threading.Lock()
_task_running = False

# Startup
def start_up():
    print("🔄 [DB2] กำลังเตรียมความพร้อม...")
    dashboard.ClearError()
    time.sleep(0.5)
    dashboard.EnableRobot()
    time.sleep(3)
    dashboard.SpeedFactor(50)
    print("✅ [DB2] READY!")


# ---------------------------------------------------------------------------
# Feedback — background thread
# Uses event-based synchronization with _feed_event instead of polling.
# The socket recv() call blocks until data arrives, so we signal _feed_event
# so wait_arrive() can wake immediately on new data without CPU overhead.
# ---------------------------------------------------------------------------
def get_feed():
    global current_actual_point, current_joints
    while is_running:
        try:
            raw = feed.socket_dobot.recv(1440)
            if len(raw) >= 1440:
                data   = np.frombuffer(raw, dtype=MyType)[0]
                point  = [round(float(v), 2) for v in data["tool_vector_actual"][:4]]
                joints = [round(float(j), 2) for j in data["q_actual"][:4]]
                with globalLockValue:
                    current_actual_point = point
                    current_joints       = joints
                _feed_event.set()   # wake any waiting wait_arrive() call
        except Exception as e:
            print(f"  ⚠️ [{LABEL}] Feedback error: {e}")
            # Brief back-off only on error to avoid tight error-loop
            time.sleep(0.05)

# ---------------------------------------------------------------------------
# WaitArrive — Cartesian point only
# Uses event-based waiting instead of polling.
# The thread sleeps at zero CPU cost until get_feed() receives a new position
# packet, then checks once and goes back to sleep. This removes latency while
# being more efficient than polling with sleep().
# ---------------------------------------------------------------------------
def wait_arrive(target: list, tolerance: float = 10.0):
    print(f"  ⏳ [{LABEL}] Waiting to arrive at Point: {target}")
    last_debug = 0.0
    while True:
        # Block until get_feed() delivers a fresh position update
        _feed_event.wait()
        _feed_event.clear()

        with globalLockValue:
            actual = current_actual_point

        if actual is None:
            print(f"  ⚠️ [{LABEL}] Warning: No feedback data yet...")
            continue

        if all(abs(actual[i] - target[i]) <= tolerance for i in range(4)):
            print(f"  ✨ [{LABEL}] Arrived at destination!")
            return

        now = time.monotonic()
        if now - last_debug >= 2.0:
            print(f"     [DEBUG] Still moving... Current Point: {actual}")
            last_debug = now

# Motion — joint move with arrival wait
def movj_wait(point: list, description: str = ""):
    x, y, z, r = point
    print(f"\n▶️ [{LABEL}] Moving to {description} → Point: {point}")
    move.MovJ(x, y, z, r)
    wait_arrive(point)

# ---------------------------------------------------------------------------
# Linear move with arrival wait
# All MovL calls tie each step to actual arrival, eliminating the pauses
# while also being correct when the robot is slower than expected.
# ---------------------------------------------------------------------------
def movl_wait(point: list):
    x, y, z, r = point
    move.MovL(x, y, z, r)
    wait_arrive(point)

# Suction
# NOTE: SUCTION_ENGAGE_DELAY / SUCTION_RELEASE_DELAY are real hardware
#       dwell times (vacuum build-up / release).  They cannot be removed.
#       However they now run AFTER we have confirmed the robot has stopped
#       (because the caller uses movl_wait), so they no longer serve as a
#       proxy for motion completion.
def suction_pick():
    dashboard.DO(DO_SUCTION_ON, 1)
    time.sleep(SUCTION_ENGAGE_DELAY)
    dashboard.DO(DO_SUCTION_ON, 0)

def suction_release():
    dashboard.DO(DO_SUCTION_OFF, 1)
    time.sleep(SUCTION_RELEASE_DELAY)
    dashboard.DO(DO_SUCTION_OFF, 0)

# ---------------------------------------------------------------------------
# Pusher stroke — DO1, exactly 3.5 seconds
# Uses threading.Timer so the caller is NOT blocked during the dwell.
# The timer fires on a separate thread and turns the output back off.
# ---------------------------------------------------------------------------
PUSHER_DURATION = 5  # seconds
_pusher_available = True
_pusher_lock = threading.Lock()

def pusher_stroke():
    """Fire DO_PUSHER for exactly PUSHER_DURATION seconds, non-blocking."""
    global _pusher_available
    with _pusher_lock:
        _pusher_available = False
    dashboard.DO(DO_PUSHER, 1)
    print(f"  ➡️  [{LABEL}] Pusher ON  (will auto-off in {PUSHER_DURATION}s)", flush=True)
    threading.Timer(PUSHER_DURATION, _pusher_off).start()

def _pusher_off():
    global _pusher_available
    dashboard.DO(DO_PUSHER, 0)
    print(f"  ⬅️  [{LABEL}] Pusher OFF", flush=True)
    # Wait additional 1 second for complete retraction before marking available
    time.sleep(3.0)
    with _pusher_lock:
        _pusher_available = True

def wait_pusher_finished():
    """Block until pusher completes its full cycle and returns to available state."""
    print(f"  ⏳ [{LABEL}] Waiting for pusher to complete...", flush=True)
    while True:
        with _pusher_lock:
            if _pusher_available:
                break
        time.sleep(0.1)
    print(f"  ✨ [{LABEL}] Pusher cycle complete!", flush=True)

def conveyer_start_stop():
    dashboard.DO(DO_CONVEYOR, 1)
    time.sleep(1.5)
    dashboard.DO(DO_CONVEYOR, 0)


# ---------------------------------------------------------------------------
# Task: Pick and Place to Conveyor (Fixed Routine)
# Uses fixed PICK_POINT to pick object and places at CONVEYOR_POINT
# Acquires task lock to prevent concurrent execution, waits for HOME before start
# ---------------------------------------------------------------------------
def pick_and_place_to_conveyor():
    global _task_running
    
    with _task_lock:
        _task_running = True
    
    try:
        approach_pos = [PICK_POINT[0], PICK_POINT[1], PICK_POINT[2] + 20, PICK_POINT[3]]
        lower_pos    = [PICK_POINT[0], PICK_POINT[1], PICK_POINT[2],      PICK_POINT[3]]
        target       = CONVEYOR_POINT
        dest         = "Conveyor ✅"

        print(f"\n▶️ [{LABEL}] START: Pick and Place → {dest}")
        
        movj_wait(HOME_POINT, "Home  (Start)")

        # Step 1: Approach
        movj_wait(approach_pos,        "Above Pick Point (prepare)")

        # Descend and pick — wait for arrival before activating suction
        movl_wait(lower_pos)
        suction_pick()

        # Step 2: Lift & Transit
        movl_wait(approach_pos)
        movj_wait(HOME_POINT, "Home  (Transit)")

        # Step 3: Place
        movj_wait(target, f"Drop Point ({dest})")
        suction_release()
        movj_wait(HOME_POINT, "Home  (Task complete)")

        # Step 4: Pusher — fires after suction off, non-blocking (3.5 s timer)
        pusher_stroke()  # brief pause before starting conveyor to allow pusher to extend
        wait_pusher_finished()  # wait for pusher to complete before conveyor
        conveyer_start_stop()
    
    finally:
        with _task_lock:
            _task_running = False
    



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
        (TOPIC_CMD,       2),   # รับคำสั่ง pick_and_place
        (TOPIC_DETECTION, 0),   # รับสัญญาณ detection
    ]
    client.subscribe(topics_to_subscribe)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"[RECV] {msg.topic} -> {payload}", flush=True)

def handle_command(client, userdata, msg):
    """
    phitt-f/db2/command   QoS 2
    Payload: {"id": 1, "action": 1}

    action:
      1 → pick_and_place → Conveyor (fixed routine)
    
    Task execution is serialized using _task_lock to prevent concurrent runs.
    """
    global is_halted, _task_running
    data   = json.loads(msg.payload.decode())
    cmd_id = data.get("id", 0)
    action = data.get("action", 0)

    print(f"\n📩 [{LABEL}] Command → id={cmd_id}  action={action}", flush=True)

    if is_halted:
        print(f"  🚫 [{LABEL}] HALTED (person detected). Command ignored.")
        return
    
    if _task_running:
        print(f"  ⏳ [{LABEL}] Task already running. Command rejected.")
        return

    if action != 1:
        print(f"  ⚠️ [{LABEL}] Unknown action: {action}. Supported: 1 (pick to conveyor)")
        return

    publish_status("working", cmd_id)

    def run_task():
        try:
            pick_and_place_to_conveyor()
        finally:
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


def mock_command_sender():
    if not DB2_MOCK_COMMAND:
        return

    print(f"  🧪 [{LABEL}] Mock command sender enabled (every {MOCK_COMMAND_INTERVAL}s)", flush=True)
    while is_running:
        time.sleep(MOCK_COMMAND_INTERVAL)
        if is_halted:
            continue
        payload = json.dumps({"id": 1, "action": 1})
        print(f"  🧪 [{LABEL}] Publishing mock command → {payload}", flush=True)
        client.publish(TOPIC_CMD, payload, qos=2)

# ---------------------------------------------------------------------------
# Sim worker  (J1–J4 at 10 Hz → phitt-f/sim/db2)
# Uses threading.Timer to schedule itself — no blocking sleep in this path.
# ---------------------------------------------------------------------------
def sim_worker():
    if not is_running:
        return

    try:
        with globalLockValue:
            joints = current_joints

        if joints:
            payload = {
                "j1": joints[0],
                "j2": joints[1],
                "j3": joints[2],
                "j4": joints[3]
            }
            client.publish(TOPIC_SIM, json.dumps(payload), qos=0)

    except Exception as e:
        print(f"📡 [{LABEL}] Sim worker error: {e}", flush=True)

    threading.Timer(SIM_INTERVAL, sim_worker).start()

# MQTT Client setup
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client.message_callback_add(TOPIC_CMD,       handle_command)
client.message_callback_add(TOPIC_DETECTION, handle_detection)
client.on_connect = on_connect
client.on_message = on_message

# Entry Point
if __name__ == "__main__":
    start_up()

    # ── IR + Conveyor pipeline — starts immediately, runs for entire lifetime ──
    # Launched before MQTT so the belt is live even if the broker is slow to connect.

    # ── Robot feedback thread ──────────────────────────────────────────────────
    threading.Thread(target=get_feed, daemon=True, name="FeedThread").start()

    # ── MQTT ──────────────────────────────────────────────────────────────────
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_start()

    if DB2_MOCK_COMMAND:
        threading.Thread(target=mock_command_sender, daemon=True).start()

    # ── Simulation publisher ───────────────────────────────────────────────────
    threading.Thread(target=sim_worker, daemon=True).start()

    publish_status("idle")

    print("\n" + "="*55)
    print(f"  [{LABEL}] Listening on MQTT — waiting for commands...")
    print(f"  CMD    → {TOPIC_CMD}")
    print(f"  DETECT → {TOPIC_DETECTION}")
    print(f"  STATUS → {TOPIC_STATUS}  (retain)")
    print(f"  SIM    → {TOPIC_SIM}")
    print(f"  IR     → local only (no MQTT publish)")
    print(f"  CONVEYOR: {'ON ✅' if conveyor_running else 'OFF ❌'}")
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