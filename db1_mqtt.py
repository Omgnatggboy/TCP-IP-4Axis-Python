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
TOPIC_CMD       = "phitt-f/db1/command"     # subscribe  QoS 2  — รับคำสั่ง  action: sweep | in | out
TOPIC_STATUS    = "phitt-f/db1/status"      # publish    QoS 2  retain — idle / working / halted
TOPIC_DETECTION = "phitt-f/detection/data"  # subscribe  QoS 0  — รับสัญญาณคนเข้า
TOPIC_SIM       = "phitt-f/sim/db1"         # publish    QoS 2  — J1–J4 simulation
TOPIC_IR        = "phitt-f/ir/data"         # publish    QoS 0  — ส่งสัญญาณ IR sensor

# Robot Configuration
DB1_IP = "192.168.2.6"
LABEL  = "DB1"
PICK_HEIGHT_OFFSET = 1
SWEEP_HEIGHT_OFFSET = 5

DO_PUSHER = 1
DO_CONVEYOR    = 2
DO_SUCTION_ON  = 9
DO_SUCTION_OFF = 10


DI_IR_SENSOR   = 10

SUCTION_ENGAGE_DELAY  = 0.5
SUCTION_RELEASE_DELAY = 1.0
SIM_INTERVAL          = 0.1  # 10 Hz  (was incorrectly set to 1 = 1 Hz)
IR_POLL_INTERVAL      = 0.05

# Waypoints  (Cartesian: X, Y, Z, R)
HOME_POINT     = [189.79,   182.00,   98.86,  51.42]
CONVEYOR_POINT = [354.30,   -52.16,  141.59,  67.61]
WASTE_POINT    = [  2.60,  -294.58,  -20.21,   9.38]

# --- Base offset & workspace limits for camera relative coords ---
BASE_X, BASE_Y, BASE_Z, BASE_R = 150.01, 240.21, -60.45, 68.67
X_LIMITS = [150.01, 257.24]
Y_LIMITS = [240.21, 377.44]

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

# Arrival event — set by get_feed() whenever position data updates,
# so wait_arrive() can block efficiently instead of spinning with sleep().
_feed_event = threading.Event()

# Conveyor / IR pipeline state  — runs independently of robot arm and MQTT
conveyor_running = False   # True while DO_CONVEYOR is HIGH
ir_blocked       = False   # True while DI_IR_SENSOR reads HIGH (object present)


# Startup
def start_up():
    print(f"🔄 [{LABEL}] กำลังเตรียมความพร้อม...")
    dashboard.ClearError()
    time.sleep(0.5)          # hardware settle — cannot be removed
    dashboard.EnableRobot()
    time.sleep(3)            # enable sequence — cannot be removed
    dashboard.SpeedFactor(50)
    print(f"✅ [{LABEL}] READY!")

# ---------------------------------------------------------------------------
# Conveyor Control helpers
# Called only from ir_conveyor_pipeline() — no other code touches the belt.
# ---------------------------------------------------------------------------
def conveyor_start():
    global conveyor_running
    if not conveyor_running:
        dashboard.DO(DO_CONVEYOR, 1)
        conveyor_running = True
        print(f"  🟢 [{LABEL}] Conveyor ON", flush=True)

def conveyor_stop():
    global conveyor_running
    if conveyor_running:
        dashboard.DO(DO_CONVEYOR, 0)
        conveyor_running = False
        print(f"  🔴 [{LABEL}] Conveyor OFF", flush=True)

# ---------------------------------------------------------------------------
# IR + Conveyor pipeline — dedicated daemon thread
#
# Runs continuously from startup, completely independent of the robot arm,
# MQTT commands, and every other subsystem.
#
# Logic:
#   DI10 = 0  (no object)  → conveyor ON,  publish {"detected": 0} on change
#   DI10 = 1  (object)     → conveyor OFF, publish {"detected": 1} on change
#
# Only publishes on state *change* to avoid flooding the broker.
# Uses time.sleep(IR_POLL_INTERVAL) between polls — 50 ms gives ~20 Hz
# responsiveness without hammering the Dobot dashboard socket.
# ---------------------------------------------------------------------------
def ir_conveyor_pipeline():
    global ir_blocked
    print(f"  🔍 [{LABEL}] IR/Conveyor pipeline started (poll={IR_POLL_INTERVAL}s)", flush=True)

    # Belt starts ON
    conveyor_start()
    last_publish = time.time()

    while is_running:
        try:
            raw    = dashboard.DI(DI_IR_SENSOR)
            print(f"  [DEBUG] DI{DI_IR_SENSOR} raw → {raw}", flush=True)
            # Dobot dashboard DI response format: "ErrorID,Value,DI(N)\n"
            parts  = str(raw).strip().split(",")
            di_val = float(parts[1].strip().strip("{}")) if len(parts) >= 2 else 0

            if di_val > 0.0 and not ir_blocked:
                # ── Object arrived ──────────────────────────────────────────
                ir_blocked = True
                conveyor_stop()
                print(f"  🚨 [{LABEL}] IR HIGH → Conveyor OFF", flush=True)

            elif di_val == 0.0 and ir_blocked:
                # ── Object gone ─────────────────────────────────────────────
                ir_blocked = False
                conveyor_start()
                print(f"  ✅ [{LABEL}] IR LOW  → Conveyor ON", flush=True)

            now = time.time()
            if now - last_publish >= 1.0:
                client.publish(TOPIC_IR, json.dumps({"detected": 1 if ir_blocked else 0}), qos=0)
                last_publish = now

        except Exception as e:
            print(f"  ⚠️ [{LABEL}] IR pipeline error: {e}", flush=True)

        time.sleep(IR_POLL_INTERVAL)

# ---------------------------------------------------------------------------
# Feedback — background thread
# FIX: removed time.sleep(0.05) busy-poll.
#      The socket recv() call already blocks until data arrives, so adding
#      a sleep on top just introduced a guaranteed 50 ms gap between every
#      position sample.  We now block purely on the socket and signal
#      _feed_event so wait_arrive() can wake immediately on new data.
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
# FIX: replaced time.sleep(0.1) spin-poll with _feed_event.wait().
#      The thread now sleeps at zero CPU cost until get_feed() receives a
#      new position packet, then checks once and goes back to sleep.
#      This removes the 100 ms latency floor that was present even when the
#      robot had already reached its target.
#      The noisy debug print that fired ~10 times/sec is replaced with a
#      2-second interval guard using a simple timestamp comparison.
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
    move.MovJ(x, y, z, r)
    wait_arrive(point)

# ---------------------------------------------------------------------------
# Linear move with arrival wait
# FIX: all MovL calls in pick_and_place() and edge_to_edge_sweep() were
#      fire-and-forget, relying on fixed time.sleep() to let the robot
#      finish before the next command or suction trigger.  Replacing with
#      movl_wait() ties each step to actual arrival, eliminating the pauses
#      while also being correct when the robot is slower than expected.
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
PUSHER_DURATION = 4   # seconds
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

# Camera Coordinate Calculator
def calc_pick_point(cam_x: float, cam_y: float) -> dict:
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
    abs_z = BASE_Z + PICK_HEIGHT_OFFSET
    abs_r = BASE_R  # rotation ไม่เปลี่ยนตามกล้อง

    # Clamp ให้อยู่ใน workspace
    abs_x = max(X_LIMITS[0], min(abs_x, X_LIMITS[1]))
    abs_y = max(Y_LIMITS[0], min(abs_y, Y_LIMITS[1]))

    print(f"  →  Robot ({abs_x:.2f}, {abs_y:.2f}, {abs_z:.2f}, {abs_r:.2f})")

    return {
        "pick_pos"    : [abs_x, abs_y, abs_z,      abs_r],
        "approach_pos": [abs_x, abs_y, abs_z + 20, abs_r],
        "lower_pos"   : [abs_x, abs_y, abs_z + 10, abs_r],
    }

def calc_sweep_point(cam_x: float, cam_y: float) -> dict:
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
    abs_z = BASE_Z + SWEEP_HEIGHT_OFFSET
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

# ---------------------------------------------------------------------------
# Tasks
# FIX: replaced all fire-and-forget MovL + time.sleep() pairs with
#      movl_wait() so each step proceeds only when the robot has physically
#      arrived, not after a fixed wall-clock delay.
# ---------------------------------------------------------------------------
def pick_and_place(to_conveyor: bool, cam_x: float, cam_y: float):
    pts    = calc_pick_point(cam_x, cam_y)
    target = CONVEYOR_POINT if to_conveyor else WASTE_POINT
    dest   = "Conveyor ✅" if to_conveyor else "Waste 🗑️"
    print(f"\n▶️ [{LABEL}] START: Pick and Place → {dest}")

    # Step 1: Approach
    movj_wait(HOME_POINT,          "Home           (เตรียมพร้อม)")
    movj_wait(pts["approach_pos"], "Above Object   (เหนือวัตถุ 20mm)")

    # Descend and pick — wait for arrival before activating suction
    movl_wait(pts["lower_pos"])
    suction_pick()

    # Step 2: Lift & Transit
    movl_wait(pts["approach_pos"])
    movj_wait(HOME_POINT, "Home  (Transit)")

    # Step 3: Place
    movj_wait(target, f"Drop Point ({dest})")
    suction_release()
    movj_wait(HOME_POINT, "Home  (Task complete)")

    # Step 4: Pusher — fires after suction off, non-blocking (3.5 s timer)
    pusher_stroke()



def edge_to_edge_sweep(cam_x: float, cam_y: float):
    pts = calc_sweep_point(cam_x, cam_y)
    sx, sz, sr       = pts["sweep_x"], pts["sweep_z"], pts["sweep_r"]
    y_start, y_end   = pts["y_start"], pts["y_end"]
    print(f"\n▶️ [{LABEL}] START: Sweep  Y {y_start} → {y_end}  at X={sx:.2f}")

    movj_wait(HOME_POINT,       "Home           (เตรียม sweep)")
    movj_wait(pts["edge_pos"],  "Sweep Edge     (ตำแหน่งเริ่ม)")

    # Lower brush — wait for arrival before sweeping
    print(f"  📉 [{LABEL}] Lowering brush...")
    movl_wait([sx, y_start, sz + 15, sr])

    # Sweep — wait for the full stroke to complete before lifting
    print(f"  ➡️  [{LABEL}] SWEEPING...")
    movl_wait([sx, y_end, sz + 15, sr])

    movl_wait([sx, y_end, sz + 25, sr])
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
    ]
    client.subscribe(topics_to_subscribe)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"[RECV] {msg.topic} -> {payload}", flush=True)

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

    cam_x = float(data.get("x"))
    cam_y = float(data.get("y"))

    print(f"\n📩 [{LABEL}] Command → id={cmd_id}  action={action}"
          f"  x={cam_x}  y={cam_y} ", flush=True)

    if is_halted:
        print(f"  🚫 [{LABEL}] HALTED (person detected). Command ignored.")
        return

    if action not in ("in", "out", "sweep"):
        print(f"  ⚠️ [{LABEL}] Unknown action: '{action}'. Supported: in | out | sweep")
        return

    publish_status("working", cmd_id)

    def run_task():
        if action == "in":
            pick_and_place(to_conveyor=True,  cam_x=cam_x, cam_y=cam_y)
        elif action == "out":
            pick_and_place(to_conveyor=False, cam_x=cam_x, cam_y=cam_y)
        elif action == "sweep":
            edge_to_edge_sweep(cam_x=cam_x, cam_y=cam_y)
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

# ---------------------------------------------------------------------------
# Sim worker  (J1–J4 at 10 Hz → phitt-f/sim/db1)
# Uses threading.Timer to schedule itself — no blocking sleep in this path.
# Also fixed SIM_INTERVAL from 1 (1 Hz) to 0.1 (10 Hz) to match the comment.
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
    threading.Thread(target=ir_conveyor_pipeline, daemon=True, name="IR-Conveyor").start()

    # ── Robot feedback thread ──────────────────────────────────────────────────
    threading.Thread(target=get_feed, daemon=True, name="FeedThread").start()

    # ── MQTT ──────────────────────────────────────────────────────────────────
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_start()

    # ── Simulation publisher ───────────────────────────────────────────────────
    threading.Thread(target=sim_worker, daemon=True).start()

    publish_status("idle")

    print("\n" + "="*55)
    print(f"  [{LABEL}] Listening on MQTT — waiting for commands...")
    print(f"  CMD    → {TOPIC_CMD}")
    print(f"  DETECT → {TOPIC_DETECTION}")
    print(f"  STATUS → {TOPIC_STATUS}  (retain)")
    print(f"  SIM    → {TOPIC_SIM}")
    print(f"  IR     → {TOPIC_IR}  (publish on change)")
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
        conveyor_stop()        # belt off on any exit
        client.loop_stop()
        client.disconnect()
        print("🔌 ปิดการทำงานเรียบร้อย")