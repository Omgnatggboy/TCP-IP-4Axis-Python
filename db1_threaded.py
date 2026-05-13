import os
import time
import json
import queue
import ssl
import numpy as np
import threading
import math
import paho.mqtt.client as mqtt
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType

# ============================================================
#  MQTT Configuration
# ============================================================
MQTT_HOST = "10.35.120.100"
MQTT_PORT = 1883

TOPIC_CMD       = "phitt-f/db1/command"
TOPIC_STATUS    = "phitt-f/db1/status"
TOPIC_DETECTION = "phitt-f/detection/data"
TOPIC_SIM       = "phitt-f/sim/db1"
TOPIC_IR        = "phitt-f/ir/data"
TOPIC_CONVEYOR1 = "phitt-f/conveyor1/status"

# ============================================================
#  Robot Configuration
# ============================================================
DB1_IP = "192.168.2.6"
LABEL  = "DB1"
PICK_HEIGHT_OFFSET  = 0
SWEEP_HEIGHT_OFFSET = 5

DO_PUSHER      = 1
DO_CONVEYOR    = 2
DO_EMERG_LIGHT = 5
DO_ALARM       = 6
DO_SUCTION_ON  = 9
DO_SUCTION_OFF = 10

DI_IR_SENSOR     = 10
DI_PUSHER_SENSOR = 9

SUCTION_ENGAGE_DELAY = 0.5
SUCTION_RELEASE_DELAY = 1.0
SIM_INTERVAL          = 0.1

# Waypoints (Cartesian: X, Y, Z, R)
HOME_POINT     = [189.79,   182.00,   98.86,  51.42]
CONVEYOR_POINT = [354.30,   -52.16,  162.59,  67.61]
WASTE_POINT    = [  2.60,  -294.58,  -20.21,   9.38]

BASE_X, BASE_Y, BASE_Z, BASE_R = 149.21, 237.31, -61.20, 68.67
X_LIMITS = [150.01, 257.24]
Y_LIMITS = [240.21, 377.44]

MOTOR_RPM          = 60.0
ROLLER_DIAMETER_MM = 40.0

# ============================================================
#  Robot API connections
# ============================================================
dashboard = DobotApiDashboard(DB1_IP, 29999)
move      = DobotApiMove(DB1_IP, 30003)
feed      = DobotApi(DB1_IP, 30004)

# ============================================================
#  Custom Exception
# ============================================================
class TaskHaltedException(Exception):
    pass

# ============================================================
#  Global State
# ============================================================
current_actual_point = None
current_joints       = None
globalLockValue      = threading.Lock()

is_running     = True
is_halted      = False
current_detect = 0
conveyor_running = False

_feed_event = threading.Event()

# ============================================================
#  Dashboard Command Queue
# ============================================================
# All dashboard.DO / dashboard.DI / dashboard.RobotMode calls are
# routed through this queue so they execute on ONE dedicated thread,
# eliminating all lock contention on the serial/TCP connection.
#
# Priority levels (lower number = higher priority):
#   0  – Safety / emergency  (E-Stop, detection, shutdown)
#   1  – Task I/O            (suction, pusher from robot tasks)
#   2  – Polling             (IR, safety monitor, status)
#
# Each item: (priority, callable)          — for fire-and-forget calls
#        or: (priority, callable, result_q) — for calls that need a return value

_dashboard_queue: queue.PriorityQueue = queue.PriorityQueue()
_dashboard_seq = 0  # tie-breaker so callables are never compared
_dashboard_seq_lock = threading.Lock()


def _enqueue(priority: int, fn, result_q: queue.Queue = None):
    """Push a callable onto the dashboard executor queue."""
    global _dashboard_seq
    with _dashboard_seq_lock:
        seq = _dashboard_seq
        _dashboard_seq += 1
    _dashboard_queue.put((priority, seq, fn, result_q))


def dashboard_do(port: int, value: int, priority: int = 1):
    """Non-blocking fire-and-forget DO command."""
    _enqueue(priority, lambda: dashboard.DO(port, value))


def dashboard_di(port: int, priority: int = 2) -> any:
    """Blocking DI read — waits for the executor to return the result."""
    result_q: queue.Queue = queue.Queue()
    _enqueue(priority, lambda: dashboard.DI(port), result_q)
    return result_q.get()  # blocks until executor completes the call


def dashboard_robot_mode(priority: int = 2) -> any:
    """Blocking RobotMode read."""
    result_q: queue.Queue = queue.Queue()
    _enqueue(priority, lambda: dashboard.RobotMode(), result_q)
    return result_q.get()


def dashboard_misc(fn, priority: int = 1, blocking: bool = False):
    """
    Generic dashboard call wrapper.
    Use blocking=True when you need the return value or must wait for completion.
    """
    if blocking:
        result_q: queue.Queue = queue.Queue()
        _enqueue(priority, fn, result_q)
        return result_q.get()
    else:
        _enqueue(priority, fn)


def dashboard_executor():
    """
    Single-threaded worker that drains _dashboard_queue.
    This is the ONLY thread that ever touches the dashboard socket.
    """
    print(f"  🗄️  [{LABEL}] Dashboard executor started", flush=True)
    while is_running:
        try:
            priority, seq, fn, result_q = _dashboard_queue.get(timeout=0.2)
            try:
                result = fn()
                if result_q is not None:
                    result_q.put(result)
            except Exception as e:
                print(f"  ⚠️ [{LABEL}] Dashboard executor error: {e}", flush=True)
                if result_q is not None:
                    result_q.put(None)   # unblock callers even on error
        except queue.Empty:
            pass
    print(f"  🗄️  [{LABEL}] Dashboard executor stopped", flush=True)


# ============================================================
#  Startup
# ============================================================
def start_up():
    print(f"🔄 [{LABEL}] กำลังเตรียมความพร้อม...")
    dashboard.ClearError()
    time.sleep(0.5)
    dashboard.EnableRobot()
    time.sleep(3)
    dashboard.SpeedFactor(50)
    print(f"✅ [{LABEL}] READY!")


# ============================================================
#  Alarm helpers
# ============================================================
def turn_off_alarm():
    dashboard_do(DO_ALARM, 0, priority=0)


# ============================================================
#  Conveyor helpers
# ============================================================
def calc_conveyor_speed() -> float:
    circumference = math.pi * ROLLER_DIAMETER_MM
    rps = MOTOR_RPM / 60.0
    return round(rps * circumference, 2)


def conveyor_start():
    global conveyor_running
    if not conveyor_running:
        dashboard_do(DO_CONVEYOR, 1, priority=1)
        conveyor_running = True
        print(f"  🟢 [{LABEL}] Conveyor ON", flush=True)


def conveyor_stop():
    global conveyor_running
    if conveyor_running:
        dashboard_do(DO_CONVEYOR, 0, priority=1)
        conveyor_running = False
        print(f"  🔴 [{LABEL}] Conveyor OFF", flush=True)


# ============================================================
#  Emergency stop helper  (priority-0 burst, non-blocking)
# ============================================================
def emergency_stop_outputs():
    """Queue a burst of safety-critical DO commands at highest priority."""
    for port, val in [
        (DO_EMERG_LIGHT, 1),
        (DO_ALARM,       1),
        (DO_CONVEYOR,    0),
        (DO_SUCTION_ON,  0),
        (DO_SUCTION_OFF, 0),
        (DO_PUSHER,      0),
    ]:
        dashboard_do(port, val, priority=0)


# ============================================================
#  Combined IR / Conveyor / Pusher Pipeline  (single thread)
#
#  Runs sequentially:
#    1. Read IR sensor  →  control conveyor
#    2. Read pusher sensor  →  fire pusher if conditions met
#    3. Publish IR MQTT heartbeat once per second
#    4. Publish conveyor status on change
#
#  All DI reads go through the queue at priority=2 so they
#  never race with safety commands (priority=0) or task I/O (1).
# ============================================================
def io_pipeline():
    """Unified IR + Conveyor + Pusher polling loop (one thread)."""
    global conveyor_running

    print(f"  🔄 [{LABEL}] IO pipeline started (IR + Conveyor + Pusher)", flush=True)
    conveyor_start()

    last_ir_publish   = time.time()
    last_conv_state   = conveyor_running
    ir_blocked        = False
    pusher_busy       = False    # True while pusher extend/retract cycle is running
    pusher_busy_until = 0.0      # monotonic time when pusher cycle finishes

    while is_running:
        now = time.monotonic()

        if is_halted:
            time.sleep(0.2)
            continue

        # ── 1. Read both DI sensors in two sequential queue calls ────────────
        # Because the executor is single-threaded these two calls are
        # guaranteed to run back-to-back without interleaving with anything.
        raw_ir     = dashboard_di(DI_IR_SENSOR,     priority=2)
        raw_pusher = dashboard_di(DI_PUSHER_SENSOR, priority=2)

        # ── Parse IR (DI10) ──────────────────────────────────────────────────
        try:
            parts = str(raw_ir).strip().split(",")
            di_ir = float(parts[1].strip().strip("{}")) if len(parts) >= 2 else 0.0
        except Exception:
            di_ir = 0.0

        # ── Parse Pusher sensor (DI9) ────────────────────────────────────────
        try:
            parts = str(raw_pusher).strip().split(",")
            di_pusher = float(parts[1].strip().strip("{}")) if len(parts) >= 2 else 0.0
        except Exception:
            di_pusher = 0.0

        # ── 2. Conveyor control (based on IR) ────────────────────────────────
        if not is_halted:
            if di_ir == 0.0 and not conveyor_running:
                conveyor_start()
                ir_blocked = False
                print(f"  ✅ [{LABEL}] IR LOW → Conveyor ON", flush=True)
            elif di_ir > 0.0 and conveyor_running:
                conveyor_stop()
                ir_blocked = True
                print(f"  🚨 [{LABEL}] IR HIGH → Conveyor OFF", flush=True)

        # ── 3. Pusher control ────────────────────────────────────────────────
        # Conditions: DI9 HIGH + DI10 LOW + not in a pusher cycle + not halted
        if (
            not is_halted
            and not pusher_busy
            and di_pusher > 0.0
            and di_ir == 0.0
        ):
            pusher_busy = True
            print(f"  🟡 [{LABEL}] Pusher condition met → extending", flush=True)

            def _run_pusher():
                """Runs the pusher extend/retract cycle on its own short thread
                so the IO poll loop is not blocked for ~12 seconds."""
                nonlocal pusher_busy, pusher_busy_until
                try:
                    dashboard_do(DO_PUSHER, 1, priority=1)
                    time.sleep(5.5)   # PUSHER_ON_DURATION
                    dashboard_do(DO_PUSHER, 0, priority=1)
                    print(f"  ⬅️  [{LABEL}] Pusher retracting (6.5 s)", flush=True)
                    time.sleep(6.5)   # PUSHER_RETRACT_DELAY
                finally:
                    pusher_busy = False

            threading.Thread(target=_run_pusher, daemon=True, name="Pusher-Cycle").start()

        # ── 4. IR MQTT heartbeat (1 Hz) ──────────────────────────────────────
        wall_now = time.time()
        if wall_now - last_ir_publish >= 1.0:
            client.publish(
                TOPIC_IR,
                json.dumps({"detected": 1 if di_ir > 0.0 else 0}),
                qos=0,
            )
            last_ir_publish = wall_now

        # ── 5. Conveyor status on change ─────────────────────────────────────
        if conveyor_running != last_conv_state:
            status = "working" if conveyor_running else "idle"
            speed  = calc_conveyor_speed() if conveyor_running else 0
            client.publish(
                TOPIC_CONVEYOR1,
                json.dumps({"id": 1, "status": status, "speed": speed}),
                qos=2, retain=True,
            )
            print(f"  📢 [{LABEL}] Conveyor1 → {status} (speed={speed})", flush=True)
            last_conv_state = conveyor_running

        # ── Poll interval ─────────────────────────────────────────────────────
        # 80 ms gives adequate resolution; two blocking DI calls already
        # pace the loop naturally via the executor queue.
        time.sleep(0.08)


# ============================================================
#  Safety Monitor  (E-Stop / Robot Error)
# ============================================================
def safety_monitor_pipeline():
    global is_halted, conveyor_running
    print(f"  🛡️ [{LABEL}] Safety monitor started", flush=True)

    while is_running:
        try:
            raw_mode = dashboard_robot_mode(priority=2)
            parts = str(raw_mode).strip().split(",")
            if len(parts) >= 2:
                mode = int(parts[1].strip().strip("{}"))
                if mode == 9 and not is_halted:
                    print(f"\n🚨 [{LABEL}] E-STOP / ERROR → HALTING", flush=True)
                    is_halted = True
                    emergency_stop_outputs()
                    threading.Timer(3.0, turn_off_alarm).start()
                    conveyor_running = False
                    publish_status("halted")
        except Exception:
            pass

        time.sleep(0.5)


# ============================================================
#  Feedback thread
# ============================================================
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
                _feed_event.set()
        except Exception as e:
            print(f"  ⚠️ [{LABEL}] Feedback error: {e}")
            time.sleep(0.05)


def wait_arrive(target: list, tolerance: float = 10.0):
    print(f"  ⏳ [{LABEL}] Waiting to arrive at {target}")
    last_debug = 0.0
    while True:
        if is_halted:
            raise TaskHaltedException()
        _feed_event.wait(timeout=0.1)
        _feed_event.clear()
        if is_halted:
            raise TaskHaltedException()
        with globalLockValue:
            actual = current_actual_point
        if actual is None:
            continue
        if all(abs(actual[i] - target[i]) <= tolerance for i in range(4)):
            print(f"  ✨ [{LABEL}] Arrived!")
            return
        now = time.monotonic()
        if now - last_debug >= 2.0:
            print(f"     [DEBUG] Moving… current={actual}")
            last_debug = now


def movj_wait(point: list, description: str = ""):
    move.MovJ(*point)
    wait_arrive(point)


def movl_wait(point: list):
    move.MovL(*point)
    wait_arrive(point)


def suction_pick():
    if is_halted:
        raise TaskHaltedException()
    dashboard_do(DO_SUCTION_ON, 1, priority=1)
    steps = int(SUCTION_ENGAGE_DELAY / 0.1)
    for _ in range(steps):
        if is_halted:
            raise TaskHaltedException()
        time.sleep(0.1)
    dashboard_do(DO_SUCTION_ON, 0, priority=1)


def suction_release():
    if is_halted:
        raise TaskHaltedException()
    dashboard_do(DO_SUCTION_OFF, 1, priority=1)
    steps = int(SUCTION_RELEASE_DELAY / 0.1)
    for _ in range(steps):
        if is_halted:
            raise TaskHaltedException()
        time.sleep(0.1)
    dashboard_do(DO_SUCTION_OFF, 0, priority=1)


# ============================================================
#  Motion helpers
# ============================================================
def calc_pick_point(cam_x: float, cam_y: float) -> dict:
    abs_x = max(X_LIMITS[0], min(BASE_X + cam_x, X_LIMITS[1]))
    abs_y = max(Y_LIMITS[0], min(BASE_Y + cam_y, Y_LIMITS[1]))
    abs_r = BASE_R
    y_start, z_start = 237.31, -60.7
    y_end,   z_end   = 373.74, -63.46
    abs_z = z_start + (abs_y - y_start) * (z_end - z_start) / (y_end - y_start) + PICK_HEIGHT_OFFSET
    return {
        "pick_pos"    : [abs_x, abs_y, abs_z,      abs_r],
        "approach_pos": [abs_x, abs_y, abs_z + 20, abs_r],
        "lower_pos"   : [abs_x, abs_y, abs_z + 10, abs_r],
    }


def calc_sweep_point(cam_x: float, cam_y: float) -> dict:
    abs_x  = max(X_LIMITS[0], min(BASE_X + cam_x, X_LIMITS[1]))
    abs_y  = BASE_Y + cam_y
    abs_z  = BASE_Z + SWEEP_HEIGHT_OFFSET
    abs_r  = BASE_R
    if abs(abs_y - Y_LIMITS[1]) > abs(abs_y - Y_LIMITS[0]):
        y_start, y_end = Y_LIMITS[0], Y_LIMITS[1]
    else:
        y_start, y_end = Y_LIMITS[1], Y_LIMITS[0]
    return {
        "sweep_x": abs_x, "sweep_z": abs_z, "sweep_r": abs_r,
        "y_start": y_start, "y_end": y_end,
        "edge_pos": [abs_x, y_start, abs_z + 25, abs_r],
    }


def pick_and_place(to_conveyor: bool, cam_x: float, cam_y: float):
    pts    = calc_pick_point(cam_x, cam_y)
    target = CONVEYOR_POINT if to_conveyor else WASTE_POINT
    dest   = "Conveyor ✅" if to_conveyor else "Waste 🗑️"
    print(f"\n▶️ [{LABEL}] Pick & Place → {dest}")

    movj_wait(HOME_POINT, "Home")
    movj_wait(pts["approach_pos"], "Above Object")
    movl_wait(pts["lower_pos"])
    suction_pick()
    movl_wait(pts["approach_pos"])
    movj_wait(HOME_POINT, "Home Transit")
    movj_wait(target, f"Drop ({dest})")

    if to_conveyor:
        above = [CONVEYOR_POINT[0], CONVEYOR_POINT[1], CONVEYOR_POINT[2] - 29.5, CONVEYOR_POINT[3]]
        movl_wait(above)
        suction_release()
        movl_wait(target)
    else:
        suction_release()

    movj_wait(HOME_POINT, "Home complete")


def edge_to_edge_sweep(cam_x: float, cam_y: float):
    pts = calc_sweep_point(cam_x, cam_y)
    sx, sz, sr     = pts["sweep_x"], pts["sweep_z"], pts["sweep_r"]
    y_start, y_end = pts["y_start"], pts["y_end"]
    print(f"\n▶️ [{LABEL}] Sweep Y {y_start}→{y_end} at X={sx:.2f}")

    movj_wait(HOME_POINT, "Home")
    movj_wait(pts["edge_pos"], "Edge")
    movl_wait([sx, y_start, sz + 15, sr])
    movl_wait([sx, y_end,   sz + 15, sr])
    movl_wait([sx, y_end,   sz + 25, sr])
    movj_wait(HOME_POINT, "Home (sweep done)")


# ============================================================
#  Status publisher
# ============================================================
def publish_status(status: str, cmd_id: int = None, color: str = None):
    payload = {"status": status}
    if cmd_id  is not None: payload["id"]    = cmd_id
    if color   not in (None, ""): payload["color"] = color
    client.publish(TOPIC_STATUS, json.dumps(payload), qos=2, retain=True)
    print(f"  📢 [{LABEL}] Status → {status} (id={cmd_id}, color={color})", flush=True)


# ============================================================
#  MQTT Callbacks
# ============================================================
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected to MQTT: {reason_code}", flush=True)
    client.subscribe([(TOPIC_CMD, 2), (TOPIC_DETECTION, 0)])


def on_message(client, userdata, msg):
    pass   # per-topic callbacks handle everything


def handle_command(client, userdata, msg):
    global is_halted
    data   = json.loads(msg.payload.decode())
    cmd_id = data.get("id",     0)
    action = data.get("action", "")
    cam_x  = float(data.get("x", 0))
    cam_y  = float(data.get("y", 0))
    color  = data.get("color", "")

    if is_halted:
        print(f"  🚫 [{LABEL}] HALTED — command ignored.")
        return

    if action not in ("in", "out", "sweep"):
        return

    publish_status("working", cmd_id, color)

    def run_task():
        try:
            if   action == "in":    pick_and_place(True,  cam_x, cam_y)
            elif action == "out":   pick_and_place(False, cam_x, cam_y)
            elif action == "sweep": edge_to_edge_sweep(cam_x, cam_y)
            publish_status("idle", cmd_id, color)
        except TaskHaltedException:
            print(f"  🛑 [{LABEL}] Task '{action}' aborted (halt)", flush=True)

    threading.Thread(target=run_task, daemon=True, name=f"Task-{action}").start()


def handle_detection(client, userdata, msg):
    global is_halted, current_detect, conveyor_running
    data             = json.loads(msg.payload.decode())
    incoming_detect  = int(data.get("detected", 0))

    if incoming_detect == current_detect:
        return
    current_detect = incoming_detect

    if current_detect == 1 and not is_halted:
        print(f"\n🚨 [{LABEL}] Human detected → HALT", flush=True)
        is_halted = True
        emergency_stop_outputs()
        dashboard_misc(lambda: dashboard.PauseRobot(), priority=0, blocking=False)
        dashboard_misc(lambda: dashboard.ClearError(),  priority=0, blocking=False)
        threading.Timer(3.0, turn_off_alarm).start()
        conveyor_running = False
        publish_status("halted")

    elif current_detect == 0 and is_halted:
        print(f"\n✅ [{LABEL}] Detection cleared → RESUMING", flush=True)
        dashboard_misc(lambda: dashboard.ClearError(), priority=0, blocking=True)
        time.sleep(0.5)
        dashboard_do(DO_EMERG_LIGHT, 0, priority=0)
        dashboard_do(DO_ALARM,       0, priority=0)
        move.MovJ(*HOME_POINT)
        is_halted = False
        publish_status("idle")


# ============================================================
#  Sim worker
# ============================================================
def sim_worker():
    if not is_running:
        return
    try:
        with globalLockValue:
            joints = current_joints
        if joints:
            payload = {"j1": joints[0], "j2": joints[1], "j3": joints[2], "j4": joints[3]}
            client.publish(TOPIC_SIM, json.dumps(payload), qos=0)
    except Exception:
        pass
    threading.Timer(SIM_INTERVAL, sim_worker).start()


# ============================================================
#  MQTT Client
# ============================================================
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.message_callback_add(TOPIC_CMD,       handle_command)
client.message_callback_add(TOPIC_DETECTION, handle_detection)
client.on_connect = on_connect
client.on_message = on_message

# ============================================================
#  Entry Point
# ============================================================
if __name__ == "__main__":
    start_up()

    # Start dashboard executor FIRST — everything else depends on it
    threading.Thread(target=dashboard_executor, daemon=True, name="DashboardExecutor").start()

    # Combined IO pipeline replaces the three separate threads
    threading.Thread(target=io_pipeline,            daemon=True, name="IO-Pipeline").start()
    threading.Thread(target=get_feed,               daemon=True, name="FeedThread").start()
    threading.Thread(target=safety_monitor_pipeline,daemon=True, name="SafetyMonitor").start()
    threading.Thread(target=sim_worker,             daemon=True).start()

    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_start()

    publish_status("idle")

    print("\n" + "=" * 55)
    print(f"  [{LABEL}] Listening on MQTT — waiting for commands…")
    print(f"  DETECT STATE → {current_detect} (0=Safe, 1=Halted)")
    print("  Ctrl+C to exit")
    print("=" * 55)

    try:
        while is_running:
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n⌨️  Shutting down…")
    finally:
        is_running = False
        conveyor_stop()
        dashboard_do(DO_EMERG_LIGHT, 0, priority=0)
        dashboard_do(DO_ALARM,       0, priority=0)
        time.sleep(0.3)   # let the executor drain the final commands
        client.loop_stop()
        client.disconnect()
        print("🔌 ปิดการทำงานเรียบร้อย")