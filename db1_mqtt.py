import os
import time
import json
import ssl
import numpy as np
import threading
import math
import paho.mqtt.client as mqtt
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType

# MQTT Configuration
MQTT_HOST = "10.35.120.100"
MQTT_PORT = 1883

# Topics
TOPIC_CMD       = "phitt-f/db1/command"     
TOPIC_STATUS    = "phitt-f/db1/status"      
TOPIC_DETECTION = "phitt-f/detection/data"  
TOPIC_SIM       = "phitt-f/sim/db1"         
TOPIC_IR        = "phitt-f/ir/data"             
TOPIC_CONVEYOR1 = "phitt-f/conveyor1/status"

# Robot Configuration
DB1_IP = "192.168.2.6"
LABEL  = "DB1"
PICK_HEIGHT_OFFSET = 0
SWEEP_HEIGHT_OFFSET = 5

DO_PUSHER      = 1
DO_CONVEYOR    = 2
DO_EMERG_LIGHT = 5
DO_ALARM       = 6  # 🌟 เพิ่มขา DO สำหรับเสียงเตือน (แก้เลขตามที่ต่อฮาร์ดแวร์จริง)
DO_SUCTION_ON  = 9
DO_SUCTION_OFF = 10

DI_IR_SENSOR   = 10

SUCTION_ENGAGE_DELAY  = 0.5
SUCTION_RELEASE_DELAY = 1.0
SIM_INTERVAL          = 0.1  
IR_POLL_INTERVAL      = 0.08

# Waypoints  (Cartesian: X, Y, Z, R)
HOME_POINT     = [189.79,   182.00,   98.86,  51.42]
CONVEYOR_POINT = [354.30,   -52.16,  162.59,  67.61]
WASTE_POINT    = [  2.60,  -294.58,  -20.21,   9.38]

BASE_X, BASE_Y, BASE_Z, BASE_R = 149.21, 237.31, -61.00, 68.67
X_LIMITS = [150.01, 257.24]
Y_LIMITS = [240.21, 377.44]

# Robot API
dashboard = DobotApiDashboard(DB1_IP, 29999)
move      = DobotApiMove(DB1_IP, 30003)
feed      = DobotApi(DB1_IP, 30004)

CONVEYOR_SPEED = 200  

# Global State
current_actual_point = None   
current_joints       = None   
globalLockValue      = threading.Lock()
dashboard_lock       = threading.Lock()  
is_running           = True

current_detect       = 0      
is_halted            = False  

_feed_event = threading.Event()

conveyor_running = False   
ir_blocked       = False   

# เพิ่ม Custom Exception สำหรับยกเลิกงานกะทันหัน
class TaskHaltedException(Exception):
    pass

# Startup
def start_up():
    print(f"🔄 [{LABEL}] กำลังเตรียมความพร้อม...")
    dashboard.ClearError()
    time.sleep(0.5)          
    dashboard.EnableRobot()
    time.sleep(3)            
    dashboard.SpeedFactor(50)
    print(f"✅ [{LABEL}] READY!")

# 🌟 1. ฟังก์ชันสำหรับปิดเสียง (จะถูกเรียกอัตโนมัติเมื่อครบ 3 วินาที)
def turn_off_alarm():
    try:
        with dashboard_lock:
            dashboard.DO(DO_ALARM, 0)
    except Exception as e:
        pass

# ---------------------------------------------------------------------------
# Conveyor Control helpers
# ---------------------------------------------------------------------------
def conveyor_start():
    global conveyor_running
    if not conveyor_running:
        with dashboard_lock: 
            dashboard.DO(DO_CONVEYOR, 1)
        conveyor_running = True
        print(f"  🟢 [{LABEL}] Conveyor ON", flush=True)

def conveyor_stop():
    global conveyor_running
    if conveyor_running:
        with dashboard_lock: 
            dashboard.DO(DO_CONVEYOR, 0)
        conveyor_running = False
        print(f"  🔴 [{LABEL}] Conveyor OFF", flush=True)

def calc_conveyor_speed():
    MOTOR_RPM = 60.0          
    ROLLER_DIAMETER_MM = 40.0 
    circumference = math.pi * ROLLER_DIAMETER_MM
    rps = MOTOR_RPM / 60.0
    speed_mms = rps * circumference
    return round(speed_mms, 2) 

# ---------------------------------------------------------------------------
# IR + Conveyor pipeline
# ---------------------------------------------------------------------------
def ir_conveyor_pipeline():
    global ir_blocked
    print(f"  🔍 [{LABEL}] IR/Conveyor pipeline started (poll={IR_POLL_INTERVAL}s)", flush=True)

    conveyor_start()
    last_publish = time.time()

    while is_running:
        try:
            if is_halted:
                time.sleep(0.5)
                continue

            with dashboard_lock:  
                raw = dashboard.DI(DI_IR_SENSOR)
                
            raw_str = str(raw).strip()
            if not raw_str:
                di_val = 0.0
            else:
                parts = raw_str.split(",")
                di_val_str = parts[1].strip().strip("{}") if len(parts) >= 2 else "0"
                di_val = float(di_val_str) if di_val_str else 0.0

            should_conveyor_run = (not is_halted) and (di_val == 0.0)

            if should_conveyor_run and not conveyor_running:
                conveyor_start()
                ir_blocked = False
                print(f"  ✅ [{LABEL}] IR LOW / Safe → Conveyor ON", flush=True)
                
            elif not should_conveyor_run and conveyor_running:
                conveyor_stop()
                if di_val > 0.0: 
                    ir_blocked = True 
                    print(f"  🚨 [{LABEL}] IR HIGH → Conveyor OFF", flush=True)

            now = time.time()
            if now - last_publish >= 1.0:
                client.publish(TOPIC_IR, json.dumps({"detected": 1 if di_val > 0.0 else 0}), qos=0)
                last_publish = now

        except Exception as e:
            print(f"  ⚠️ [{LABEL}] IR pipeline error: {e}", flush=True)

        time.sleep(IR_POLL_INTERVAL)

# ---------------------------------------------------------------------------
# Safety Monitor — เช็คการกดปุ่ม E-Stop หรือ Error ของหุ่นยนต์
# ---------------------------------------------------------------------------
def safety_monitor_pipeline():
    global is_halted, conveyor_running
    print(f"  🛡️ [{LABEL}] Safety (E-Stop) monitor started", flush=True)
    
    while is_running:
        try:
            with dashboard_lock:
                raw_mode = dashboard.RobotMode()
            
            parts = str(raw_mode).strip().split(",")
            if len(parts) >= 2:
                mode = int(parts[1].strip().strip("{}"))
                
                if mode == 9 and not is_halted:
                    print(f"\n🚨 [{LABEL}] E-STOP OR ERROR DETECTED! → HALTING SYSTEM", flush=True)
                    is_halted = True
                    
                    with dashboard_lock:
                        dashboard.DO(DO_EMERG_LIGHT, 1)
                        dashboard.DO(DO_ALARM, 1)       # 🌟 2. สั่งเปิดเสียงเตือนเมื่อกด E-Stop
                        
                        dashboard.DO(DO_CONVEYOR, 0)
                        dashboard.DO(DO_SUCTION_ON, 0)
                        dashboard.DO(DO_SUCTION_OFF, 0)
                        dashboard.DO(DO_PUSHER, 0)
                    
                    # 🌟 ตั้งเวลาให้ปิดเสียงอัตโนมัติในอีก 3 วินาที
                    threading.Timer(3.0, turn_off_alarm).start()
                    #time.sleep(3.0)  # รอให้ระบบหยุดนิ่งก่อนที่จะประกาศสถานะ halted
                    #turn_off_alarm()  # ปิดเสียงเตือนหลังจากครบ 3 วินาที
                    
                    conveyor_running = False
                    publish_status("halted")
                    
        except Exception as e:
            pass
            
        time.sleep(0.5)

# ---------------------------------------------------------------------------
# Conveyor Status Publisher
# ---------------------------------------------------------------------------
def conveyor_status_pipeline():
    print(f"  📡 [{LABEL}] Conveyor status publisher started", flush=True)
    last_state = None
    
    while is_running:
        try:
            current_state = conveyor_running 
            
            if current_state != last_state:
                status = "working" if current_state else "idle"
                speed = calc_conveyor_speed() if current_state else 0
                
                payload = {"id": 1, "status": status, "speed": speed}
                client.publish(TOPIC_CONVEYOR1, json.dumps(payload), qos=2, retain=True)
                print(f"  📢 [{LABEL}] Conveyor1 Status → {status} (Speed: {speed})", flush=True)
                last_state = current_state 
                
        except Exception as e:
            print(f"  ⚠️ [{LABEL}] Conveyor status publisher error: {e}", flush=True)
            
        time.sleep(0.1)

# ---------------------------------------------------------------------------
# Feedback — background thread
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
                _feed_event.set()   
        except Exception as e:
            print(f"  ⚠️ [{LABEL}] Feedback error: {e}")
            time.sleep(0.05)

def wait_arrive(target: list, tolerance: float = 10.0):
    print(f"  ⏳ [{LABEL}] Waiting to arrive at Point: {target}")
    last_debug = 0.0
    while True:
        if is_halted: raise TaskHaltedException()
        
        _feed_event.wait(timeout=0.1)
        _feed_event.clear()

        if is_halted: raise TaskHaltedException()

        with globalLockValue:
            actual = current_actual_point

        if actual is None:
            continue

        if all(abs(actual[i] - target[i]) <= tolerance for i in range(4)):
            print(f"  ✨ [{LABEL}] Arrived at destination!")
            return

        now = time.monotonic()
        if now - last_debug >= 2.0:
            print(f"     [DEBUG] Still moving... Current Point: {actual}")
            last_debug = now

def movj_wait(point: list, description: str = ""):
    x, y, z, r = point
    move.MovJ(x, y, z, r)
    wait_arrive(point)

def movl_wait(point: list):
    x, y, z, r = point
    move.MovL(x, y, z, r)
    wait_arrive(point)

def suction_pick():
    if is_halted: raise TaskHaltedException()
    with dashboard_lock:  
        dashboard.DO(DO_SUCTION_ON, 1)
    
    steps = int(SUCTION_ENGAGE_DELAY / 0.1)
    for _ in range(steps):
        if is_halted: raise TaskHaltedException()
        time.sleep(0.1)

    with dashboard_lock:
        dashboard.DO(DO_SUCTION_ON, 0)

def suction_release():
    if is_halted: raise TaskHaltedException()
    with dashboard_lock:  
        dashboard.DO(DO_SUCTION_OFF, 1)
        
    steps = int(SUCTION_RELEASE_DELAY / 0.1)
    for _ in range(steps):
        if is_halted: raise TaskHaltedException()
        time.sleep(0.1)

    with dashboard_lock:
        dashboard.DO(DO_SUCTION_OFF, 0)

def calc_pick_point(cam_x: float, cam_y: float) -> dict:
    abs_x = BASE_X + cam_x
    abs_y = BASE_Y + cam_y
    abs_r = BASE_R  
    abs_x = max(X_LIMITS[0], min(abs_x, X_LIMITS[1]))
    abs_y = max(Y_LIMITS[0], min(abs_y, Y_LIMITS[1]))

    y_start, z_start = 237.31, -60.7
    y_end, z_end = 373.74, -63.46
    z_interpolated = z_start + (abs_y - y_start) * (z_end - z_start) / (y_end - y_start)
    abs_z = z_interpolated + PICK_HEIGHT_OFFSET
    return {
        "pick_pos"    : [abs_x, abs_y, abs_z,      abs_r],
        "approach_pos": [abs_x, abs_y, abs_z + 20, abs_r],
        "lower_pos"   : [abs_x, abs_y, abs_z + 10, abs_r],
    }

def calc_sweep_point(cam_x: float, cam_y: float) -> dict:
    abs_x = BASE_X + cam_x
    abs_y = BASE_Y + cam_y
    abs_z = BASE_Z + SWEEP_HEIGHT_OFFSET
    abs_r = BASE_R
    sweep_x = max(X_LIMITS[0], min(abs_x, X_LIMITS[1]))

    if abs(abs_y - Y_LIMITS[1]) > abs(abs_y - Y_LIMITS[0]):
        y_start, y_end = Y_LIMITS[0], Y_LIMITS[1]
    else:
        y_start, y_end = Y_LIMITS[1], Y_LIMITS[0]

    return {
        "sweep_x" : sweep_x, "sweep_z" : abs_z, "sweep_r" : abs_r,
        "y_start" : y_start, "y_end"   : y_end,
        "edge_pos": [sweep_x, y_start, abs_z + 25, abs_r],
    }

def pick_and_place(to_conveyor: bool, cam_x: float, cam_y: float):
    pts    = calc_pick_point(cam_x, cam_y)
    target = CONVEYOR_POINT if to_conveyor else WASTE_POINT
    dest   = "Conveyor ✅" if to_conveyor else "Waste 🗑️"
    print(f"\n▶️ [{LABEL}] START: Pick and Place → {dest}")

    movj_wait(HOME_POINT,          "Home")
    movj_wait(pts["approach_pos"], "Above Object")
    movl_wait(pts["lower_pos"])
    suction_pick()
    movl_wait(pts["approach_pos"])
    movj_wait(HOME_POINT, "Home Transit")
    movj_wait(target, f"Drop Point ({dest})")
    
    if to_conveyor:
        above = [CONVEYOR_POINT[0], CONVEYOR_POINT[1], CONVEYOR_POINT[2] - 29.5, CONVEYOR_POINT[3]]
        movl_wait(above)
        suction_release()
        movl_wait(target)
    else:
        suction_release()
    movj_wait(HOME_POINT, "Home Task complete")

def edge_to_edge_sweep(cam_x: float, cam_y: float):
    pts = calc_sweep_point(cam_x, cam_y)
    sx, sz, sr       = pts["sweep_x"], pts["sweep_z"], pts["sweep_r"]
    y_start, y_end   = pts["y_start"], pts["y_end"]
    print(f"\n▶️ [{LABEL}] START: Sweep  Y {y_start} → {y_end}  at X={sx:.2f}")

    movj_wait(HOME_POINT,       "Home")
    movj_wait(pts["edge_pos"],  "Sweep Edge")
    movl_wait([sx, y_start, sz + 15, sr])
    movl_wait([sx, y_end, sz + 15, sr])
    movl_wait([sx, y_end, sz + 25, sr])
    movj_wait(HOME_POINT, "Home (Sweep finished)")

# ---------------------------------------------------------------------------
# Pusher Configuration & Pipeline
# ---------------------------------------------------------------------------
DI_PUSHER_SENSOR   = 9      
PUSHER_ON_DURATION = 5.5    
PUSHER_RETRACT_DELAY = 6.5  
PUSHER_POLL_INTERVAL = 0.05 

def pusher_pipeline():
    print(f"  ➡️  [{LABEL}] Pusher pipeline started (poll={PUSHER_POLL_INTERVAL}s)", flush=True)
    while is_running:
        try:
            if is_halted:
                time.sleep(0.5)
                continue

            with dashboard_lock:  
                raw9   = dashboard.DI(DI_PUSHER_SENSOR)
                raw10  = dashboard.DI(DI_IR_SENSOR)

            raw9_text = str(raw9).strip()
            parts9 = raw9_text.split(",") if raw9_text else []
            di9_val = parts9[1].strip().strip("{}") if len(parts9) >= 2 else ""
            di9 = float(di9_val) if di9_val else 0.0

            raw10_text = str(raw10).strip()
            parts10 = raw10_text.split(",") if raw10_text else []
            di10_val = parts10[1].strip().strip("{}") if len(parts10) >= 2 else ""
            di10 = float(di10_val) if di10_val else 0.0

            if di9 > 0.0 and di10 == 0.0 and not is_halted:
                print(f"  🟡 [{LABEL}] DI9 HIGH + DI10 LOW → Pusher ON  ({PUSHER_ON_DURATION}s)", flush=True)
                with dashboard_lock:
                    dashboard.DO(DO_PUSHER, 1)
                time.sleep(PUSHER_ON_DURATION)

                with dashboard_lock:
                    dashboard.DO(DO_PUSHER, 0)
                print(f"  ⬅️  [{LABEL}] Pusher OFF — retracting ({PUSHER_RETRACT_DELAY}s)", flush=True)
                time.sleep(PUSHER_RETRACT_DELAY)
                continue

            elif di9 > 0.0 and di10 > 0.0:
                time.sleep(PUSHER_POLL_INTERVAL)
            else:
                time.sleep(PUSHER_POLL_INTERVAL)

        except Exception as e:
            print(f"  ⚠️ [{LABEL}] Pusher pipeline error: {e}", flush=True)
            time.sleep(PUSHER_POLL_INTERVAL)

def publish_status(status: str, cmd_id: int = None, color: str = None):
    payload = {"status": status}
    
    if cmd_id is not None:
        payload["id"] = cmd_id
        
    if color is not None and color != "":
        payload["color"] = color
        
    client.publish(TOPIC_STATUS, json.dumps(payload), qos=2, retain=True)
    print(f"  📢 [{LABEL}] Status → {status} (ID: {cmd_id}, Color: {color})", flush=True)

# ---------------------------------------------------------------------------
# MQTT Callbacks
# ---------------------------------------------------------------------------
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected to MQTT: {reason_code}", flush=True)
    topics_to_subscribe = [
        (TOPIC_CMD,       2),   
        (TOPIC_DETECTION, 0),   
    ]
    client.subscribe(topics_to_subscribe)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()

def handle_command(client, userdata, msg):
    global is_halted
    data   = json.loads(msg.payload.decode())
    
    cmd_id = data.get("id", 0)
    action = data.get("action", "")
    cam_x  = float(data.get("x", 0))
    cam_y  = float(data.get("y", 0))
    color  = data.get("color", "")

    if is_halted:
        print(f"  🚫 [{LABEL}] HALTED (person detected). Command ignored.")
        return

    if action not in ("in", "out", "sweep"):
        return

    publish_status("working", cmd_id, color)

    def run_task():
        try:
            if action == "in": pick_and_place(True, cam_x, cam_y)
            elif action == "out": pick_and_place(False, cam_x, cam_y)
            elif action == "sweep": edge_to_edge_sweep(cam_x, cam_y)
            
            publish_status("idle", cmd_id, color)
            
        except TaskHaltedException:
            print(f"  🛑 [{LABEL}] TASK ABORTED: ยกเลิกคำสั่ง {action} กลางคัน เนื่องจากตรวจพบคน", flush=True)

    threading.Thread(target=run_task, daemon=True).start()

def handle_detection(client, userdata, msg):
    global is_halted, current_detect, conveyor_running 
    data = json.loads(msg.payload.decode())
    incoming_detect = int(data.get("detected", 0))

    if incoming_detect != current_detect:
        current_detect = incoming_detect

        if current_detect == 1 and not is_halted:
            print(f"\n🚨 [{LABEL}] Human detected (0->1) → HALTING ALL DEVICES IMMEDIATELY", flush=True)
            is_halted = True 
            
            with dashboard_lock:
                dashboard.DO(DO_EMERG_LIGHT, 1)  
                dashboard.DO(DO_ALARM, 1)       # 🌟 3. สั่งเปิดเสียงเมื่อเซนเซอร์เจอคน
                
                dashboard.DO(DO_CONVEYOR, 0)
                dashboard.DO(DO_SUCTION_ON, 0)
                dashboard.DO(DO_SUCTION_OFF, 0)
                dashboard.DO(DO_PUSHER, 0)     
                
                dashboard.PauseRobot()         
                dashboard.ClearError()
            
            # 🌟 ตั้งเวลาปิดเสียง 3 วินาที
            threading.Timer(3.0, turn_off_alarm).start()
            
            conveyor_running = False
            publish_status("halted")

        elif current_detect == 0 and is_halted:
            print(f"\n✅ [{LABEL}] Detection cleared (1->0) → RESETTING TO HOME", flush=True)
            
            with dashboard_lock:
                dashboard.ClearError()         
                time.sleep(0.5) 
                dashboard.DO(DO_EMERG_LIGHT, 0)  
                dashboard.DO(DO_ALARM, 0)       # 🌟 4. สั่งปิดเสียงซ้ำเพื่อความปลอดภัยเมื่อระบบกลับมาปกติ
            
            move.MovJ(HOME_POINT[0], HOME_POINT[1], HOME_POINT[2], HOME_POINT[3])
                
            is_halted = False
            publish_status("idle")

# ---------------------------------------------------------------------------
# Sim worker
# ---------------------------------------------------------------------------
def sim_worker():
    if not is_running: return
    try:
        with globalLockValue: joints = current_joints
        if joints:
            payload = {"j1": joints[0], "j2": joints[1], "j3": joints[2], "j4": joints[3]}
            client.publish(TOPIC_SIM, json.dumps(payload), qos=0)
    except Exception as e:
        pass
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

    threading.Thread(target=ir_conveyor_pipeline, daemon=True, name="IR-Conveyor").start()
    threading.Thread(target=get_feed, daemon=True, name="FeedThread").start()
    threading.Thread(target=conveyor_status_pipeline, daemon=True, name="ConveyorStatus").start()
    threading.Thread(target=pusher_pipeline, daemon=True, name="Pusher").start()
    threading.Thread(target=sim_worker, daemon=True).start()
    threading.Thread(target=safety_monitor_pipeline, daemon=True, name="SafetyMonitor").start()

    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_start()

    publish_status("idle")

    print("\n" + "="*55)
    print(f"  [{LABEL}] Listening on MQTT — waiting for commands...")
    print(f"  DETECT STATE → {current_detect} (0=Safe, 1=Halted)")
    print("  Ctrl+C to exit")
    print("="*55)

    try:
        while is_running:
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n⌨️  KeyboardInterrupt — shutting down...")
    finally:
        is_running = False
        conveyor_stop()        
        
        # 🌟 5. เพิ่มคำสั่งปิดไฟและปิดเสียง ก่อนที่จะออกจากโปรแกรม
        with dashboard_lock:
            dashboard.DO(DO_EMERG_LIGHT, 0)
            #dashboard.DO(DO_ALARM, 0)
            
        client.loop_stop()
        client.disconnect()
        print("🔌 ปิดการทำงานเรียบร้อย")