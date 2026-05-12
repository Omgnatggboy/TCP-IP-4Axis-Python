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
TOPIC_CMD       = "phitt-f/db2/command"     
TOPIC_STATUS    = "phitt-f/db2/status"      
TOPIC_DETECTION = "phitt-f/detection/data"  
TOPIC_SIM       = "phitt-f/sim/db2"         
TOPIC_IR        = "phitt-f/ir/data"         
TOPIC_CONVEYOR2 = "phitt-f/conveyor2/status" 

# Robot Configuration
DB2_IP = "192.168.1.6"
LABEL  = "DB2"

DO_PUSHER = 1
DO_CONVEYOR    = 2
DO_SUCTION_ON  = 9
DO_SUCTION_OFF = 10
DO_EMERG_LIGHT = 5
DO_ALARM       = 6   # 🌟 เพิ่มขา DO สำหรับเสียงเตือน

SUCTION_ENGAGE_DELAY  = 0.5
SUCTION_RELEASE_DELAY = 0.5
SIM_INTERVAL          = 0.1  
IR_POLL_INTERVAL      = 0.05
DB2_MOCK_COMMAND      = False
MOCK_COMMAND_INTERVAL = float(os.getenv("DB2_MOCK_INTERVAL", "5.0"))

CONVEYOR_SPEED = 200

# Waypoints  (Cartesian: X, Y, Z, R)
HOME_POINT      = [189.79,   182.00,   115.86,   0]
PICK_POINT      = [202.15,   311.88,  -45.68, 0] 
CONVEYOR_POINT  = [350.40,   15.79,  -12.94,   0] 

# Robot API
dashboard = DobotApiDashboard(DB2_IP, 29999)
move      = DobotApiMove(DB2_IP, 30003)
feed      = DobotApi(DB2_IP, 30004)

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

_task_lock = threading.Lock()
_task_running = False

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

# 🌟 ฟังก์ชันสำหรับปิดเสียง (จะถูกเรียกอัตโนมัติเมื่อครบ 3 วินาที)
def turn_off_alarm():
    try:
        with dashboard_lock:
            dashboard.DO(DO_ALARM, 0)
    except Exception as e:
        pass

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
                        #dashboard.DO(DO_ALARM, 1)       # 🌟 สั่งเปิดเสียงเตือน
                        
                        dashboard.DO(DO_CONVEYOR, 0)
                        dashboard.DO(DO_SUCTION_ON, 0)
                        dashboard.DO(DO_SUCTION_OFF, 0)
                        dashboard.DO(DO_PUSHER, 0)
                    
                    # 🌟 ตั้งเวลาให้ปิดเสียงอัตโนมัติในอีก 3 วินาที
                    threading.Timer(3.0, turn_off_alarm).start()
                    
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
                client.publish(TOPIC_CONVEYOR2, json.dumps(payload), qos=2, retain=True)
                print(f"  📢 [{LABEL}] Conveyor2 Status → {status} (Speed: {speed})", flush=True)
                last_state = current_state 
                
        except Exception as e:
            pass
            
        time.sleep(0.1)

def calc_conveyor_speed():
    MOTOR_RPM = 60.0          
    ROLLER_DIAMETER_MM = 40.0 
    circumference = math.pi * ROLLER_DIAMETER_MM
    rps = MOTOR_RPM / 60.0
    speed_mms = rps * circumference
    return round(speed_mms, 2) 

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
    print(f"\n▶️ [{LABEL}] Moving to {description} → Point: {point}")
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

# ---------------------------------------------------------------------------
# Pusher stroke
# ---------------------------------------------------------------------------
PUSHER_DURATION = 5  
_pusher_available = True
_pusher_lock = threading.Lock()

def pusher_stroke():
    global _pusher_available
    with _pusher_lock:
        _pusher_available = False
    with dashboard_lock:
        dashboard.DO(DO_PUSHER, 1)
    print(f"  ➡️  [{LABEL}] Pusher ON  (will auto-off in {PUSHER_DURATION}s)", flush=True)
    threading.Timer(PUSHER_DURATION, _pusher_off).start()

def _pusher_off():
    global _pusher_available
    with dashboard_lock:
        dashboard.DO(DO_PUSHER, 0)
    print(f"  ⬅️  [{LABEL}] Pusher OFF", flush=True)
    time.sleep(3.0)
    with _pusher_lock:
        _pusher_available = True

def wait_pusher_finished():
    print(f"  ⏳ [{LABEL}] Waiting for pusher to complete...", flush=True)
    while True:
        if is_halted: raise TaskHaltedException() 
        with _pusher_lock:
            if _pusher_available:
                break
        time.sleep(0.1)
    print(f"  ✨ [{LABEL}] Pusher cycle complete!", flush=True)

def conveyer_start_stop():
    global conveyor_running
    if is_halted: raise TaskHaltedException()
    
    with dashboard_lock:
        dashboard.DO(DO_CONVEYOR, 1)
    conveyor_running = True
    
    steps = int(1.5 / 0.1)
    for _ in range(steps):
        if is_halted: raise TaskHaltedException() 
        time.sleep(0.1)
        
    with dashboard_lock:
        dashboard.DO(DO_CONVEYOR, 0)
    conveyor_running = False

# ---------------------------------------------------------------------------
# Task
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
        movj_wait(approach_pos,        "Above Pick Point (prepare)")
        movl_wait(lower_pos)
        suction_pick()

        movl_wait(approach_pos)
        movj_wait(HOME_POINT, "Home  (Transit)")

        movj_wait(target, f"Drop Point ({dest})")
        suction_release()
        movj_wait(HOME_POINT, "Home  (Task complete)")

        #pusher_stroke()  
        #wait_pusher_finished()  
        conveyer_start_stop()
    
    finally:
        with _task_lock:
            _task_running = False

# MQTT Helpers
def publish_status(status: str, cmd_id: int = None):
    payload = {"status": status}
    if cmd_id is not None:
        payload["id"] = cmd_id
    client.publish(TOPIC_STATUS, json.dumps(payload), qos=2, retain=True)
    print(f"  📢 [{LABEL}] Status → {status}", flush=True)

# MQTT Callbacks
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected to MQTT: {reason_code}", flush=True)
    topics_to_subscribe = [
        (TOPIC_CMD,       2),   
        (TOPIC_DETECTION, 0),   
    ]
    client.subscribe(topics_to_subscribe)

def on_message(client, userdata, msg):
    pass 

def handle_command(client, userdata, msg):
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
        return

    publish_status("working", cmd_id)

    def run_task():
        try:
            pick_and_place_to_conveyor()
            publish_status("idle", cmd_id)
        except TaskHaltedException:
            print(f"  🛑 [{LABEL}] TASK ABORTED: ยกเลิกคำสั่งกลางคัน เนื่องจากตรวจพบคน", flush=True)

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
                #dashboard.DO(DO_ALARM, 1)       # 🌟 สั่งเปิดเสียงเตือน
                
                dashboard.DO(DO_CONVEYOR, 0)
                dashboard.DO(DO_SUCTION_ON, 0)
                dashboard.DO(DO_SUCTION_OFF, 0)
                dashboard.DO(DO_PUSHER, 0)     
                
                dashboard.PauseRobot()         
                dashboard.ClearError()
            
            # 🌟 ตั้งเวลาให้ปิดเสียงอัตโนมัติในอีก 3 วินาที
            #threading.Timer(3.0, turn_off_alarm).start()
            
            conveyor_running = False
            publish_status("halted")

        elif current_detect == 0 and is_halted:
            print(f"\n✅ [{LABEL}] Detection cleared (1->0) → RESETTING TO HOME", flush=True)
            
            with dashboard_lock:
                dashboard.ClearError()         
                time.sleep(0.5) 
                dashboard.DO(DO_EMERG_LIGHT, 0)  
                dashboard.DO(DO_ALARM, 0)       # 🌟 ปิดเสียงซ้ำอีกทีเพื่อความชัวร์ เมื่อระบบกลับมาปกติ
            
            move.MovJ(HOME_POINT[0], HOME_POINT[1], HOME_POINT[2], HOME_POINT[3])
                
            is_halted = False
            publish_status("idle")

def mock_command_sender():
    if not DB2_MOCK_COMMAND: return
    while is_running:
        time.sleep(MOCK_COMMAND_INTERVAL)
        if is_halted: continue
        
        print(f"\n🧪 [{LABEL}] MOCK OFFLINE: จำลองส่งคำสั่งโดยไม่ผ่านเซิร์ฟเวอร์", flush=True)
        if not _task_running:
            threading.Thread(target=pick_and_place_to_conveyor, daemon=True).start()

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

    threading.Thread(target=get_feed, daemon=True, name="FeedThread").start()
    threading.Thread(target=conveyor_status_pipeline, daemon=True, name="ConveyorStatus").start()
    threading.Thread(target=sim_worker, daemon=True).start()
    threading.Thread(target=safety_monitor_pipeline, daemon=True, name="SafetyMonitor").start()

    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_start()

    if DB2_MOCK_COMMAND:
        threading.Thread(target=mock_command_sender, daemon=True).start()

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
        
        with dashboard_lock:
            dashboard.DO(DO_CONVEYOR, 0)
            dashboard.DO(DO_EMERG_LIGHT, 0)
            dashboard.DO(DO_ALARM, 0)
            
        client.loop_stop()
        client.disconnect()
        print("🔌 ปิดการทำงานเรียบร้อย")