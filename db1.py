import time
import json
import ssl
import numpy as np
import threading
import paho.mqtt.client as mqtt
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType

# --- Global Variables สำหรับสถานะหุ่นยนต์ ---
current_actual = None        # เก็บค่า J1, J2, J3, J4
current_actual_point = None  # เก็บค่า X, Y, Z, R
globalLockValue = threading.Lock()
is_running = True

# =============================================================================
# Configuration
# =============================================================================
DB1_IP = "192.168.2.6"
LABEL  = "DB1"

# --- API Instances ---
dashboard = DobotApiDashboard(DB1_IP, 29999)
move      = DobotApiMove(DB1_IP, 30003)
feedback  = DobotApi(DB1_IP, 30004)

# --- MQTT Configuration ---
BROKER_URL   = "7301c4eb07c44d2ca436d360c487bcf8.s1.eu.hivemq.cloud"
TOPIC_JOINTS = "dobot/mg400/joints"

# --- DO & Timing ---
DO_SUCTION_ON  = 9
DO_SUCTION_OFF = 10
SUCTION_ENGAGE_DELAY  = 0.5
SUCTION_RELEASE_DELAY = 1.0
MQTT_INTERVAL         = 0.1

# --- Waypoints (Joint Space สำหรับจุดพักและจุดวางปลายทาง) ---
HOME_POINT      = [189.79, 182.00, 98.86, 51.42]
CONVEYOR_POINT  = [351.86, -53.95, 158.03, 88.25]
WASTE_POINT     = [2.60, -294.58, -20.21, 9.38]

# --- Offset Configuration (Base Cartesian) ---
BASE_X, BASE_Y, BASE_Z, BASE_R = 149.6, 239.96, -60.03, 87.49
X_LIMITS = [149.0, 255.0]
Y_LIMITS = [239.0, 375.0]

# =============================================================================
# Robot Functions
# =============================================================================

def start_up():
    print(f"🔄 [{LABEL}] กำลังเตรียมความพร้อม...")
    dashboard.ClearError()
    time.sleep(0.5)
    dashboard.EnableRobot()
    time.sleep(3)
    dashboard.SpeedFactor(50)
    print(f"✅ [{LABEL}] READY!")

def WaitArrive(target, is_joint=True):
    """รอจนกว่าจะถึงจุดหมาย พร้อม Debug Log"""
    mode = "Joint" if is_joint else "Point"
    print(f"  ⏳ [{LABEL}] Waiting to arrive at {mode}: {target}")
    
    start_wait = time.time()
    while True:
        is_arrive = True
        globalLockValue.acquire()
        actual = current_actual if is_joint else current_actual_point
        
        if actual is not None:
            limit = 1.0 if is_joint else 5
            for i in range(4):
                if abs(actual[i] - target[i]) > limit:
                    is_arrive = False
                    # Debug: พิมพ์ค่าปัจจุบันทุกๆ 2 วินาทีเพื่อดูความเคลื่อนไหว
                    if int(time.time() - start_wait) % 2 == 0 and time.time() - start_wait > 0.1:
                        print(f"     [DEBUG] Still moving... Current {mode}: {actual}")
                    break
            if is_arrive:
                globalLockValue.release()
                print(f"  ✨ [{LABEL}] Arrived at destination!")
                return
        else:
            print(f"  ⚠️ [{LABEL}] Warning: No feedback data yet...")
            
        globalLockValue.release()
        time.sleep(0.1)

def movj_wait(point, description="", is_joint=False):
    """สั่งเคลื่อนที่และรอจนกว่าจะถึงจุดหมายพร้อม Log"""
    print(f"  🦾 [{LABEL}] Command: {description}")
    p1, p2, p3, p4 = point
    move.MovJ(p1, p2, p3, p4)
    WaitArrive(point, is_joint=is_joint)

def suction_pick():
    print(f"  ⚡ [{LABEL}] Suction ON (Engaging...)")
    dashboard.DO(DO_SUCTION_ON, 1)
    time.sleep(SUCTION_ENGAGE_DELAY)
    dashboard.DO(DO_SUCTION_ON, 0)

def suction_release():
    print(f"  ⚡ [{LABEL}] Suction OFF (Releasing...)")
    dashboard.DO(DO_SUCTION_OFF, 1)
    time.sleep(SUCTION_RELEASE_DELAY)
    dashboard.DO(DO_SUCTION_OFF, 0)

def pick_and_place(to_conveyor, relative_coords=None):
    print(f"\n▶️ [{LABEL}] START: Pick and Place Task")
    coords = relative_coords if relative_coords else [0.0, 0.0, 0.0, 0.0]
    ox, oy, oz, orot = [BASE_X + coords[0], BASE_Y + coords[1], BASE_Z + coords[2], BASE_R + coords[3]]
    
    target_dest = CONVEYOR_POINT if to_conveyor else WASTE_POINT
    
    movj_wait(HOME_POINT, "Moving to HOME", is_joint=False)
    
    # Step 1: ไปเหนือวัตถุ
    approach_pos = [ox, oy, oz + 20, orot]
    movj_wait(approach_pos, "Approach Above Object", is_joint=False)
    
    # Step 2: ลงไปหยิบ
    print(f"  👇 [{LABEL}] Lowering to object...")
    move.MovL(ox, oy, oz + 10, orot)
    suction_pick()
    
    # Step 3: ยกขึ้น
    print(f"  👆 [{LABEL}] Lifting object...")
    move.MovL(ox, oy, oz + 20, orot)

    movj_wait(HOME_POINT, "Transit to HOME", is_joint=False)
    movj_wait(target_dest, "Moving to DROP Point", is_joint=False)
    
    suction_release()
    movj_wait(HOME_POINT, "Task Complete - Back to HOME", is_joint=False)

def edge_to_edge_sweep(relative_coords):
    print(f"\n▶️ [{LABEL}] START: Sweep Task")
    coords = relative_coords if relative_coords else [0.0, 0.0, 0.0, 0.0]
    ox, oy, oz, orot = [BASE_X + coords[0], BASE_Y + coords[1], BASE_Z + coords[2], BASE_R + coords[3]]
    
    target_x = max(min(ox, X_LIMITS[1]), X_LIMITS[0])
    y_start, y_end = (Y_LIMITS[0], Y_LIMITS[1]) if abs(oy - Y_LIMITS[1]) > abs(oy - Y_LIMITS[0]) else (Y_LIMITS[1], Y_LIMITS[0])

    print(f"  🧹 [{LABEL}] Sweep Path: Y {y_start} -> {y_end} at X {target_x}")
    
    movj_wait(HOME_POINT, "Move to HOME for Sweep Start", is_joint=False)
    movj_wait([target_x, y_start, oz + 25, orot], "Positioning at Sweep Edge", is_joint=False)
    
    print(f"  📉 [{LABEL}] Lowering brush...")
    move.MovL(target_x, y_start, oz + 15, orot)
    time.sleep(0.2)
    
    print(f"  ➡️ [{LABEL}] SWEEPING...")
    move.MovL(target_x, y_end, oz + 15, orot)
    time.sleep(0.2)
    
    move.MovL(target_x, y_end, oz + 25, orot)
    movj_wait(HOME_POINT, "Sweep Finished - Back to HOME", is_joint=False)

# =============================================================================
# MQTT Worker (หัวใจของการดึงข้อมูล Feedback)
# =============================================================================
def mqtt_worker():
    global current_actual, current_actual_point
    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set("Test1234", "Test1234")
    mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
    
    try:
        mqtt_client.connect(BROKER_URL, 8883, 60)
        mqtt_client.loop_start()
        print(f"📡 [{LABEL}] MQTT & Feedback Worker Started")
        
        while is_running:
            try:
                raw = feedback.socket_dobot.recv(1440)
                if len(raw) >= 1440:
                    data = np.frombuffer(raw, dtype=MyType)[0]
                    
                    globalLockValue.acquire()
                    current_actual = [round(float(j), 2) for j in data["q_actual"][:4]]
                    current_actual_point = [round(float(c), 2) for c in data["tool_vector_actual"][:4]]
                    globalLockValue.release()

                    payload = {
                        "timestamp": time.time(),
                        "robot": LABEL.lower(),
                        "joints": current_actual,
                        "coords": current_actual_point
                    }
                    mqtt_client.publish(TOPIC_JOINTS, json.dumps(payload))
            except Exception as e:
                print(f"     [FEEDBACK ERROR] {e}")
            time.sleep(MQTT_INTERVAL)
    except Exception as e:
        print(f"❌ MQTT Worker Fatal Error: {e}")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

# =============================================================================
# Main
# =============================================================================
if __name__ == "__main__":
    start_up()
    threading.Thread(target=mqtt_worker, daemon=True).start()

    try:
        while True:
            print("\n" + "="*45)
            print(" COMMANDS: 1: Conveyor | 2: Waste | 3: Sweep | q: Quit")
            print("="*45)
            cmd = input("Select Command → ").strip()
            
            if cmd in ["1", "2", "3"]:
                print(f"  [INPUT] Command {cmd} received.")
                raw = input("Enter x,y,z,r (Relative to Offset, or Enter for 0,0,0,0) → ").strip()
                try:
                    coords = [float(v) for v in raw.split(",")] if raw else [0.0, 0.0, 0.0, 0.0]
                    if len(coords) != 4: raise ValueError("Must be 4 values")
                    
                    if cmd == "1": pick_and_place(True, coords)
                    elif cmd == "2": pick_and_place(False, coords)
                    elif cmd == "3": edge_to_edge_sweep(coords)
                except ValueError as ve:
                    print(f"  ⚠️ Invalid input format: {ve}")
            elif cmd == "q": break
    finally:
        is_running = False
        print("🔌 System shutting down...")
        time.sleep(1)