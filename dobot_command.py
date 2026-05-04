import time
import json
import ssl
import struct
import threading
import paho.mqtt.client as mqtt
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi

# --- Configuration ---
DB1_IP = "192.168.1.6" 
BROKER_URL = "7301c4eb07c44d2ca436d360c487bcf8.s1.eu.hivemq.cloud"
TOPIC_JOINTS = "dobot/mg400/joints"

class RobotController:
    def __init__(self, ip, label):
        self.ip = ip
        self.label = label
        self.dashboard = DobotApiDashboard(ip, 29999)
        self.move = DobotApiMove(ip, 30003)
        self.feedback = DobotApi(ip, 30004)
        
        self.pos_conveyor = [200, 100, 50, 0]
        self.pos_waste = [100, -200, 50, 0]
        self.is_running = True

    def start_up(self):
        print(f"🔄 กำลังเตรียมความพร้อม {self.label}...")
        self.dashboard.ClearError() 
        time.sleep(0.5)
        self.dashboard.EnableRobot()
        time.sleep(3) 
        self.dashboard.SpeedFactor(50)
        print(f"✅ {self.label} READY!")

    def get_joints_manual(self):
        """แกะรหัส Byte Stream จากพอร์ต 30004 เพื่อหาค่า Joint"""
        try:
            # ดึงข้อมูลดิบจาก Socket
            data = self.feedback.feed_data()
            
            # MG400 Feedback Package มีขนาด 1440 bytes
            if len(data) >= 1440:
                # โครงสร้างข้อมูล MG400:
                # Offset 0-23: Header & Time
                # Offset 28-75: Joint Positions (6 Joints * 8 bytes each, float64/double)
                # ใช้ format string '6d' หมายถึง double 6 ตัวเรียงกัน
                # '<' คือ Little-endian (มาตรฐานของ Dobot)
                joint_data = struct.unpack('<6d', data[28:76])
                
                # คืนค่าเฉพาะ J1, J2, J3, J4 และปัดเศษ 2 ตำแหน่ง
                return [round(j, 2) for j in joint_data[:4]]
            else:
                return None
        except Exception as e:
            # print(f"Unpack Error: {e}") # เปิดไว้ตอน debug
            return None

    def pick_and_place(self, x, y, z, r, is_good):
        safe_z = z + 50
        target = self.pos_conveyor if is_good else self.pos_waste
        
        print(f"📦 Action: {'Good' if is_good else 'Faulty'} item")
        self.move.MovL(x, y, safe_z, r) 
        self.move.MovL(x, y, z, r)
        self.dashboard.DO(9, 1) 
        time.sleep(1.5) 
        self.dashboard.DO(9, 0) 
        time.sleep(1.5) 
        self.move.MovL(x, y, safe_z, r)
        self.move.MovL(target[0], target[1], target[2] + 50, target[3])
        self.move.MovL(*target)
        self.dashboard.DO(10, 1) 
        time.sleep(1.0)
        self.dashboard.DO(10, 0) 
        time.sleep(1.0)
        self.move.MovL(target[0], target[1], target[2] + 50, target[3])

# --- MQTT Setup ---1
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set("Test1234", "Test1234")
mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED)



def mqtt_worker():
    """ฟังก์ชันทำงานเบื้องหลังเพื่อส่งค่า Joint ไปยัง Digital Twin"""
    print("📡 MQTT Worker Started")
    while db1.is_running:
        joints = db1.get_joints_manual()
        if joints:
            payload = {
                "timestamp": time.time(),
                "robot1": {f"j{i+1}": joints[i] for i in range(4)}
            }
            mqtt_client.publish(TOPIC_JOINTS, json.dumps(payload))
        time.sleep(0.1) # ความถี่ 10Hz

if __name__ == "__main__":
    db1 = RobotController(DB1_IP, "DB1")
    try:
        mqtt_client.connect(BROKER_URL, 8883, 60)
        mqtt_client.loop_start()
        
        db1.start_up()

        # สร้าง Thread แยกสำหรับส่งข้อมูล เพื่อไม่ให้โปรแกรมหยุดรอ input
        thread = threading.Thread(target=mqtt_worker, daemon=True)
        thread.start()

        print("\n--- ควบคุมหุ่นยนต์ ---")
        det_x, det_y, det_z, det_r = 217, 294.70, -36.48, 121.01

        while True:
            db1.dashboard.DO(9, 1) # สั่งเปิดลมดูดที่ DO9
            time.sleep(1.5) 
            db1.dashboard.DO(9, 0)
            time.sleep(1.5) 
            break

            # cmd = input(f"[{db1.label}] '1': Good, '2': Waste, 'q': Exit: ")
            # if cmd == '1':
            #     db1.pick_and_place(det_x, det_y, det_z, det_r, True)
            # elif cmd == '2':
            #     db1.pick_and_place(det_x, det_y, det_z, det_r, False)
            # elif cmd == 'q':
            #     db1.is_running = False
            #     break
    finally:
        db1.is_running = False
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("🔌 ปิดการทำงานเรียบร้อย")