import time
import json
import ssl
import paho.mqtt.client as mqtt
# นำเข้า Class จากไฟล์ dobot_api.py ของคุณ
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove

# --- Configuration ---
BROKER_URL = "7301c4eb07c44d2ca436d360c487bcf8.s1.eu.hivemq.cloud"
TOPIC_JOINTS = "dobot/mg400/joints"

class RobotController:
    def __init__(self, ip, label):
        self.ip = ip
        self.label = label
        # เชื่อมต่อ 3 Ports หลักของ MG400
        self.dashboard = DobotApiDashboard(ip, 29999)
        self.move = DobotApiMove(ip, 30003)
        self.feedback = DobotApi(ip, 30004) # เพิ่มการดึงข้อมูล Real-time
        
        # ตำแหน่งพิกัดสำหรับโปรเจกต์[cite: 1]
        self.pos_pick = [250, 0, 20, 0]
        self.pos_conveyor = [300, 150, 20, 0]
        self.pos_waste = [200, -150, 20, 0]

    def start_up(self):
        """เริ่มต้นการทำงานของแขนกล[cite: 1]"""
        print(f"กำลังเริ่มระบบ {self.label}...")
        self.dashboard.ClearError()
        time.sleep(0.5)
        self.dashboard.EnableRobot()
        time.sleep(2) 
        self.dashboard.SpeedFactor(50)[cite: 1]
        print(f"✅ {self.label} พร้อมทำงาน")

    def get_joints(self):
        """ดึงค่า Joint Angle ผ่าน DobotApiFeedback[cite: 1]"""
        try:
            # ใช้ feed_data() จาก DobotApiFeedback ซึ่งเร็วกว่า GetAngle()[cite: 1]
            data = self.feedback.feed_data()
            if data:
                # โดยปกติ feedback จะส่งค่ามาเป็นอาเรย์ขนาดใหญ่ 
                # ตำแหน่งของ Joint J1-J4 มักจะอยู่ที่ index ต้นๆ (ตรวจสอบตามเวอร์ชัน API)[cite: 1]
                return [round(data[i], 2) for i in range(4)] 
            return [0.0, 0.0, 0.0, 0.0]
        except Exception as e:
            # print(f"Error reading feedback: {e}")
            return [0.0, 0.0, 0.0, 0.0]

    def move_to(self, x, y, z, r):
        """ส่งคำสั่งเคลื่อนที่แบบ String เพื่อความชัวร์[cite: 1]"""
        cmd = f"MovL({x},{y},{z},{r})"
        self.move.send_data(cmd)[cite: 1]

    def pick_and_place(self, target_pos):
        """ฟังก์ชันรวมการหยิบและวางที่ประหยัดโค้ด[cite: 1]"""
        safe_z = 100
        
        # 1. ขั้นตอนการหยิบ[cite: 1]
        self.move_to(self.pos_pick[0], self.pos_pick[1], safe_z, self.pos_pick[3])
        time.sleep(0.5)
        self.move_to(*self.pos_pick)
        self.dashboard.ToolDO(9, 1)  # เปิด Suction[cite: 1]
        self.dashboard.ToolDO(9, 0)
        time.sleep(0.5)
        self.move_to(self.pos_pick[0], self.pos_pick[1], safe_z, self.pos_pick[3])
        
        # 2. ขั้นตอนการวาง[cite: 1]
        self.move_to(target_pos[0], target_pos[1], safe_z, target_pos[3])
        time.sleep(0.5)
        self.move_to(*target_pos)
        self.dashboard.ToolDO(10, 1) # ปิด Suction[cite: 1]
        self.dashboard.ToolDO(10, 0)
        time.sleep(0.5)
        self.move_to(target_pos[0], target_pos[1], safe_z, target_pos[3])

# --- การตั้งค่า MQTT ---
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set("Test1234", "Test1234")
mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED)

# --- สร้าง Instance สำหรับหุ่นยนต์ ---[cite: 1]
db1 = RobotController("192.168.1.6", "Robot_1")
db2 = RobotController("192.168.1.7", "Robot_2") # เผื่อไว้สำหรับตัวที่ 2[cite: 1]

if __name__ == "__main__":
    mqtt_client.connect(BROKER_URL, 8883, 60)
    mqtt_client.loop_start()
    
    db1.start_up()

    try:
        while True:
            # ดึงค่า Joint จากทั้ง 2 ตัวส่งไป MQTT[cite: 1]
            j1 = db1.get_joints()
            j2 = db2.get_joints() # หากยังไม่ต่อ db2 ค่าจะเป็น 0.0[cite: 1]
            
            payload = {
                "robot1": {f"j{i+1}": j1[i] for i in range(4)},
                "robot2": {f"j{i+1}": j2[i] for i in range(4)}
            }
            mqtt_client.publish(TOPIC_JOINTS, json.dumps(payload))

            # รับคำสั่งควบคุมผ่านหน้าจอ[cite: 1]
            cmd = input(f"\n[{db1.label}] 1: Conveyor, 2: Waste, q: Quit -> ")
            if cmd == '1':
                db1.pick_and_place(db1.pos_conveyor)
            elif cmd == '2':
                db1.pick_and_place(db1.pos_waste)
            elif cmd == 'q':
                break
            time.sleep(0.1)
            
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()