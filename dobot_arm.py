from dobot_api import DobotApiDashboard, DobotApiMove
import time

# --- ตั้งค่า IP ของแขนกลตัวที่ 1 ---
IP_ROBOT_1 = "192.168.1.6" 

class Robot1Tester:
    def __init__(self, ip):
        # พอร์ต 29999 มักใช้สำหรับ Feedback สถานะ แต่พอร์ตควบคุมหลักคือ 29000
        # แนะนำให้ลองเปลี่ยนเป็น 29000 หากคำสั่งส่งไม่ไป 
        self.dashboard = DobotApiDashboard(ip, 29999) 
        self.move = DobotApiMove(ip, 30003)
        
    def setup(self):
        print("--- กำลังเตรียมความพร้อม Robot 1 ---")
        
        # 1. ล้าง Error และสถานะค้างเก่า
        self.dashboard.ClearError() 
        time.sleep(0.5)
        
        # 2. ปล่อยเบรกและเปิดใช้งานมอเตอร์ (Enable)
        self.dashboard.EnableRobot()
        print("Waiting for Robot to Enable...")
        
        # รอจนกว่าสถานะจะพร้อม (ตรวจสอบ Error หรือรอเวลา)
        time.sleep(2) 
        
        # 3. ตั้งค่า User และ Tool Coordinate เป็น 0 (สำคัญมากเพื่อให้แขนกลไม่งงพิกัด)
        self.dashboard.User(0)
        self.dashboard.Tool(0)
        
        # 4. ตั้งความเร็วและอัตราเร่ง (Speed & Accel)
        self.dashboard.SpeedFactor(50) # ตั้งความเร็วที่ 50% เพื่อความปลอดภัย [cite: 5]
        
        print("Robot 1 is READY!")

    def process_item(self, x, y, z, r, item_type):
        """
        กระบวนการหยิบชิ้นงาน: [cite: 20]
        """
        # 1. เคลื่อนที่ไปเหนือชิ้นงาน (Safety Height)
        self.move.MovL(x, y, z + 50, r) 
        
        # 2. ลงไปหยิบชิ้นงาน
        self.move.MovL(x, y, z, r)
        
        # --- ขั้นตอนการหยิบ (Sucking/Gripping) ---
        print("Action: Sucking Item...")
        self.dashboard.DO(9, 1) # สั่งเปิดลมดูดที่ DO9
        time.sleep(1.5)             # เพิ่มเวลาให้ลมดูดทำงานเต็มที่ (ห้ามสั่ง DO OFF ทันที)
        self.dashboard.DO(9, 0) 
        time.sleep(1.5) 
        # 3. ยกขึ้น (สถานะ DO9 ต้องยังคงเป็น 1)
        self.move.MovL(x, y, z + 50, r)

        if item_type == 'green':
            print("Status: Good Item detected [cite: 30]")
            # นำไปวางที่จุดนำเข้าสายพาน 1 [cite: 15, 34]
            self.move.MovL(200, 100, 50, 0) 
            
            # เมื่อถึงเป้าหมายแล้วค่อยปล่อย (Release)
            self.dashboard.ToolDO(10, 1) 
            time.sleep(1.5)
            self.dashboard.ToolDO(10, 0) 
            print("Action: Released on Conveyor 1")
            time.sleep(1.5)  
            
        else:
            print("Status: Faulty Item detected [cite: 33]")
            # นำไปวางที่จุดวางชิ้นงานเสีย [cite: 17, 35]
            self.move.MovL(100, -200, 50, 0) 
            self.dashboard.ToolDO(10, 1) 
            time.sleep(1.5)
            self.dashboard.ToolDO(10, 0) 
            print("Action: Discarded Faulty Item")
            time.sleep(1.5)  

        time.sleep(0.5)
        print("เสร็จสิ้นการทำงาน 1 รอบ")

# --- Main Test Execution ---
if __name__ == "__main__":
    test_robot = Robot1Tester(IP_ROBOT_1)
    test_robot.setup()

    # พิกัดเป้าหมาย (x, y, z, r) จากระบบประมวลผลภาพ [cite: 32, 64]
    detected_x, detected_y, detected_z, detected_r = 217, 294.70, -36.48, 121.01
    status = 'green'

    try:
        test_robot.process_item(detected_x, detected_y, detected_z, detected_r, status)
    except KeyboardInterrupt:
        print("Emergency Stop triggered by User.")