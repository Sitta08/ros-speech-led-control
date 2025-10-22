#!/usr/bin/env python3
import rospy
import serial # <<< 1. Import ไลบรารีสำหรับคุยกับ USB
from std_msgs.msg import String

# สร้างตัวแปร ser (Serial port) ไว้ข้างนอก
ser = None

def callback(msg):
    """
    ฟังก์ชันนี้จะทำงานทุกครั้งที่ได้ยินข้อมูลจาก topic '/data'
    """
    # 3. รับข้อมูล (เช่น "red", "green")
    data = msg.data.lower()

    # 4. "แปล" ข้อความเป็นตัวเลขสำหรับ Arduino
    if data == "red":
        value = "1\n" # \n คือการขึ้นบรรทัดใหม่ (ที่ Arduino รอฟัง)
    elif data == "yellow":
        value = "2\n"
    elif data == "green":
        value = "3\n"
    else:
        value = "0\n" # ถ้าพูดอย่างอื่น ให้ส่ง 0 (ปิดไฟ)

    # 5. ส่งข้อมูลที่แปลแล้ว (เช่น "1\n") ไปยัง Arduino
    try:
        ser.write(value.encode()) # .encode() คือการแปลง String เป็น byte
        rospy.loginfo(f"Received '{data}', Sent to Arduino: {value.strip()}")
    except Exception as e:
        rospy.logwarn(f"Error writing to serial port: {e}")

if __name__ == '__main__':
    try:
        # 2. ตั้งค่าการเชื่อมต่อ Serial
        # ---!!!สำคัญมาก!!! ---
        # คุณต้องเช็คว่าบอร์ดของคุณคือ /dev/ttyACM0 หรือ /dev/ttyUSB0
        serial_port = '/dev/ttyACM0' 
        # -------------------------

        baud_rate = 9600 # ต้องตรงกับในโค้ด Arduino

        # เปิดการเชื่อมต่อ
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        rospy.loginfo(f"Serial connected to {serial_port}")

        # 1. เริ่มต้น Node
        rospy.init_node('arduino_node', anonymous=True)

        # 2. สั่งให้ Node นี้ คอยฟัง (Subscribe) topic '/data'
        rospy.Subscriber('/data', String, callback)

        rospy.loginfo("Real Arduino Node is running...")

        # สั่งให้ Node นี้ทำงานไปเรื่อยๆ
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except serial.serialutil.SerialException as e:
        rospy.logerr(f"Serial port error: {e}")
        rospy.logerr(f"*** ไม่พบ Arduino ที่ {serial_port} ***")
        rospy.logerr(f"*** ตรวจสอบว่าเสียบสาย USB และลองรัน 'sudo chmod a+rw {serial_port}' ***")
    finally:
        # ปิดพอร์ตเมื่อจบการทำงาน
        if ser and ser.is_open:
            ser.close()
            rospy.loginfo("Serial port closed.")
