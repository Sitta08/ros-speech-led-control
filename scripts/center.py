#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk # <<< เราจะใช้ ttk เพื่อความสวยงาม

class ColorBoxGUI:
    """
    GUI ที่จะแสดงกล่องสี (เวอร์ชันสวย)
    """
    def __init__(self, root):
        self.root = root
        self.root.title("Color Box (Listener)")
        self.root.geometry("250x250")

        # 1. ตั้งค่าพื้นหลังหน้าต่างเป็นสีเข้ม
        self.root.configure(bg="#2E2E2E") 

        # --- 2. สร้าง Style ต่างๆ ---
        self.style = ttk.Style(root)
        self.style.theme_use('clam')

        # สร้าง Style ให้ "กล่อง" (Frame)
        self.style.configure("Red.TFrame", background="#F08080") # สีแดงอ่อน
        self.style.configure("Green.TFrame", background="#90EE90") # สีเขียวอ่อน
        self.style.configure("Yellow.TFrame", background="#F0E68C") # สีเหลือง

        # สร้าง Style ให้ "ตัวอักษร" (Label)
        # เราจะไม่ระบุ 'family' แต่จะระบุ 'size' และ 'weight' (ปลอดภัย)
        self.style.configure("Red.TLabel", background="#F08080", foreground="#333333", padding=10, font=("", 18, "bold"))
        self.style.configure("Green.TLabel", background="#90EE90", foreground="#333333", padding=10, font=("", 18, "bold"))
        self.style.configure("Yellow.TLabel", background="#F0E68C", foreground="#333333", padding=10, font=("", 18, "bold"))

        # --- 3. สร้าง Widgets ---
        # สร้าง Frame (กล่อง) เริ่มต้นด้วย Style สีเหลือง
        self.box_frame = ttk.Frame(self.root, style="Yellow.TFrame")
        # ยืดกล่องให้เต็มหน้าต่าง และเว้นขอบ 20px (ให้ดูเหมือนลอย)
        self.box_frame.pack(fill="both", expand=True, padx=20, pady=20) 

        # สร้าง Label (ข้อความ) เริ่มต้นด้วย Style สีเหลือง
        self.text_label = ttk.Label(self.box_frame, text="Waiting...", style="Yellow.TLabel", anchor="center")
        # ยืดข้อความให้เต็มกล่อง
        self.text_label.pack(fill="both", expand=True)

    def update_color(self, color_name):
        # ฟังก์ชันเปลี่ยนสี (แบบใหม่)
        # เราจะเปลี่ยน Style ของ Frame และ Label
        new_style_name = color_name.capitalize() # "red" -> "Red"

        self.box_frame.configure(style=f"{new_style_name}.TFrame")
        self.text_label.configure(text=color_name.upper(), style=f"{new_style_name}.TLabel")

class CenterNode:
    """
    Node ที่ทำหน้าที่ 2 อย่าง (เหมือนเดิม)
    """
    def __init__(self):
        rospy.init_node("center", anonymous=True)
        self.gui = ColorBoxGUI(tk.Tk())

        self.pub = rospy.Publisher("/data", String, queue_size=10)
        rospy.Subscriber("/box", String, self.callback)

    def callback(self, msg):
        color_th = msg.data.lower() # รับคำภาษาไทย
        color_en = "yellow" # สีเริ่มต้น

        # --- "แปล" คำสั่งภาษาไทย ---
        if "เขียว" in color_th:
            color_en = "green"
        elif "แดง" in color_th:
            color_en = "red"
        elif "เหลือง" in color_th:
            color_en = "yellow"
        else:
            color_en = "yellow"

        # อัปเดต GUI (แบบใหม่)
        self.gui.update_color(color_en) # [cite: 919-924, 1577-1581]

        # ส่งต่อเป็นภาษาอังกฤษ
        self.pub.publish(color_en)

    def run(self):
        self.gui.root.mainloop()

if __name__ == "__main__":
    node = CenterNode()
    node.run()
