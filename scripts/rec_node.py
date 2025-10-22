#!/usr/bin/env python3
import speech_recognition as sr
import time
import tkinter as tk
from tkinter import ttk 
import rospy # <<< 1. Import rospy
from std_msgs.msg import String # <<< 2. Import String message

# --- ฟังก์ชันหลัก (เพิ่ม publisher) ---
def recognize_speech_from_mic(duration, label, publisher): # <<< 3. เพิ่ม publisher
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        label.config(text="...กำลังปรับระดับเสียง...", foreground="#666666") 
        try:
            recognizer.adjust_for_ambient_noise(source, duration=1)
            label.config(text="กรุณาพูดได้เลยครับ...", foreground="#000000") 

            audio = recognizer.listen(source, timeout=duration)

            label.config(text="...กำลังประมวลผล...", foreground="#666666")
            text = recognizer.recognize_google(audio, language="th-TH")
            label.config(text=f"คุณพูดว่า: {text}", foreground="#006400") 

            # <<< 5. Publish ข้อความที่แปลงได้
            rospy.loginfo(f"Publishing: {text}")
            publisher.publish(text) # ส่งข้อความ (ภาษาไทย) ออกไป

        except sr.UnknownValueError:
            label.config(text="[Error] ไม่สามารถเข้าใจเสียงที่พูดได้", foreground="#D2042D") 
        except sr.RequestError:
            label.config(text="[Error] เกิดข้อผิดพลาดในการเชื่อมต่อ", foreground="#D2042D")

def start_recording():
    duration = int(slider.get()) 
    result_label.config(foreground="#333333")
    # ส่ง pub เข้าไปในฟังก์ชัน
    recognize_speech_from_mic(duration, result_label, pub) # <<< 3. 

# --- 4. เริ่มต้น ROS Node และ Publisher ---
rospy.init_node('talker', anonymous=True)
pub = rospy.Publisher('/box', String, queue_size=10) # จะส่งไปที่ Topic ชื่อ /box


# --- สร้างหน้าต่าง GUI (เหมือนเดิม) ---
root = tk.Tk()
root.title("Speech Recognition Node (Talker)")
root.geometry("400x350")
root.configure(bg="#2E2E2E") 

style = ttk.Style(root)
style.theme_use('clam')
style.configure("Dark.TLabel", background="#2E2E2E", foreground="#FFFFFF")
style.configure("TButton", padding=10)
style.configure("Horizontal.TScale", background="#2E2E2E")
style.configure("Result.TFrame", background="#F0F0F0") 
style.configure("Result.TLabel", background="#F0F0F0", foreground="#333333") 

header_label = ttk.Label(root, text="ตั้งเวลาอัดเสียง (วินาที)", style="Dark.TLabel")
header_label.pack(pady=(20, 10)) 

slider = ttk.Scale(root, from_=1, to=10, orient="horizontal", length=300)
slider.pack(pady=10)

record_button = ttk.Button(root, text="RECORD", command=start_recording)
record_button.pack(pady=20)

result_frame = ttk.Frame(root, style="Result.TFrame", padding=(10, 10))
result_frame.pack(padx=20, fill="x") 

result_label = ttk.Label(result_frame, text="กด RECORD เพื่อเริ่ม", style="Result.TLabel", wraplength=340) 
result_label.pack()

root.mainloop()
