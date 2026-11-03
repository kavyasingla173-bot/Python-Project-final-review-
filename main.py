from __future__ import annotations
import tkinter as tk
from tkinter import ttk
import serial
import time
import math
import random
from typing import Sequence

# ==========================================
# SERIAL SETUP (Original + Block 1)
# ==========================================
SERIAL_PORT = "COM9"
BAUD_RATE = 115200
ser = None

def connect_serial():
    global ser
    if ser and ser.is_open:
        return True
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05, write_timeout=0.05)
        time.sleep(0.5)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        return True
    except Exception as e:
        print(f"Serial port {SERIAL_PORT} not connected:", e)
        ser = None
        return False

connect_serial()


# ==========================================
# BLOCK 2: Basic Sine + Noise Temp Generator
# ==========================================
start_time = time.time()
BASE_TEMPS = [34.5, 42.0, 31.0, 51.5]

def new_temp():
    t = time.time() - start_time
    temperatures = []
    
    for i in range(4):
        cyclic_trend = 6.5 * math.sin((t / 15.0) + (i * 1.2))
        noise = random.uniform(-0.8, 0.8)
        temp = BASE_TEMPS[i] + cyclic_trend + noise
        temperatures.append(round(temp, 2))
        
    return temperatures


# ==========================================
# BLOCK 3: Damped Thermal Cluster Generator
# ==========================================
NODE_COUNT = 4

def initial_temperatures(ambient_c: float = 28.0) -> tuple[float, ...]:
    return tuple(ambient_c + 8.0 + 5.0 * node for node in range(NODE_COUNT))

def generate_target_temperature(
    node: int,
    time_s: float,
    sample_index: int,
    *,
    seed: int = 42,
    ambient_c: float = 28.0,
) -> float:
    if not 0 <= node < NODE_COUNT:
        raise ValueError(f"node must be between 0 and {NODE_COUNT - 1}")

    phase = node * math.pi / 3.0
    baseline_c = ambient_c + 8.0 + 6.5 * node
    slow_cycle = 9.0 * math.sin((2.0 * math.pi * time_s / 45.0) + phase)
    fast_cycle = 3.5 * math.sin((2.0 * math.pi * time_s / 12.0) + phase / 2.0)

    node_seed = seed + 10_007 * node + 1_000_003 * sample_index
    noise_c = random.Random(node_seed).uniform(-1.5, 1.5)
    return baseline_c + slow_cycle + fast_cycle + noise_c

def damp_temperature(
    current_c: float,
    target_c: float,
    elapsed_s: float,
    response_time_s: float = 6.0,
) -> float:
    if elapsed_s < 0:
        raise ValueError("elapsed_s cannot be negative")
    if response_time_s <= 0:
        raise ValueError("response_time_s must be greater than zero")
    alpha = 1.0 - math.exp(-elapsed_s / response_time_s)
    return current_c + alpha * (target_c - current_c)

def update_cluster_temperatures(
    current_temperatures_c: Sequence[float],
    time_s: float,
    elapsed_s: float,
    sample_index: int,
    *,
    seed: int = 42,
    ambient_c: float = 28.0,
    response_time_s: float = 6.0,
) -> tuple[float, ...]:
    if len(current_temperatures_c) != NODE_COUNT:
        raise ValueError(f"expected {NODE_COUNT} node temperatures")

    updated = []
    for node, current_c in enumerate(current_temperatures_c):
        target_c = generate_target_temperature(
            node,
            time_s,
            sample_index,
            seed=seed,
            ambient_c=ambient_c,
        )
        updated.append(
            damp_temperature(current_c, target_c, elapsed_s, response_time_s)
        )
    return tuple(updated)


# ==========================================
# BLOCK 4: Adaptive Pump Speed & PWM Control
# ==========================================
def temp_to_target_speed(temp, idle_temp=30.0, max_temp=80.0, min_pwm=40, max_pwm=255):
    if temp <= idle_temp:
        return float(min_pwm)
    if temp >= max_temp:
        return float(max_pwm)
    ratio = (temp - idle_temp) / (max_temp - idle_temp)
    return min_pwm + ratio * (max_pwm - min_pwm)

def damped_adaptive_speeds(
    temps,
    prev_speeds,
    damping=0.25,
    deadband=2.0,
    idle_temp=30.0,
    max_temp=80.0,
    min_pwm=40,
    max_pwm=255,
):
    new_speeds = []

    for i in range(4):
        target = temp_to_target_speed(
            temps[i], idle_temp, max_temp, min_pwm, max_pwm
        )
        prev = prev_speeds[i]

        if abs(target - prev) < deadband:
            new_speeds.append(prev)
            continue

        speed = prev + damping * (target - prev)
        speed = max(min_pwm, min(max_pwm, speed))
        new_speeds.append(speed)

    return new_speeds


# ==========================================
# INTEGRATION & SERIAL TX/RX (Main Loop)
# ==========================================
current_temps = initial_temperatures()
motor_speeds = [40.0, 40.0, 40.0, 40.0]
sample_idx = 0
last_time = time.time()
emergency_halted = False
last_arduino_ack = "None"

def send_and_receive_arduino(speeds):
    # Writes 4 PWM speeds to COM9 and reads back Arduino ACK from COM8
    global ser, last_arduino_ack
    if ser and ser.is_open:
        try:
            pwm_ints = [str(int(round(s))) for s in speeds]
            payload = ",".join(pwm_ints) + "\n"
            ser.write(payload.encode())

            if ser.in_waiting > 0:
                response = ser.readline().decode(errors="ignore").strip()
                if response:
                    last_arduino_ack = response
            return True
        except (serial.SerialException, serial.SerialTimeoutException):
            ser = None
            return False
    return False

def emergency_stop():
    global emergency_halted, motor_speeds
    emergency_halted = not emergency_halted
    if emergency_halted:
        motor_speeds = [0.0, 0.0, 0.0, 0.0]
        send_and_receive_arduino(motor_speeds)
        btn_stop.config(text="RESUME SYSTEM", bg="#008000")
        lbl_status.config(text="EMERGENCY STOP ACTIVE", fg="red")
    else:
        motor_speeds = [40.0, 40.0, 40.0, 40.0]
        btn_stop.config(text="EMERGENCY STOP", bg="#ff0800")


# ==========================================
# BLOCK 1: GUI Dashboard & Update Loop
# ==========================================
def update_dashboard(data, serial_ok):
    if len(data) == 8:
        try:
            temp1, temp2, temp3, temp4 = [float(x) for x in data[0:4]]
            speed1, speed2, speed3, speed4 = [float(x) for x in data[4:8]]

            temp_labels[0].config(text=f"{temp1:.1f} °C")
            temp_labels[1].config(text=f"{temp2:.1f} °C")
            temp_labels[2].config(text=f"{temp3:.1f} °C")
            temp_labels[3].config(text=f"{temp4:.1f} °C")

            speed_labels[0].config(text=f"{speed1:.0f} %")
            speed_labels[1].config(text=f"{speed2:.0f} %")
            speed_labels[2].config(text=f"{speed3:.0f} %")
            speed_labels[3].config(text=f"{speed4:.0f} %")

            if serial_ok:
                lbl_status.config(
                    text=f"COM9 CONNECTED | ARDUINO {last_arduino_ack}",
                    fg="green"
                )
            else:
                lbl_status.config(
                    text="COM9 DISCONNECTED - RUNNING LOCAL SIMULATION",
                    fg="orange"
                )

        except ValueError:
            lbl_status.config(text="INVALID DATA RECEIVED", fg="red")

def read_data():
    global current_temps, motor_speeds, sample_idx, last_time

    now = time.time()
    elapsed = max(0.01, now - last_time)
    last_time = now
    sample_idx += 1

    if not emergency_halted:
        # Step 1: Blend Block 3 and Block 2 temperatures
        b3_temps = update_cluster_temperatures(
            current_temps,
            time_s=now - start_time,
            elapsed_s=elapsed,
            sample_index=sample_idx,
        )
        b2_temps = new_temp()
        current_temps = tuple((b3_temps[i] + b2_temps[i]) / 2.0 for i in range(4))

        # Step 2: Calculate damped PWM speeds (40-255)
        motor_speeds = damped_adaptive_speeds(current_temps, motor_speeds)

        # Step 3: Send PWM values to Arduino over COM9 and check ACK
        serial_ok = send_and_receive_arduino(motor_speeds)

        # Step 4: Convert PWM (0-255) to percentage (0-100%) for UI display
        speed_pcts = [round((s / 255.0) * 100.0, 1) for s in motor_speeds]
        combined_data = list(current_temps) + speed_pcts
        update_dashboard(combined_data, serial_ok)
    else:
        for i in range(4):
            speed_labels[i].config(text="0 %")

    root.after(500, read_data)


# Main Dashboard Window
root = tk.Tk()
root.title("Four Node Server Cooling & Motor System")
root.geometry("600x600")
root.resizable(False, False)

title = tk.Label(
    root,
    text="FOUR NODE TEMPERATURE AND MOTOR MONITORING",
    font=("Segoe UI", 14, "bold")
)
title.pack(pady=12)

info = tk.Label(
    root,
    text="Damped Thermal Workload & Adaptive Pump Speeds (COM9 <-> COM8)",
    font=("Segoe UI", 10)
)
info.pack(pady=4)

node_frame = tk.Frame(root)
node_frame.pack(pady=10)

temp_labels = []
speed_labels = []

for i in range(4):
    frame = tk.LabelFrame(
        node_frame,
        text="NODE " + str(i + 1),
        font=("Segoe UI", 11, "bold"),
        padx=20,
        pady=12
    )
    frame.grid(row=i // 2, column=i % 2, padx=12, pady=10)

    tk.Label(frame, text="Temperature", font=("Segoe UI", 10)).pack()
    temp = tk.Label(frame, text="Waiting...", font=("Segoe UI", 18, "bold"), fg="teal")
    temp.pack(pady=4)
    temp_labels.append(temp)

    tk.Label(frame, text="Pump Speed", font=("Segoe UI", 10)).pack()
    speed = tk.Label(frame, text="Waiting...", font=("Segoe UI", 18, "bold"), fg="#0066cc")
    speed.pack(pady=4)
    speed_labels.append(speed)

ctrl_frame = tk.Frame(root)
ctrl_frame.pack(pady=8)

btn_stop = tk.Button(
    ctrl_frame,
    text="EMERGENCY STOP",
    command=emergency_stop,
    bg="#ff0800",
    fg="#ffffff",
    font=("Segoe UI", 10, "bold"),
    padx=12,
    pady=6
)
btn_stop.grid(row=0, column=0, padx=8)

btn_reconnect = tk.Button(
    ctrl_frame,
    text="RECONNECT COM9",
    command=connect_serial,
    bg="#0066cc",
    fg="#ffffff",
    font=("Segoe UI", 10, "bold"),
    padx=12,
    pady=6
)
btn_reconnect.grid(row=0, column=1, padx=8)

lbl_status = tk.Label(
    root,
    text="WAITING FOR DATA",
    font=("Segoe UI", 11, "bold"),
    fg="orange"
)
lbl_status.pack(pady=8)

root.after(500, read_data)
root.mainloop()

if ser and ser.is_open:
    ser.close()
