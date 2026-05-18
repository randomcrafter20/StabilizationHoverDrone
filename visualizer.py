import math
import serial
from serial.tools import list_ports
from vpython import box, vector, rate, scene, color, arrow, label

# -----------------------------
# Serial settings
# -----------------------------
PORT = "COM3"         # Set to a specific COM port string (e.g., "COM6") or keep None for auto-detect
BAUD = 230400
RX_BUFFER = bytearray()

# -----------------------------
# Scene setup
# -----------------------------
scene.title = "Quadcopter Attitude Viewer"
scene.width = 1000
scene.height = 700
scene.background = color.black
scene.center = vector(0, 0, 0)


def to_display(v: vector) -> vector:
    # Logical frame -> VPython display frame:
    # +X (away) -> -Z, +Y (left) -> -X, +Z (up) -> +Y
    return vector(-v.y, v.z, -v.x)

ARM_TIP = 1.5   # half-arm length in body-frame units

# World axes
arrow(pos=vector(0, 0, 0), axis=to_display(vector(3, 0, 0)), color=color.red)    # +X (away)
arrow(pos=vector(0, 0, 0), axis=to_display(vector(0, 3, 0)), color=color.green)  # +Y (left)
arrow(pos=vector(0, 0, 0), axis=to_display(vector(0, 0, 3)), color=color.blue)   # +Z (up)

# Quadcopter body
body = box(pos=vector(0, 0, 0), size=vector(2.5, 0.15, 0.8), color=color.cyan)

# Arms (simple cross shape)
arm1 = box(pos=vector(0, 0, 0), size=vector(3.0, 0.05, 0.12), color=color.white)
arm2 = box(pos=vector(0, 0, 0), size=vector(0.12, 0.05, 3.0), color=color.white)

# Nose marker
nose = arrow(pos=vector(0, 0, 0), axis=vector(1.3, 0, 0), color=color.yellow, shaftwidth=0.08)

info = label(
    pos=vector(0, 2.2, 0),
    text="Roll: 0.0  Pitch: 0.0  Yaw: 0.0  Alt: 0.0",
    height=16,
    box=False,
    color=color.white
)

# Motor speed labels — one per corner, positions updated each frame by rotate_model()
_tip = ARM_TIP
_off = to_display(vector(0, 0, 0.35))   # small up-offset at rest
lbl_m1 = label(pos=to_display(vector( _tip,  _tip, 0.35)), text="M1\n---", height=13, box=False, color=color.yellow)
lbl_m2 = label(pos=to_display(vector( _tip, -_tip, 0.35)), text="M2\n---", height=13, box=False, color=color.yellow)
lbl_m3 = label(pos=to_display(vector(-_tip, -_tip, 0.35)), text="M3\n---", height=13, box=False, color=color.yellow)
lbl_m4 = label(pos=to_display(vector(-_tip,  _tip, 0.35)), text="M4\n---", height=13, box=False, color=color.yellow)
del _tip, _off

# -----------------------------
# Rotation helper
# -----------------------------
def rotate_model(roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
    # Body frame: +X forward (away), +Y left, +Z up.
    fwd = vector(1, 0, 0)
    left = vector(0, 1, 0)
    up_vec = vector(0, 0, 1)

    # Convert to radians
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    # Yaw about body-up axis.
    fwd = fwd.rotate(angle=yaw, axis=up_vec)
    left = left.rotate(angle=yaw, axis=up_vec)

    # Pitch about body-left axis, then roll about body-forward axis.
    fwd = fwd.rotate(angle=pitch, axis=left)
    up_vec = up_vec.rotate(angle=pitch, axis=left)

    left = left.rotate(angle=roll, axis=fwd)
    up_vec = up_vec.rotate(angle=roll, axis=fwd)

    # Apply orientation to all objects
    body.axis = to_display(fwd)
    arm1.axis = to_display(fwd)
    nose.axis = to_display(fwd.norm() * 1.3)

    body.up = to_display(up_vec)
    arm1.up = to_display(up_vec)
    nose.up = to_display(up_vec)

    # arm2 is the lateral arm (+Y left / -Y right) in body frame
    arm2.axis = to_display(left.norm() * 3.0)
    arm2.up = to_display(up_vec)

    # Reposition motor labels at each arm tip (slightly above the arm plane)
    f = fwd.norm() * ARM_TIP
    s = left.norm() * ARM_TIP
    u = up_vec.norm() * 0.35
    lbl_m1.pos = to_display(f + s + u)   # front-left
    lbl_m2.pos = to_display(f - s + u)   # front-right
    lbl_m3.pos = to_display(-f - s + u)  # back-right
    lbl_m4.pos = to_display(-f + s + u)  # back-left


# -----------------------------
# Main loop
# -----------------------------
def available_ports() -> str:
    ports = sorted(p.device for p in list_ports.comports())
    return ", ".join(ports) if ports else "none"


def pick_port() -> str | None:
    ports = list(list_ports.comports())
    if not ports:
        return None

    if PORT:
        for p in ports:
            if p.device.upper() == PORT.upper():
                return p.device
        return None

    # Prefer common USB-UART adapters used by ESP32 dev boards.
    preferred_tags = ("CP210", "CH340", "USB", "UART", "ESP32")
    for p in ports:
        desc = (p.description or "").upper()
        hwid = (p.hwid or "").upper()
        if any(tag in desc or tag in hwid for tag in preferred_tags):
            return p.device

    return ports[0].device


def parse_angles(raw: str):
    cleaned = raw.strip()
    if not cleaned:
        return None

    # Accept labeled formats like: "roll:1.2,pitch:-2.3,yaw:10.0,altitude:123.4,m1:1500,m2:1520,m3:1480,m4:1510"
    lower = cleaned.lower()
    if "roll" in lower and "pitch" in lower and "yaw" in lower:
        tokenized = cleaned.replace(";", ",").split(",")
        values = {}
        for token in tokenized:
            t = token.strip()
            if ":" in t:
                k, v = t.split(":", 1)
            elif "=" in t:
                k, v = t.split("=", 1)
            else:
                continue
            key = k.strip().lower()
            if key == "yawdbg":
                key = "yaw"
            if key in ("roll", "pitch", "yaw", "altitude", "alt", "alt_ft", "m1", "m2", "m3", "m4"):
                values[key] = float(v.strip())
        if all(k in values for k in ("roll", "pitch", "yaw")):
            altitude = values.get("altitude", values.get("alt", values.get("alt_ft")))
            motors = {k: values[k] for k in ("m1", "m2", "m3", "m4") if k in values}
            return values["roll"], values["pitch"], values["yaw"], altitude, motors

    # Accept plain CSV: "roll,pitch,yaw[,altitude[,m1,m2,m3,m4]]"
    parts = [p.strip() for p in cleaned.replace(";", ",").split(",") if p.strip()]
    if len(parts) >= 3:
        roll = float(parts[0])
        pitch = float(parts[1])
        yaw = float(parts[2])
        altitude = float(parts[3]) if len(parts) >= 4 else None
        motors = {}
        if len(parts) >= 8:
            motors = {
                "m1": float(parts[4]),
                "m2": float(parts[5]),
                "m3": float(parts[6]),
                "m4": float(parts[7]),
            }
        return roll, pitch, yaw, altitude, motors

    return None


try:
    selected_port = pick_port()
    if not selected_port:
        raise serial.SerialException("No serial ports found")

    # Non-blocking serial setup reduces display latency.
    ser = serial.Serial(selected_port, BAUD, timeout=0, write_timeout=0, inter_byte_timeout=0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"Connected to {selected_port} @ {BAUD} baud")
except serial.SerialException as e:
    ser = None
    requested = PORT if PORT else "auto-detected port"
    print(f"Could not open {requested}: {e}")
    print(f"Available ports: {available_ports()}")
    print("Close Serial Monitor/TeraTerm/other scripts using the port, then restart.")
    info.text = "Serial error. Close other app using port and restart."

calibration_done = False

while True:
    rate(200)

    if ser is None:
        continue

    try:
        waiting = ser.in_waiting
        if waiting:
            RX_BUFFER.extend(ser.read(waiting))
            # Keep bounded buffer in case sender floods data.
            if len(RX_BUFFER) > 4096:
                del RX_BUFFER[:-2048]
    except serial.SerialException:
        ser = None
        info.text = "Serial disconnected. Reconnect and restart."
        continue

    last_line = None
    while b"\n" in RX_BUFFER:
        line, _, remainder = RX_BUFFER.partition(b"\n")
        RX_BUFFER[:] = remainder
        if line:
            last_line = line

    # Only process the latest full sample to avoid backlog lag.
    if last_line is None:
        continue

    raw = last_line.decode(errors="ignore").strip()
    if not raw:
        continue

    if raw.startswith("CALIBRATING..."):
        progress = raw[len("CALIBRATING..."):].strip()
        status = progress if progress else raw
        print(f"calibrating: {status}")
        info.text = f"calibrating: {status}"
        continue

    if "CALIBRATION DONE" in raw:
        calibration_done = True
        print("CALIBRATION DONE")
        info.text = "Calibration done. Reading roll, pitch, yaw, altitude..."
        continue

    try:
        angles = parse_angles(raw)
        if angles is None:
            if not calibration_done:
                continue
            continue

        roll, pitch, yaw, altitude, motors = angles

        # If numeric data is already streaming, calibration happened before startup.
        if not calibration_done:
            calibration_done = True
            print("Calibration assumed complete (numeric stream detected).")

        rotate_model(roll, pitch, yaw)
        alt_text = f"{altitude:.2f}" if altitude is not None else "N/A"
        info.text = (
            f"Roll: {roll:.2f} deg   Pitch: {pitch:.2f} deg   "
            f"Yaw: {yaw:.2f} deg   Alt: {alt_text}"
        )

        # Update motor speed labels
        for key, lbl in (("m1", lbl_m1), ("m2", lbl_m2), ("m3", lbl_m3), ("m4", lbl_m4)):
            val = motors.get(key)
            lbl.text = f"{key.upper()}\n{int(val)}" if val is not None else f"{key.upper()}\n---"

    except ValueError:
        continue