import time
import serial
from serial.tools import list_ports

BAUD = 115200
ser = None


def serial_is_open():
    return ser is not None and ser.is_open


def list_serial_ports():
    return [p.device for p in list_ports.comports()]


def open_serial(port):
    global ser
    close_serial()
    ser = serial.Serial(port, BAUD, timeout=0.4, write_timeout=0.8)


def close_serial():
    global ser
    try:
        if ser is not None and ser.is_open:
            ser.close()
    except Exception:
        pass
    ser = None


def drain():
    if not serial_is_open():
        return
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass
    time.sleep(0.03)


def send_line(s: str):
    if not serial_is_open():
        raise RuntimeError("Porta serial não conectada.")
    ser.write((s + "\n").encode())
    ser.flush()


def read_line():
    if not serial_is_open():
        return ""
    return ser.readline().decode(errors="ignore").strip()


def read_until_prefix(prefix: str, tmo=2.0):
    if not serial_is_open():
        return None
    t0 = time.time()
    while time.time() - t0 < tmo:
        line = read_line()
        if not line:
            continue
        if line.startswith(prefix):
            return line
    return None
