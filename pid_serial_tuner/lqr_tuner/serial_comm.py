import time
import serial
from serial.tools import list_ports

BAUD = 115200
ser = None
# Bytes recebidos ainda sem '\n'. Ler em blocos (in_waiting) e muito mais
# rapido que readline() byte a byte; se o leitor atrasa, o buffer do driver
# serial do Windows estoura e linhas velhas se misturam com novas.
_rx = bytearray()


def serial_is_open():
    return ser is not None and ser.is_open


def list_serial_ports():
    return [p.device for p in list_ports.comports()]


def open_serial(port):
    global ser
    close_serial()
    ser = serial.Serial(port, BAUD, timeout=0.4, write_timeout=0.8)
    try:
        ser.set_buffer_size(rx_size=1 << 20, tx_size=4096)  # so existe no Windows
    except Exception:
        pass


def close_serial():
    global ser
    try:
        if ser is not None and ser.is_open:
            ser.close()
    except Exception:
        pass
    ser = None
    _rx.clear()


def drain():
    if not serial_is_open():
        return
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        _rx.clear()
    except Exception:
        pass
    time.sleep(0.03)


def send_line(s: str):
    if not serial_is_open():
        raise RuntimeError("Porta serial não conectada.")
    ser.write((s + "\n").encode())
    ser.flush()


def read_line():
    """Retorna a proxima linha (sem '\n'), ou "" se nada chegar em ~timeout."""
    if not serial_is_open():
        return ""
    deadline = time.monotonic() + (ser.timeout or 0.4)
    while True:
        i = _rx.find(b"\n")
        if i >= 0:
            line = bytes(_rx[:i])
            del _rx[:i + 1]
            return line.decode(errors="ignore").strip()
        waiting = ser.in_waiting
        chunk = ser.read(waiting if waiting else 1)
        if chunk:
            _rx.extend(chunk)
        elif time.monotonic() >= deadline:
            return ""


def read_until_prefix(prefix: str, tmo=2.0, on_line=None):
    if not serial_is_open():
        return None
    t0 = time.monotonic()
    while time.monotonic() - t0 < tmo:
        line = read_line()
        if not line:
            continue
        if on_line is not None:
            on_line(line)
        if line.startswith(prefix):
            return line
    return None
