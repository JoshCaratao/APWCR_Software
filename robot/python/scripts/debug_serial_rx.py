import time
import serial

PORT = "COM5"      # change if needed
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=0.2)

# Optional: avoid auto-reset loops. Try both ways if needed.
# ser.dtr = False
# time.sleep(0.2)
# ser.reset_input_buffer()

print("Opened", ser.port, "baud", ser.baudrate)

t0 = time.time()
while time.time() - t0 < 10:
    raw = ser.readline()
    if raw:
        print("RX:", raw[:200])
    else:
        print("RX: <none>")
    time.sleep(0.2)

ser.close()
