import serial
import time

# Open serial connection to the Giga (adjust if using a different port)
ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(2)  # Give Arduino time to reset after serial open

# Example speed values for 6 motors (can be from -800 to +800)
motor_speeds = [0, 0, 0, 0, 0, 0]

# Convert list to comma-separated string
command = ','.join(str(int(s)) for s in motor_speeds) + '\n'

# Send the command over serial
ser.write(command.encode())
print(f"Command sent: {command.strip()}")

# Optional: wait and close
time.sleep(0.5)
ser.close()
