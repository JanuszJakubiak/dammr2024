import serial
import time

# Function to read from the serial port until a specific string is found
def wait_for_prompt(ser, prompt="dwm>"):
    buffer = ""
    while True:
        if ser.in_waiting > 0:
            byte = ser.read(1).decode('utf-8', errors='ignore')
            buffer += byte
            print(byte, end='', flush=True)  # Optional: Print received bytes for debugging
            if buffer.endswith(prompt):
                print("\nPrompt received.")
                break

def main():
    # Connect to the serial port
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        print("Sending new lines until prompt is received...")
        while True:
            ser.write(b'\n\n\n')
            wait_for_prompt(ser)

            # Send "lep" command after receiving the first prompt
            print("Sending 'lep' command...")
            ser.write(b'lep\n')

            # Read the continuous stream of data and process full lines
            print("Printing stream from device (Press Ctrl+C to exit)...")
            buffer = ""
            while True:
                if ser.in_waiting > 0:
                    buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.startswith("POS"):
                            parts = line.split(',')
                            if len(parts) == 5:
                                print(','.join(parts[1:4]), flush=True)

    except KeyboardInterrupt:
        print("\nExiting on user request.")
        try:
            print("Sending 'reset' command...")
            #ser.write(b'reset\n')
            # Wait for the device to process the reset command
            time.sleep(1)
        except Exception as e:
            print(f"An error occurred while sending reset: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
