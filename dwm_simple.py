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
            ser.write(b'\n')
            wait_for_prompt(ser)

            while True:
                # Wait for user command
                user_input = input("Enter command: ")
                if user_input.lower() in ['exit', 'quit']:
                    print("Exiting...")
                    return

                # Send user command to the device
                ser.write((user_input + '\n').encode('utf-8'))

                # Read and print the response from the device
                wait_for_prompt(ser)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
