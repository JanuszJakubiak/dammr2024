## Check the topic name - define your own name!
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SerialDataPublisher(Node):

    def __init__(self):
        super().__init__('serial_data_publisher')
        self.publisher_ = self.create_publisher(String, 'dwm1000_string', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.read_serial_data)
        
        self.ser = None
        self.buffer = ""

        self.initialize_serial_connection()

    def initialize_serial_connection(self):
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
            
            self.send_newlines_until_prompt()
            self.send_lep_command()
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.destroy_node()

    def send_newlines_until_prompt(self):
        self.get_logger().info("Sending new lines until prompt is received...")
        while True:
            self.ser.write(b'\n')
            if self.wait_for_prompt():
                break

    def send_lep_command(self):
        self.get_logger().info("Sending 'lep' command...")
        self.ser.write(b'lep\n')

    def wait_for_prompt(self, prompt="dwm>"):
        buffer = ""
        while True:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1).decode('utf-8', errors='ignore')
                buffer += byte
                if buffer.endswith(prompt):
                    self.get_logger().info("Prompt received.")
                    return True
        return False

    def read_serial_data(self):
        if self.ser and self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                if line.startswith("POS"):
                    parts = line.split(',')
                    if len(parts) == 5:
                        msg = String()
                        msg.data = ','.join(parts[1:4])
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Publishing: {msg.data}")

    def destroy(self):
        if self.ser:
            try:
                self.get_logger().info("Sending 'reset' command...")
                #self.ser.write(b'reset\n')
                time.sleep(1)
            except Exception as e:
                self.get_logger().error(f"An error occurred while sending reset: {e}")
            finally:
                self.ser.close()
                self.get_logger().info("Serial connection closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_data_publisher = SerialDataPublisher()
    try:
        rclpy.spin(serial_data_publisher)
    except KeyboardInterrupt:
        serial_data_publisher.get_logger().info("Exiting on user request.")
    finally:
        serial_data_publisher.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
