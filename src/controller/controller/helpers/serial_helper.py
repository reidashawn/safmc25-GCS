import serial
import time

class SerialHelper:
    def __init__(self, com_port, baud_rate=115200, timeout=1):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = serial.Serial(com_port, baudrate=baud_rate, timeout=timeout)
    
    def restart(self):
        self.ser = serial.Serial(self.com_port, baudrate=self.baud_rate, timeout=self.timeout)
        
    def clear_buffer(self):
        """
        Clears the serial input buffer by reading and discarding any unread data.
        """
        try:
            self.ser.reset_input_buffer()  # Clears the input buffer
            # print("Serial buffer cleared.")
        except serial.SerialException as e:
            print(f"Error clearing the serial buffer: {e}")
    
    def read_from_serial(self):
        """
        Reads data from an Arduino connected to the specified COM port after clearing the buffer.
        
        Returns:
            string: A string containing data sent. Returns None if no valid data is read.
        """
        try:
            self.clear_buffer()  # Clear the buffer before reading
            # Wait until data is available and then read it
            while self.ser.in_waiting == 0:  # Wait for data to arrive
                time.sleep(0.1)  # Small delay to avoid busy-waiting
                
            # Read the line of data once it's available
            line = self.ser.readline().decode('utf-8').strip()
            return line
        except:
            print(f"Error reading from serial port: {e}")
            return None

    def write_to_serial(self, message):
        """
        Write a string to the specified serial port.
        
        :param message: The string to send to the serial device.
        """
        try:
            self.ser.write(message.encode('utf-8'))
            print(f"Message sent: {message}")
        except serial.SerialException as e:
            print(f"Error writing to serial port: {e}")
