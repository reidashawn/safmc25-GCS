from std_msgs.msg import Int32
import time

class Button:
    def __init__(self, topic, long_press_time=1, press_callback=None, release_callback=None, short_callback=None, long_callback=None):
        self.topic = topic
        # self.node = node
        # self.sub = self.node.create_subscription(Int32, topic, self.data_callback, 10)
        self.press_callback = press_callback
        self.release_callback = release_callback
        self.short_callback = short_callback
        self.long_callback = long_callback
        self.last_state = 0
        self.last_time = None
        self.long_press_time = long_press_time
    
    def data_callback(self, msg):
        if msg.data == 1:
            if self.press_callback is not None:
                self.press_callback() 
            if self.last_state == 0:
                self.last_state = 1
                self.last_time = time.time()
                if self.short_callback is not None and self.long_callback is None:
                    self.short_callback()
            elif (time.time() - self.last_time) > self.long_press_time and self.long_callback is not None:
                self.long_callback()
        else:
            if self.last_state == 1:
                if self.short_callback is not None and self.long_callback is not None and (time.time() - self.last_time) < self.long_press_time:
                    self.short_callback()
                elif self.release_callback is not None:
                    self.release_callback()
            self.last_state = 0

