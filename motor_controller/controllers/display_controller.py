#!/usr/bin/env python3
import time
import threading
from PIL import Image, ImageDraw, ImageFont
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from gpiozero import PWMOutputDevice 
import rclpy

class DisplayController:
    """Controller for buzzer and OLED display interactions using luma.oled and PWM buzzer"""
    
    def __init__(self, node):
        """Initialize the display controller with the given ROS node"""
        self.node = node
        
        # Initialize OLED display
        self.oled = None
        self.is_display_active = False
        
        # Initialize buzzer with GPIO Zero PWM
        self.buzzer_pin = 25
        self.buzzer = None
        self.is_buzzer_active = False
        self.buzzer_thread = None
        
        # Initialize components
        self.setup_display()
        self.setup_buzzer()
        
        self.node.get_logger().info('Display and buzzer controller initialized')
    
    def setup_display(self):
        """Initialize the OLED display using luma.oled"""
        try:
            # Create the I2C interface
            serial = i2c(port=1, address=0x3C)  # Change to 0x3D if needed
            
            # Create SSD1306 OLED object (128x64)
            self.oled = ssd1306(serial, width=128, height=64)
            
            # Clear the display
            self.oled.clear()
            
            # Set flag indicating successful initialization
            self.is_display_active = True
            self.node.get_logger().info('OLED display initialized successfully using luma.oled')
        
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize OLED display: {str(e)}')
            self.is_display_active = False
    
    def setup_buzzer(self):
        """Initialize the buzzer using GPIO Zero PWM"""
        try:
            # Set up buzzer using GPIO Zero PWM for tone generation
            self.buzzer = PWMOutputDevice(self.buzzer_pin)
            self.node.get_logger().info('PWM Buzzer initialized successfully')
        
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize buzzer: {str(e)}')
    
    def clear_display(self):
        """Clear the OLED display"""
        if not self.is_display_active or self.oled is None:
            return
        
        try:
            self.oled.clear()
        except Exception as e:
            self.node.get_logger().error(f'Error clearing display: {str(e)}')
    
    def show_text(self, text_lines, clear_first=True):
        """
        Display text on the OLED display
        
        Args:
            text_lines: List of strings to display
            clear_first: Whether to clear the display first
        """
        if not self.is_display_active or self.oled is None:
            return
        
        try:
            # Create blank image for drawing
            image = Image.new("1", (self.oled.width, self.oled.height))
            draw = ImageDraw.Draw(image)
            
            # Load default font
            font = ImageFont.load_default()
            
            # Clear display if requested
            if clear_first:
                self.oled.clear()
            
            # Draw each line of text
            line_height = 10
            for i, line in enumerate(text_lines):
                y_pos = i * line_height
                draw.text((0, y_pos), line, font=font, fill=255)
            
            # Display the image
            self.oled.display(image)
        
        except Exception as e:
            self.node.get_logger().error(f'Error displaying text: {str(e)}')
    
    def sound_buzzer(self, pattern=None, duration=1.0, frequency=1000, volume=0.5):
        """
        Sound the buzzer with a specific pattern and frequency
        
        Args:
            pattern: List of (frequency, on_time, off_time) tuples
                    If None, buzzer will sound at specified frequency for 'duration' seconds
            duration: Total duration to sound if pattern is None
            frequency: Tone frequency in Hz (if pattern is None)
            volume: Volume level as duty cycle (0.0 to 1.0)
        """
        # Cancel any existing buzzer thread
        self.stop_buzzer()
        
        # Start new buzzer thread
        self.buzzer_thread = threading.Thread(
            target=self._buzzer_thread_function,
            args=(pattern, duration, frequency, volume),
            daemon=True
        )
        self.buzzer_thread.start()
    
    def _buzzer_thread_function(self, pattern, duration, frequency, volume):
        """Thread function to handle buzzer patterns using GPIO Zero PWM"""
        try:
            self.is_buzzer_active = True
            
            if self.buzzer is None:
                self.node.get_logger().warn('Buzzer not initialized, cannot sound')
                return
                
            if pattern is None:
                # Simple tone for specified duration
                self.buzzer.frequency = frequency
                self.buzzer.value = volume
                time.sleep(duration)
                self.buzzer.off()
            else:
                # Pattern-based tones
                start_time = time.time()
                while time.time() - start_time < duration and self.is_buzzer_active:
                    for freq, on_time, off_time in pattern:
                        if not self.is_buzzer_active:
                            break
                        
                        # Play tone at specified frequency
                        self.buzzer.frequency = freq
                        self.buzzer.value = volume
                        time.sleep(on_time)
                        
                        # Pause between tones
                        self.buzzer.off()
                        time.sleep(off_time)
            
            # Ensure buzzer is off when done
            self.buzzer.off()
            self.is_buzzer_active = False
        
        except Exception as e:
            self.node.get_logger().error(f'Error in buzzer thread: {str(e)}')
            # Ensure buzzer is off on error
            try:
                if self.buzzer is not None:
                    self.buzzer.off()
            except:
                pass
            self.is_buzzer_active = False
    
    def stop_buzzer(self):
        """Stop the buzzer immediately"""
        self.is_buzzer_active = False
        try:
            if self.buzzer is not None:
                self.buzzer.off()
        except Exception as e:
            self.node.get_logger().error(f'Error stopping buzzer: {str(e)}')
    
    def play_tone(self, frequency, duration, volume=0.5):
        """
        Play a single tone at the specified frequency
        
        Args:
            frequency: Tone frequency in Hz
            duration: Duration in seconds
            volume: Volume level (0.0 to 1.0)
        """
        self.sound_buzzer(frequency=frequency, duration=duration, volume=volume)
    
    def cleanup(self):
        """Clean up resources"""
        # Stop buzzer
        self.stop_buzzer()
        
        # Clear display
        if self.is_display_active and self.oled is not None:
            try:
                self.clear_display()
            except:
                pass        
        self.node.get_logger().info('Display and buzzer resources cleaned up')
