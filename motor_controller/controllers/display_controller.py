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
    
    def show_image(self, image_source, clear_first=True):
        """
        Display an image on the OLED display
        
        Args:
            image_source: Either a file path (string) or a PIL Image object
            clear_first: Whether to clear the display first
        """
        if not self.is_display_active or self.oled is None:
            return
        
        try:
            # Clear display if requested
            if clear_first:
                self.oled.clear()
            
            # Load image if a file path is provided
            if isinstance(image_source, str):
                image = Image.open(image_source)
            elif isinstance(image_source, Image.Image):
                image = image_source
            else:
                self.node.get_logger().error('Image source must be a file path or PIL Image object')
                return
            
            # Resize image to fit the display
            image = image.resize((self.oled.width, self.oled.height))
            
            # Convert to black and white (mode "1")
            if image.mode != '1':
                image = image.convert('1')
            
            # Display the image
            self.oled.display(image)
        
        except Exception as e:
            self.node.get_logger().error(f'Error displaying image: {str(e)}')
    
    def show_robot_status_screen(self, status_text, battery_level=None, signal_level=None):
        """
        Display a status screen with a robot icon and status information
        
        Args:
            status_text: Main status text to display
            battery_level: Battery level (0-100) or None
            signal_level: Signal strength (0-100) or None
        """
        if not self.is_display_active or self.oled is None:
            return
        
        try:
            # Create new image
            image = Image.new("1", (self.oled.width, self.oled.height))
            draw = ImageDraw.Draw(image)
            
            # Load default font
            font = ImageFont.load_default()
            small_font = ImageFont.load_default()
            
            # Draw a small robot icon (simple rectangle with wheels)
            icon_x, icon_y = 5, 5
            icon_width, icon_height = 20, 20
            
            # Robot body
            draw.rectangle(
                [(icon_x, icon_y), (icon_x + icon_width, icon_y + icon_height)],
                outline=1,
                fill=0
            )
            
            # Robot wheels
            draw.rectangle(
                [(icon_x - 3, icon_y + 5), (icon_x, icon_y + icon_height - 5)],
                outline=1,
                fill=1
            )
            draw.rectangle(
                [(icon_x + icon_width, icon_y + 5), (icon_x + icon_width + 3, icon_y + icon_height - 5)],
                outline=1,
                fill=1
            )
            
            # Robot eyes
            draw.rectangle(
                [(icon_x + 5, icon_y + 5), (icon_x + 8, icon_y + 8)],
                outline=0,
                fill=1
            )
            draw.rectangle(
                [(icon_x + 12, icon_y + 5), (icon_x + 15, icon_y + 8)],
                outline=0,
                fill=1
            )
            
            # Status text
            draw.text((icon_x + icon_width + 10, icon_y), status_text, font=font, fill=1)
            
            # Draw battery level if provided
            if battery_level is not None:
                battery_y = icon_y + icon_height + 5
                draw.text((5, battery_y), f"Battery: {battery_level}%", font=small_font, fill=1)
            
            # Draw signal level if provided
            if signal_level is not None:
                signal_y = icon_y + icon_height + 15
                draw.text((5, signal_y), f"Signal: {signal_level}%", font=small_font, fill=1)
            
            # Display the image
            self.oled.display(image)
            
        except Exception as e:
            self.node.get_logger().error(f'Error displaying robot status screen: {str(e)}')
    
    def show_alert(self, alert_message, alert_type='warning'):
        """
        Display an alert with an appropriate icon
        
        Args:
            alert_message: The alert text to display
            alert_type: Type of alert ('warning', 'error', 'info')
        """
        if not self.is_display_active or self.oled is None:
            return
        
        try:
            # Create new image
            image = Image.new("1", (self.oled.width, self.oled.height))
            draw = ImageDraw.Draw(image)
            
            # Load default font
            font = ImageFont.load_default()
            
            # Define icon position
            icon_x, icon_y = 5, 5
            icon_size = 15
            
            # Draw appropriate icon based on alert type
            if alert_type == 'warning':
                # Draw warning triangle
                points = [
                    (icon_x + icon_size//2, icon_y),  # Top
                    (icon_x, icon_y + icon_size),     # Bottom left
                    (icon_x + icon_size, icon_y + icon_size)  # Bottom right
                ]
                draw.polygon(points, outline=1, fill=0)
                # Exclamation mark
                draw.line([(icon_x + icon_size//2, icon_y + 3), 
                           (icon_x + icon_size//2, icon_y + icon_size - 4)], fill=1, width=1)
                draw.point((icon_x + icon_size//2, icon_y + icon_size - 2), fill=1)
                
            elif alert_type == 'error':
                # Draw error X
                draw.ellipse([(icon_x, icon_y), (icon_x + icon_size, icon_y + icon_size)], outline=1)
                draw.line([(icon_x + 3, icon_y + 3), (icon_x + icon_size - 3, icon_y + icon_size - 3)], fill=1)
                draw.line([(icon_x + icon_size - 3, icon_y + 3), (icon_x + 3, icon_y + icon_size - 3)], fill=1)
                
            elif alert_type == 'info':
                # Draw info circle with i
                draw.ellipse([(icon_x, icon_y), (icon_x + icon_size, icon_y + icon_size)], outline=1)
                draw.line([(icon_x + icon_size//2, icon_y + 4), 
                           (icon_x + icon_size//2, icon_y + 5)], fill=1)
                draw.line([(icon_x + icon_size//2, icon_y + 7), 
                           (icon_x + icon_size//2, icon_y + icon_size - 4)], fill=1)
            
            # Draw alert message (wrap text if needed)
            text_x = icon_x + icon_size + 5
            text_y = icon_y
            
            # Simple text wrapping
            max_line_length = 16  # Approximate character count that fits
            words = alert_message.split()
            lines = []
            current_line = ""
            
            for word in words:
                test_line = current_line + " " + word if current_line else word
                if len(test_line) <= max_line_length:
                    current_line = test_line
                else:
                    lines.append(current_line)
                    current_line = word
            
            if current_line:
                lines.append(current_line)
            
            # Draw each line
            line_height = 10
            for i, line in enumerate(lines):
                draw.text((text_x, text_y + i * line_height), line, font=font, fill=1)
            
            # Display the image
            self.oled.display(image)
            
        except Exception as e:
            self.node.get_logger().error(f'Error displaying alert: {str(e)}')
    
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
