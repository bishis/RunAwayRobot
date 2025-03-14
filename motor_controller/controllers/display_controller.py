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
        
        self.buzzer_pin = 21
        self.buzzer = None
        self.is_buzzer_active = False
        self.buzzer_thread = None
        
        # Add animation control
        self.animation_thread = None
        self.is_animation_active = False
        
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
    
    def show_text(self, text, clear_first=True):
        """
        Display text on the OLED display
        
        Args:
            text: String or list of strings to display
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
            
            # Process input text
            if isinstance(text, str):
                # Split single string into lines
                text_lines = text.split('\n')
            elif isinstance(text, list):
                # Use provided list
                text_lines = text
            else:
                # Convert anything else to string
                text_lines = [str(text)]
            
            # Handle word wrapping for long lines
            wrapped_lines = []
            max_chars_per_line = 21  # Approximate for default font
            
            for line in text_lines:
                if len(line) <= max_chars_per_line:
                    wrapped_lines.append(line)
                else:
                    # Simple word wrapping
                    words = line.split()
                    current_line = ""
                    
                    for word in words:
                        test_line = current_line + " " + word if current_line else word
                        if len(test_line) <= max_chars_per_line:
                            current_line = test_line
                        else:
                            wrapped_lines.append(current_line)
                            current_line = word
                    
                    if current_line:
                        wrapped_lines.append(current_line)
            
            # Draw each line of text
            line_height = 10
            for i, line in enumerate(wrapped_lines):
                if i * line_height >= self.oled.height:
                    break  # Don't draw beyond display boundaries
                
                y_pos = i * line_height
                draw.text((0, y_pos), line, font=font, fill=255)
            
            # Display the image
            self.oled.display(image)
        
        except Exception as e:
            self.node.get_logger().error(f'Error displaying text: {str(e)}')
    
    def sound_buzzer(self, pattern=None, duration=2.0, frequency=1000, volume=0.5):
        """
        Sound the buzzer with a specific pattern and frequency
        
        Args:
            pattern: List of (frequency, on_time, off_time) tuples or a complete jingle
                    If None, buzzer will sound at specified frequency for 'duration' seconds
            duration: Total duration to sound if pattern is None, or max time to play pattern
            frequency: Tone frequency in Hz (if pattern is None)
            volume: Volume level as duty cycle (0.0 to 1.0)
        """
        # Cancel any existing buzzer thread
        self.stop_buzzer()
        
        if callable(pattern):
            pattern = pattern()
        
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
                self.node.get_logger().info(f'No pattern, playing tone at {frequency} Hz for {duration} seconds')
                self.buzzer.frequency = frequency
                self.buzzer.value = volume
                time.sleep(duration)
                self.buzzer.off()
            else:
                # If this is a single iteration pattern, play it once
                if isinstance(pattern, list) and all(isinstance(x, tuple) for x in pattern):
                    if len(pattern) > 0 and len(pattern[0]) == 3:
                        for freq, vol, dur in pattern:
                            if not self.is_buzzer_active:
                                break
                            
                            # Play tone at specified frequency and volume
                            self.buzzer.frequency = freq
                            self.buzzer.value = vol
                            time.sleep(dur)
                            
                            self.buzzer.off()
            
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
        
        """
        self.sound_buzzer(frequency=frequency, duration=duration, volume=volume)
    
    def show_image(self, image_source, clear_first=True):
        """
        Display an image on the OLED display
        
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
    
    def show_gif(self, gif_path, loops=1, clear_first=True):
        """
        Display an animated GIF on the OLED display
        """
        if not self.is_display_active or self.oled is None:
            self.node.get_logger().warn('Display not active, cannot show GIF')
            return
        
        # Stop any existing animation thread
        self.stop_animation()
        
        try:
            # Load the GIF file
            gif = Image.open(gif_path)
            
            # Check if it's actually a GIF
            if not hasattr(gif, 'is_animated') or not gif.is_animated:
                self.node.get_logger().warn(f'Image at {gif_path} is not an animated GIF')
                # Just display as a static image
                self.show_image(gif_path, clear_first)
                return
            
            # Start animation thread
            self.animation_thread = threading.Thread(
                target=self._animation_thread_function,
                args=(gif, loops, clear_first),
                daemon=True
            )
            self.is_animation_active = True
            self.animation_thread.start()
            self.node.get_logger().info(f'Started GIF animation with {gif.n_frames} frames')
        
        except Exception as e:
            self.node.get_logger().error(f'Error displaying GIF: {str(e)}')
    
    def _animation_thread_function(self, gif, loops, clear_first):
        """Thread function to handle GIF animations"""
        try:
            # Clear display if requested
            if clear_first and self.oled is not None:
                self.oled.clear()
            
            # Get the number of frames
            n_frames = gif.n_frames
            current_loop = 0
            
            # Loop through the animation
            while self.is_animation_active and (loops == 0 or current_loop < loops):
                for frame_idx in range(n_frames):
                    if not self.is_animation_active:
                        break
                    
                    # Seek to the specific frame
                    gif.seek(frame_idx)
                    
                    # Get the frame duration in milliseconds
                    try:
                        # Some GIFs don't have duration info for frames
                        duration = gif.info.get('duration', 100) / 1000  # Convert to seconds
                    except:
                        duration = 0.1  # Default to 100ms
                    
                    # Extract the frame
                    frame = gif.copy().convert('1').resize((self.oled.width, self.oled.height))
                    
                    # Display the frame
                    if self.oled is not None:
                        self.oled.display(frame)
                    
                    # Wait for the frame duration
                    time.sleep(duration)
                
                # Increment loop counter
                current_loop += 1
                
                # Reset to first frame if we're doing another loop
                if self.is_animation_active and (loops == 0 or current_loop < loops):
                    gif.seek(0)
            
            self.node.get_logger().debug('GIF animation finished')
        except Exception as e:
            self.node.get_logger().error(f'Error in animation thread: {str(e)}')
        finally:
            self.is_animation_active = False
    
    def stop_animation(self):
        """Stop any currently playing animation"""
        if hasattr(self, 'animation_thread') and self.animation_thread is not None:
            self.is_animation_active = False
            # Wait for animation thread to end, but not too long
            if self.animation_thread.is_alive():
                self.animation_thread.join(timeout=0.5)
            self.animation_thread = None
    
    def cleanup(self):
        """Clean up resources"""
        # Stop animation
        self.stop_animation()
        
        # Stop buzzer
        self.stop_buzzer()
        
        # Clear display
        if self.is_display_active and self.oled is not None:
            try:
                self.clear_display()
            except:
                pass        
        self.node.get_logger().info('Display and buzzer resources cleaned up')
