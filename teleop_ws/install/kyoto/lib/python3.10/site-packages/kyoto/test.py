import pygame
import tkinter as tk
from tkinter import ttk

class XboxDebugGUI:
    def __init__(self):
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("Xbox Controller Debug")
        self.root.geometry("400x600")
        
        # Try to connect to controller
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        
        # Create GUI elements
        self.create_widgets()
        
        # Start update loop
        self.update()
        
    def create_widgets(self):
        # Connection status
        self.status_label = ttk.Label(self.root, text="Status: Disconnected")
        self.status_label.pack(pady=10)
        
        # Create frame for axes
        axes_frame = ttk.LabelFrame(self.root, text="Axes", padding=10)
        axes_frame.pack(fill='x', padx=10)
        
        # Create labels for each axis
        self.axis_labels = []
        axis_names = [
            "Left Stick X (0)",
            "Left Stick Y (1)",
            "Left Trigger (2)",
            "Right Stick X (3)",
            "Right Stick Y (4)",
            "Right Trigger (5)"
        ]
        
        for i in range(6):
            label = ttk.Label(axes_frame, text=f"{axis_names[i]}: 0.000")
            label.pack(anchor='w')
            self.axis_labels.append(label)
            
        # Create frame for buttons
        buttons_frame = ttk.LabelFrame(self.root, text="Buttons", padding=10)
        buttons_frame.pack(fill='x', padx=10, pady=10)
        
        # Create labels for each button
        self.button_labels = []
        button_names = [
            "A (0)", "B (1)", "X (2)", "Y (3)",
            "LB (4)", "RB (5)", "Back (6)", "Start (7)",
            "L3 (8)", "R3 (9)"
        ]
        
        for i in range(10):
            label = ttk.Label(buttons_frame, text=f"{button_names[i]}: OFF")
            label.pack(anchor='w')
            self.button_labels.append(label)
            
        # D-pad display
        dpad_frame = ttk.LabelFrame(self.root, text="D-Pad", padding=10)
        dpad_frame.pack(fill='x', padx=10)
        self.dpad_label = ttk.Label(dpad_frame, text="Hat: (0, 0)")
        self.dpad_label.pack()
        
    def update(self):
        if self.joystick:
            pygame.event.pump()
            
            # Update connection status
            self.status_label.config(text="Status: Connected")
            
            # Update axes
            for i in range(6):
                try:
                    value = round(self.joystick.get_axis(i), 3)
                    self.axis_labels[i].config(text=f"{self.axis_labels[i]['text'].split(':')[0]}: {value}")
                except pygame.error:
                    pass
                    
            # Update buttons
            for i in range(10):
                try:
                    state = "ON" if self.joystick.get_button(i) else "OFF"
                    self.button_labels[i].config(text=f"{self.button_labels[i]['text'].split(':')[0]}: {state}")
                except pygame.error:
                    pass
                    
            # Update D-pad
            try:
                hat = self.joystick.get_hat(0)
                self.dpad_label.config(text=f"Hat: {hat}")
            except pygame.error:
                pass
        else:
            # Try to connect if not connected
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
            else:
                self.status_label.config(text="Status: Disconnected")
                
        # Schedule next update
        self.root.after(50, self.update)  # Update every 50ms (20Hz)
        
    def run(self):
        self.root.mainloop()
        
    def cleanup(self):
        pygame.quit()

if __name__ == "__main__":
    app = XboxDebugGUI()
    try:
        app.run()
    finally:
        app.cleanup()