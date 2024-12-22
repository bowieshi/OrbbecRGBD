import tkinter as tk
from tkinter import scrolledtext, messagebox, StringVar
from tkinter import ttk
import subprocess
import threading
import yaml
import os
import psutil
import signal
import select

cur_path = os.path.dirname(os.path.realpath(__file__))
data_root = os.path.join(cur_path, "data")

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("RGBD Video Recorder")
        self.root.geometry("1000x800")

        self.process = None
        self.process_thread = None
        self.stop_event = threading.Event()

        self.button_state = "Record"
        default_font = ("Helvetica", 20)
        
        # Create a custom style for the OptionMenu
        style = ttk.Style()
        style.configure("TMenubutton", font=default_font)
        
        # Multi-choice Toggle Selection Bar
        self.res_option_menu_label = tk.Label(root, text="Resolution:", font=default_font)
        self.res_option_menu_label.grid(row=0, column=0, padx=0, pady=0)

        self.res_option_var = StringVar(root)
        self.res_option_var.set("1280x960")  # Default value
        self.res_option_menu = ttk.OptionMenu(root, self.res_option_var, "1280x960", "1920x1080", "2560x1440", "3840x2160")
        self.res_option_menu.grid(row=0, column=1, padx=0, pady=0)
        self.res_option_menu["menu"].config(font=default_font)


        self.fps_option_menu_label = tk.Label(root, text="FPS:", font=default_font)
        self.fps_option_menu_label.grid(row=1, column=0, padx=0, pady=0)

        self.fps_option_var = StringVar(root)
        self.fps_option_var.set("30")  # Default value
        self.fps_option_menu = ttk.OptionMenu(root, self.fps_option_var, "30", "25", "15", "5")
        self.fps_option_menu.grid(row=1, column=1, padx=0, pady=0)
        self.fps_option_menu["menu"].config(font=default_font)


        # Scene Name Label and Entry
        self.scene_label = tk.Label(root, text="Scene Name (str):", font=default_font)
        self.scene_label.grid(row=2, column=0, padx=0, pady=0)
        self.scene_var = StringVar(value="scene")
        self.scene_entry = tk.Entry(root, textvariable=self.scene_var, font=default_font)
        self.scene_entry.grid(row=2, column=1, padx=0, pady=0)
        
        self.exposure_label = tk.Label(root, text="Camera Exposure (int):", font=default_font)
        self.exposure_label.grid(row=3, column=0, padx=0, pady=0)
        self.exposure_var = StringVar(value="200")  # Default value for exposure
        self.exposure_entry = tk.Entry(root, textvariable=self.exposure_var, font=default_font)
        self.exposure_entry.grid(row=3, column=1, padx=0, pady=0)


        self.gain_label = tk.Label(root, text="Camera Gain (int):", font=default_font)
        self.gain_label.grid(row=4, column=0, padx=0, pady=0)
        self.gain_var = StringVar(value="0")
        self.gain_entry = tk.Entry(root, textvariable=self.gain_var, font=default_font)
        self.gain_entry.grid(row=4, column=1, padx=0, pady=0)


        self.button = tk.Button(root, text=self.button_state, command=self.toggle_command, width=10, height=2, font=default_font, bg="green")
        self.button.grid(row=5, column=0, columnspan=2, padx=0, pady=0)

        self.exit_button = tk.Button(root, text="Exit", command=self.exit_button, width=10, height=2, font=default_font, bg="red")
        self.exit_button.grid(row=6, column=0, columnspan=2, padx=0, pady=0)

        self.text_area = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=50, height=30, font=default_font)
        self.text_area.grid(row=7, column=0, columnspan=2, padx=0, pady=0)

    def toggle_command(self):
        if self.button_state == "Record":
            if not self.validate_input():
                return
            self.button_state = "Stop"
            self.button.config(text=self.button_state)
            self.start_command()
        else:
            self.button_state = "Record"
            self.button.config(text=self.button_state)
            self.stop_command()
    
    def exit_button(self):
        self.stop_command()
        root.quit()
    
    def validate_input(self):
        try:
            scene_name = self.scene_entry.get()
            exposure = int(self.exposure_entry.get())
            gain = int(self.gain_entry.get())
            if not scene_name:
                raise ValueError("Scene Name cannot be empty")
            self.save_to_yaml(scene_name, exposure, gain, self.res_option_var.get(), self.fps_option_var.get())
            return True
        except ValueError as e:
            messagebox.showerror("Input Error", str(e))
            return False
    
    def save_to_yaml(self, scene_name, exposure, gain, resolution, fps):
        self.scene_root = os.path.join(data_root, scene_name)
        os.makedirs(self.scene_root, exist_ok=True)
        self.config_root = os.path.join(data_root, scene_name, "config")
        os.makedirs(self.config_root, exist_ok=True)
        self.motion_root = os.path.join(data_root, scene_name, "motion")
        os.makedirs(self.motion_root, exist_ok=True)

        if resolution == "1280x960":
            width, height = 1280, 960
        elif resolution == "1920x1080":
            width, height = 1920, 1080
        elif resolution == "2560x1440":
            width, height = 2560, 1440
        elif resolution == "3840x2160":
            width, height = 3840, 2160
        if fps == "30":
            fps = 30
        elif fps == "25":
            fps = 25
        elif fps == "15":
            fps = 15
        elif fps == "5":
            fps = 5
        data = {
            "scene_name": scene_name, 
            "exposure": exposure,
            "gain": gain, 
            "fps": fps,
            "colorCamera.width": width, 
            "colorCamera.height": height
        }
        self.yaml_path = os.path.join(self.config_root, "scan.yaml")
        with open(self.yaml_path, "w") as file:
            yaml.dump(data, file)

    def start_command(self):
        self.text_area.delete(1.0, tk.END)
        self.stop_event.clear()
        self.process_thread = threading.Thread(target=self.run_commands)
        self.process_thread.start()


    def stop_command(self):
        print("Stop command")
        self.stop_event.set()

    def run_commands(self):
        # Define the paths
        script_path = os.path.join(cur_path, "run_commands.sh")

        # Construct the content of the bash script
        script_content = f'''#!/bin/zsh
cd {self.scene_root}
source /opt/ros/humble/setup.zsh
source {os.path.join(cur_path, 'camera_ws/install/setup.zsh')}
ros2 launch {os.path.join(cur_path, "camera_ws/launch/camera_launch.py")}'''


        # Write the script content to the bash script file
        with open(script_path, "w") as script_file:
            script_file.write(script_content)

        # Make the script executable
        os.chmod(script_path, 0o755)

        # Execute the shell script using zsh

        self.process = subprocess.Popen(["zsh", script_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # Monitor the process output
        while self.process.poll() is None:
            if self.stop_event.is_set():
                if self.process.poll() is None:  # Check if the process is still running
                    # os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                    self.terminate_process_tree(self.process.pid)
                    print("Terminated process and child processes")
                else:
                    print("Process already terminated")
                break
            
            # Use select to check if there is data to read
            rlist, _, _ = select.select([self.process.stdout], [], [], 0.1)  # 0.1 second timeout
            if self.process.stdout in rlist:
                output = self.process.stdout.readline()
                if output:
                    self.text_area.insert(tk.END, output)
                    self.text_area.see(tk.END)
    
    def terminate_process_tree(self, pid):
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)
        for child in children:
            child.send_signal(signal.SIGINT)
        parent.send_signal(signal.SIGINT)

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()