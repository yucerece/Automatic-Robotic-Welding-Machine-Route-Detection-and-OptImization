import tkinter as tk
from tkinter import messagebox
import subprocess

# Arayüzü oluştur
class CurveApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Curve Selector")
        
        self.curve_list = []

        self.label = tk.Label(master, text="Choose the curve:")
        self.label.pack(pady=5)

        self.curve_entry = tk.Entry(master)
        self.curve_entry.pack(pady=5)

        self.button_frame = tk.Frame(master)
        self.button_frame.pack(pady=10)

        self.test_button = tk.Button(self.button_frame, text="Test", command=self.test_curve)
        self.test_button.pack(side=tk.LEFT, padx=5)

        self.run_button = tk.Button(self.button_frame, text="Run", command=self.run_curves)
        self.run_button.pack(side=tk.RIGHT, padx=5)

    def test_curve(self):
        curve = self.curve_entry.get()
        if curve:
            self.curve_list.append(curve)
            messagebox.showinfo("Info", f"Curve {curve} added to the list for testing.")
            subprocess.run(['python', 'etape1.py', curve])
        else:
            messagebox.showwarning("Warning", "Please enter a curve value.")

    def run_curves(self):
        if self.curve_list:
            #self.run_code(self.curve_list)
            subprocess.run(['python', 'etape3.py'])
        else:
            messagebox.showwarning("Warning", "No curves to run.")

    def test_code(self, curve):
        # Test işlemlerinizi burada yapabilirsiniz
        print(f"Testing curve {curve}")

    def run_code(self, curve_list):
        # Run işlemlerinizi burada yapabilirsiniz
        print("Running curves:", curve_list)

def center_window(window, width=250, height=120):
    # Ekran boyutlarını al
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    # Pencerenin sol üst köşesinin koordinatlarını hesapla
    x = (screen_width - width) // 2
    y = (screen_height - height) // 2

    # Pencereyi konumlandır ve boyutlandır
    window.geometry(f'{width}x{height}+{x}+{y}')

if __name__ == "__main__":
    root = tk.Tk()
    app = CurveApp(root)
    center_window(root)  # Pencereyi ortala
    root.mainloop()
    print(app.curve_list)