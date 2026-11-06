"""Run on laptop: python lidar_viewer.py --remote 10.42.0.1
Optional local Pi debug mode: python lidar_viewer.py --local
"""
import argparse, socket, threading, time, queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from lidar_net import PORT, send_packet, recv_packet

class Viewer:
    def __init__(self, root, host):
        self.root, self.host = root, host
        self.points, self.q = [], queue.Queue()
        self.running = True
        self.lifetime = tk.StringVar(value="30")
        self.step = tk.StringVar(value="2")
        self.status = tk.StringVar(value="Connecting...")
        self.rate = tk.StringVar(value="Estimated: -- points/min")
        self.sock = socket.create_connection((host, PORT), timeout=8)
        self.sock.settimeout(None)
        self.rx = threading.Thread(target=self.receive, daemon=True); self.rx.start()
        root.title("M1C1 3D LiDAR Viewer")
        root.geometry("1200x850")
        bar=ttk.Frame(root,padding=8); bar.pack(fill=tk.X)
        ttk.Button(bar,text="Start",command=lambda:self.send({"type":"start"})).pack(side=tk.LEFT,padx=3)
        ttk.Button(bar,text="Stop",command=lambda:self.send({"type":"stop"})).pack(side=tk.LEFT,padx=3)
        ttk.Label(bar,text="Servo step (°)").pack(side=tk.LEFT,padx=(12,3))
        ttk.Entry(bar,textvariable=self.step,width=7).pack(side=tk.LEFT)
        ttk.Label(bar,text="Point lifetime (s)").pack(side=tk.LEFT,padx=(10,3))
        ttk.Entry(bar,textvariable=self.lifetime,width=7).pack(side=tk.LEFT)
        ttk.Button(bar,text="Apply",command=self.apply).pack(side=tk.LEFT,padx=4)
        ttk.Button(bar,text="Export PLY",command=self.export_ply).pack(side=tk.RIGHT,padx=4)
        self.fig=plt.Figure(figsize=(9,7),dpi=100)
        self.ax=self.fig.add_subplot(111,projection="3d")
        self.ax.set_xlabel("X (mm)"); self.ax.set_ylabel("Y (mm)"); self.ax.set_zlabel("Z (mm)")
        self.ax.set_title("M1C1 3D Point Cloud")
        self.ax.set_box_aspect((1,1,1))
        self.scatter=self.ax.scatter([],[],[],s=1)
        FigureCanvasTkAgg
        self.canvas=FigureCanvasTkAgg(self.fig,master=root); self.canvas.get_tk_widget().pack(fill=tk.BOTH,expand=True)
        foot=ttk.Frame(root,padding=6); foot.pack(fill=tk.X)
        ttk.Label(foot,textvariable=self.status).pack(side=tk.LEFT)
        ttk.Label(foot,textvariable=self.rate).pack(side=tk.RIGHT)
        root.protocol("WM_DELETE_WINDOW",self.close)
        root.after(100,self.update)

    def send(self,p):
        try: send_packet(self.sock,p)
        except OSError as e: self.q.put(("error",str(e)))

    def receive(self):
        buf=bytearray()
        try:
            while self.running:
                p=recv_packet(self.sock,buf)
                if p is None: break
                self.q.put(("packet",p))
        except Exception as e: self.q.put(("error",str(e)))

    def apply(self):
        try:
            step=float(self.step.get()); life=float(self.lifetime.get())
            if not 0<step<=180 or life<0: raise ValueError
            self.send({"type":"settings","angle_step":step,"point_lifetime":life})
        except ValueError: messagebox.showerror("Invalid settings","Step must be >0 and ≤180; lifetime must be ≥0.")

    def update(self):
        if not self.running: return
        while not self.q.empty():
            kind,p=self.q.get()
            if kind=="error": self.status.set("Network error: "+p); continue
            typ=p.get("type")
            if typ=="points":
                now=time.monotonic()
                self.points.extend((x,y,z,now) for x,y,z in p["points"])
                self.rate.set(f"Estimated: {p.get('points_per_min_est',0):,.0f} points/min")
            elif typ=="status": self.status.set(p.get("status",""))
            elif typ=="error": self.status.set("Scanner error: "+p.get("message",""))
        try: life=max(0,float(self.lifetime.get()))
        except ValueError: life=30
        now=time.monotonic()
        self.points=[p for p in self.points if life>0 and now-p[3]<=life]
        if self.points:
            self.scatter._offsets3d=([p[0] for p in self.points],[p[1] for p in self.points],[p[2] for p in self.points])
        else: self.scatter._offsets3d=([],[],[])
        self.status.set(self.status.get().split(" | ")[0]+f" | Points: {len(self.points):,}")
        self.canvas.draw_idle()
        self.root.after(150,self.update)

    def export_ply(self):
        pts=[p[:3] for p in self.points]
        if not pts: messagebox.showinfo("Export","No visible points."); return
        name=filedialog.asksaveasfilename(defaultextension=".ply",filetypes=[("PLY point cloud","*.ply")],initialfile="m1c1_pointcloud.ply")
        if not name:return
        with open(name,"w",encoding="ascii") as f:
            f.write("ply\\nformat ascii 1.0\\n")
            f.write(f"element vertex {len(pts)}\\nproperty float x\\nproperty float y\\nproperty float z\\nend_header\\n")
            for x,y,z in pts:f.write(f"{x:.4f} {y:.4f} {z:.4f}\\n")
        messagebox.showinfo("Export",f"Exported {len(pts):,} points.")

    def close(self):
        self.running=False
        try:self.sock.close()
        except OSError:pass
        self.root.destroy()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--remote",help="Raspberry Pi IP address")
    ap.add_argument("--local",action="store_true",help="Local debug mode (connect to localhost service)")
    a=ap.parse_args()
    host="127.0.0.1" if a.local else a.remote
    if not host: ap.error("Use --remote PI_IP or --local")
    root=tk.Tk(); Viewer(root,host); root.mainloop()

if __name__=="__main__":main()
