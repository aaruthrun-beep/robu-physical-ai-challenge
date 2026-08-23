"""
5-Bar Parallel Robot — Digital Twin desktop app (Tkinter, Windows-friendly).

Run:  python app.py
The collision checker guards every move for both the simulator and real hardware.
"""
import math
import time
import tkinter as tk
from tkinter import ttk, messagebox

from fivebar.config import default_config
from fivebar import kinematics as kin
from fivebar import workspace as ws
from fivebar import collision as col
from fivebar.backends import SimBackend, CanBackend
from fivebar.command import CommandManager, Rejection

BG="#0d1216"; PANEL="#141c22"; INK="#e6f0f2"; DIM="#7f929b"; LINE="#26333c"
ACC="#35e0c8"; BAD="#ff5a5f"; GOAL="#ffd166"; AMBER="#f5b34a"


class App:
    def __init__(self, root):
        self.root = root
        root.title("5-Bar Parallel Robot — Digital Twin")
        root.configure(bg=BG)
        root.geometry("1180x760")

        self.cfg = default_config()
        start = kin.ik(self.cfg, 0, 220, +1) or (math.pi/2, math.pi/2)
        self.backend = SimBackend(self.cfg, start=start)
        self.cmd = CommandManager(self.cfg, self.backend, assembly=+1)
        self.goal_xy = (0.0, 220.0)
        self.goal_valid = True
        self.live_confirmed = False
        self.ws_image = None
        self._last = time.time()

        self._build_layout()
        self._recompute_workspace()
        self._loop()

    # ---------------- layout ----------------
    def _build_layout(self):
        left = tk.Frame(self.root, bg=PANEL, width=270); left.pack(side="left", fill="y")
        left.pack_propagate(False)
        self.canvas = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_click)
        self.canvas.bind("<Configure>", lambda e: self._recompute_workspace())
        right = tk.Frame(self.root, bg=PANEL, width=250); right.pack(side="left", fill="y")
        right.pack_propagate(False)
        self._build_left(left); self._build_right(right)

    def _hdr(self, parent, text):
        tk.Label(parent, text=text, bg=PANEL, fg=DIM, font=("Segoe UI",8,"bold")).pack(anchor="w", pady=(12,4), padx=12)

    def _entry(self, parent, label, val):
        row=tk.Frame(parent,bg=PANEL); row.pack(fill="x", padx=12, pady=2)
        tk.Label(row,text=label,bg=PANEL,fg=DIM,font=("Segoe UI",9),width=14,anchor="w").pack(side="left")
        v=tk.StringVar(value=str(val))
        tk.Entry(row,textvariable=v,width=8,bg=BG,fg=INK,insertbackground=INK,
                 relief="flat",highlightthickness=1,highlightbackground=LINE).pack(side="left")
        return v

    def _build_left(self, p):
        self._hdr(p,"GEOMETRY (mm)")
        self.g_L1a=self._entry(p,"Prox L (L1a)",self.cfg.L1a)
        self.g_L1b=self._entry(p,"Prox R (L1b)",self.cfg.L1b)
        self.g_L2a=self._entry(p,"Dist L (L2a)",self.cfg.L2a)
        self.g_L2b=self._entry(p,"Dist R (L2b)",self.cfg.L2b)
        self.g_d  =self._entry(p,"Base sep (d)",self.cfg.d)
        self.g_dh =self._entry(p,"Height off (dh)",self.cfg.dh)
        self.g_margin=self._entry(p,"Margin",self.cfg.margin)
        self.g_speed=self._entry(p,"Max Vel (rad/s)",self.cfg.max_vel)
        self.g_accel=self._entry(p,"Max Acc (rad/s²)",self.cfg.max_acc)
        tk.Button(p,text="Apply parameters",command=self._apply_geom,bg="#1a242c",fg=INK,
                  relief="flat",activebackground=LINE).pack(fill="x",padx=12,pady=(6,2))

        self._hdr(p,"JOG (deg)")
        row=tk.Frame(p,bg=PANEL); row.pack(fill="x",padx=12)
        tk.Label(row,text="step",bg=PANEL,fg=DIM,font=("Segoe UI",9)).pack(side="left")
        self.jog_step=tk.StringVar(value="5")
        tk.Entry(row,textvariable=self.jog_step,width=6,bg=BG,fg=INK,relief="flat",
                 highlightthickness=1,highlightbackground=LINE).pack(side="left",padx=6)
        for (lbl,m,sgn) in [("θ1 −",0,-1),("θ1 +",0,+1),("θ2 −",1,-1),("θ2 +",1,+1)]:
            tk.Button(p,text=lbl,command=lambda mm=m,ss=sgn:self._jog(mm,ss),bg="#1a242c",
                      fg=INK,relief="flat",activebackground=LINE).pack(fill="x",padx=12,pady=1)

        self._hdr(p,"TARGET (mm)")
        self.t_x=self._entry(p,"X",0); self.t_y=self._entry(p,"Y",220)
        tk.Button(p,text="Validate",command=self._validate_target,bg="#1a242c",fg=INK,
                  relief="flat",activebackground=LINE).pack(fill="x",padx=12,pady=(6,2))
        self.move_btn=tk.Button(p,text="MOVE / SEND",command=self._move,bg=ACC,fg="#062018",
                                relief="flat",font=("Segoe UI",10,"bold"))
        self.move_btn.pack(fill="x",padx=12,pady=2)

        self._hdr(p,"MODE")
        self.mode_var=tk.StringVar(value="SIM")
        row=tk.Frame(p,bg=PANEL); row.pack(fill="x",padx=12)
        ttk.Radiobutton(row,text="Simulation",variable=self.mode_var,value="SIM",command=self._set_mode).pack(anchor="w")
        ttk.Radiobutton(row,text="Live (CAN)",variable=self.mode_var,value="LIVE",command=self._set_mode).pack(anchor="w")
        ttk.Radiobutton(row,text="Monitor (Read-Only)",variable=self.mode_var,value="MONITOR",command=self._set_mode).pack(anchor="w")
        tk.Button(p,text="■  E-STOP",command=self._estop,bg=BAD,fg="white",
                  relief="flat",font=("Segoe UI",10,"bold")).pack(fill="x",padx=12,pady=(10,4))

    def _build_right(self, p):
        self._hdr(p,"STATUS")
        self.status={}
        for key,label in [("mode","Mode"),("conn","Connection"),("t1","θ left"),
                          ("t2","θ right"),("x","EE X"),("y","EE Y"),
                          ("margin","Collision margin"),("check","Last check")]:
            row=tk.Frame(p,bg=PANEL); row.pack(fill="x",padx=12,pady=3)
            tk.Label(row,text=label,bg=PANEL,fg=DIM,font=("Consolas",9),width=15,anchor="w").pack(side="left")
            v=tk.Label(row,text="—",bg=PANEL,fg=INK,font=("Consolas",9,"bold"),anchor="e")
            v.pack(side="right"); self.status[key]=v
        self.reach_lbl=tk.Label(p,text="REACHABLE",bg=PANEL,fg=ACC,font=("Consolas",11,"bold"))
        self.reach_lbl.pack(pady=8)
        self._hdr(p,"REJECTED COMMANDS")
        self.log_box=tk.Listbox(p,bg=BG,fg=BAD,font=("Consolas",8),relief="flat",
                                highlightthickness=1,highlightbackground=LINE,height=14)
        self.log_box.pack(fill="both",expand=True,padx=12,pady=(0,12))

    # ---------------- coordinate transform ----------------
    def _view(self):
        x0,x1,y0,y1 = ws.bounds(self.cfg)
        W=max(50,self.canvas.winfo_width()); H=max(50,self.canvas.winfo_height())
        sc=min(W/(x1-x0),H/(y1-y0))*0.92
        ox=W/2-(x0+x1)/2*sc; oy=H/2+(y0+y1)/2*sc
        return sc,ox,oy
    def w2s(self,x,y):
        sc,ox,oy=self._view(); return ox+x*sc, oy-y*sc
    def s2w(self,px,py):
        sc,ox,oy=self._view(); return (px-ox)/sc, (oy-py)/sc

    # ---------------- workspace bitmap ----------------
    def _recompute_workspace(self):
        W=max(50,self.canvas.winfo_width()); H=max(50,self.canvas.winfo_height())
        if W<60 or H<60: return
        block=6
        nx,ny=W//block, H//block
        img=tk.PhotoImage(width=nx, height=ny)
        colmap={ws.SAFE:"#12403a", ws.FORBIDDEN:"#5a1d20", ws.UNREACHABLE:"#0d1216"}
        for jy in range(ny):
            row=[]
            py=jy*block+block/2
            for jx in range(nx):
                px=jx*block+block/2
                x,y=self.s2w(px,py)
                row.append(colmap[ws.classify(self.cfg,x,y)[0]])
            img.put("{"+" ".join(row)+"}", to=(0,jy))
        self.ws_image=img.zoom(block)   # nearest-neighbour scale-up to canvas size
        self._validate_target(silent=True)

    # ---------------- actions ----------------
    def _on_click(self, e):
        """Click / drag on the canvas -> set that point as the target."""
        x, y = self.s2w(e.x, e.y)
        self.t_x.set(f"{x:.0f}"); self.t_y.set(f"{y:.0f}")
        self._validate_target()

    def _apply_geom(self):
        try:
            self.cfg.L1a=float(self.g_L1a.get()); self.cfg.L1b=float(self.g_L1b.get())
            self.cfg.L2a=float(self.g_L2a.get()); self.cfg.L2b=float(self.g_L2b.get())
            self.cfg.d=float(self.g_d.get());     self.cfg.dh=float(self.g_dh.get())
            self.cfg.margin=float(self.g_margin.get())
            self.cfg.max_vel=float(self.g_speed.get())
            self.cfg.max_acc=float(self.g_accel.get())
        except ValueError:
            messagebox.showerror("Bad value","All parameters must be numbers."); return
        self._recompute_workspace()

    def _jog(self, motor, sgn):
        try: step=math.radians(float(self.jog_step.get()))
        except ValueError: return
        try:
            self.cmd.jog(motor, sgn*step)
        except Rejection as e:
            self._log(f"jog blocked: {e}")

    def _validate_target(self, silent=False):
        try:
            x=float(self.t_x.get()); y=float(self.t_y.get())
        except ValueError:
            return
        self.goal_xy=(x,y)
        try:
            self.cmd.validate_target(x,y)
            self.goal_valid=True
            self.move_btn.config(state="normal",bg=ACC)
            self.status["check"].config(text="OK",fg=ACC)
        except Rejection as e:
            self.goal_valid=False
            self.move_btn.config(state="disabled",bg="#2a3138")
            self.status["check"].config(text="REJECT",fg=BAD)
            if not silent: self._log(f"target ({x:.0f},{y:.0f}): {e}")

    def _move(self):
        x,y=self.goal_xy
        if self.mode_var.get()=="LIVE" and not self.live_confirmed:
            if not messagebox.askyesno("Live hardware",
                "Send this command to the REAL robot over CAN?"):
                return
            self.live_confirmed=True
        try:
            self.cmd.move_to(x,y)
        except Rejection as e:
            self._log(f"move blocked: {e}")

    def _set_mode(self):
        mode=self.mode_var.get()
        if hasattr(self, "backend") and hasattr(self.backend, "close"):
            try:
                self.backend.close()
            except Exception:
                pass
        if mode in ("LIVE", "MONITOR"):
            try:
                is_mon = (mode == "MONITOR")
                be = CanBackend(self.cfg, is_monitor=is_mon)
                be.connect()
                self.backend = be
                self.cmd.backend = be
                self.live_confirmed = False
                if is_mon:
                    self._log("Switched to MONITOR ONLY mode (Passive encoder reading, motors unpowered)")
                    self.move_btn.config(state="disabled", text="MONITOR ONLY (Read-Only)")
                else:
                    self._log("Switched to LIVE mode (RobStride CAN)")
                    self.move_btn.config(state="normal", text="MOVE / SEND")
            except Exception as e:
                messagebox.showerror("CAN connect failed",
                    f"{e}\n\nStaying in simulation. Check adapter / config.")
                self.mode_var.set("SIM")
                self.backend = SimBackend(self.cfg)
                self.cmd.backend = self.backend
                self.move_btn.config(state="normal", text="MOVE / SEND")
        else:
            self.backend = SimBackend(self.cfg, start=self.backend.read_angles() if hasattr(self, "backend") else (math.pi/2, math.pi/2))
            self.cmd.backend = self.backend
            self._log("Switched to SIM mode")
            self.move_btn.config(state="normal", text="MOVE / SEND")

    def _estop(self):
        cur=self.backend.read_angles()
        self.backend.send_angles(*cur)          # freeze at current
        if hasattr(self.backend,"target"): self.backend.target=list(cur)
        self._log("E-STOP pressed")

    def _log(self, msg):
        self.log_box.insert(0, time.strftime("%H:%M:%S ")+msg)

    # ---------------- render loop ----------------
    def _loop(self):
        now=time.time(); dt=min(0.05, now-self._last); self._last=now
        if hasattr(self.backend, "step"): self.backend.step(dt)
        self._draw()
        self._update_status()
        self.root.after(30, self._loop)

    def _draw(self):
        c=self.canvas; c.delete("robot")
        if self.ws_image is not None:
            c.delete("ws"); c.create_image(0,0,anchor="nw",image=self.ws_image,tags="ws")
        # goal marker
        gx,gy=self.w2s(*self.goal_xy)
        gc=GOAL if self.goal_valid else BAD
        c.create_line(gx-9,gy,gx+9,gy,fill=gc,tags="robot")
        c.create_line(gx,gy-9,gx,gy+9,fill=gc,tags="robot")
        # current pose
        t1,t2=self.backend.read_angles()
        ee=kin.fk(self.cfg,t1,t2,self.cmd.assembly)
        (A1x,A1y),(A2x,A2y)=self.cfg.bases()
        a1=self.w2s(A1x,A1y); a2=self.w2s(A2x,A2y)
        c.create_line(*a1,*a2,fill="#2f3f48",width=3,tags="robot")
        if ee is not None:
            C1,C2=kin.elbows(self.cfg,t1,t2)
            c1=self.w2s(*C1); c2=self.w2s(*C2); p=self.w2s(*ee)
            c.create_line(*a1,*c1,fill="#dbeff1",width=5,tags="robot")
            c.create_line(*a2,*c2,fill="#dbeff1",width=5,tags="robot")
            c.create_line(*c1,*p,fill=ACC,width=4,tags="robot")
            c.create_line(*c2,*p,fill=ACC,width=4,tags="robot")
            for (cx,cy) in (c1,c2):
                c.create_oval(cx-4,cy-4,cx+4,cy+4,fill=PANEL,outline="#8fa4ad",tags="robot")
            c.create_oval(p[0]-6,p[1]-6,p[0]+6,p[1]+6,fill=ACC,outline="#0b0f13",width=2,tags="robot")
        for (ax,ay) in (a1,a2):
            c.create_oval(ax-6,ay-6,ax+6,ay+6,fill="#0b0f13",outline=ACC,width=2,tags="robot")

    def _update_status(self):
        t1,t2=self.backend.read_angles()
        ee=kin.fk(self.cfg,t1,t2,self.cmd.assembly)
        rep=col.check_angles(self.cfg,t1,t2,self.cmd.assembly)
        self.status["mode"].config(text=self.mode_var.get())
        conn="connected" if getattr(self.backend,"connected",False) else "—"
        self.status["conn"].config(text=conn)
        self.status["t1"].config(text=f"{math.degrees(t1):.1f}°")
        self.status["t2"].config(text=f"{math.degrees(t2):.1f}°")
        if ee:
            self.status["x"].config(text=f"{ee[0]:.1f}")
            self.status["y"].config(text=f"{ee[1]:.1f}")
        m=rep.min_clearance
        self.status["margin"].config(text=f"{m:.1f} mm",
                                     fg=ACC if rep.ok else BAD)
        self.status["check"].config(text="OK" if rep.ok else "COLLISION",
                                    fg=ACC if rep.ok else BAD)
        if self.goal_valid:
            self.reach_lbl.config(text="REACHABLE",fg=ACC)
        else:
            self.reach_lbl.config(text="NOT REACHABLE",fg=BAD)
        # refresh rejection log
        while len(self.cmd.log):
            ts,what,why=self.cmd.log.pop(0)
            self.log_box.insert(0,f"{ts} {what}: {why}")


if __name__ == "__main__":
    root = tk.Tk()
    try:
        style = ttk.Style(); style.theme_use("clam")
        style.configure("TRadiobutton", background=PANEL, foreground=INK)
    except Exception:
        pass
    app = App(root)
    
    def on_closing():
        if hasattr(app, "backend") and hasattr(app.backend, "close"):
            try:
                app.backend.close()
            except Exception:
                pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
