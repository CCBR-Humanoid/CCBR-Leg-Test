import time
import threading
import math

from config import MOTOR_IDS, DT

# Local helper (small, self-contained)
def lerp(a, b, t):
    return a + t * (b - a)

# --- tiny Overleap-style force profile (200 ms, sin^2) ---
class ForceProfile:
    def __init__(self, duration_ms=200, impulse=2500.0, fx_max=0.0, fx_prop=0.0):
        self.duration_ms = duration_ms
        self.impulse = impulse
        self.fx_max = fx_max
        self.fx_prop = fx_prop
        self.b = 1.0 / max(1.0, duration_ms)  # ms^-1

    def sample(self, t_ms):
        if t_ms < 0 or t_ms > self.duration_ms:
            return 0.0, 0.0
        a = self.impulse / 100.0  # mirrors Overleap scaling
        s = math.sin(self.b * math.pi * t_ms)
        fy = a * (s * s)          # vertical (down +)
        fx = self.fx_prop * fy
        if abs(fx) > self.fx_max:
            fx = math.copysign(self.fx_max, fx)
        return fx, fy

# Jacobianᵀ torque map for the planar (pitch+knee) pair
def torques_from_foot_force(q_pitch, q_knee, fx, fy):
    # Using l2, l3 as the planar links (adjust if your naming differs)
    from kinematics import l2, l3
    s1, c1 = math.sin(q_pitch), math.cos(q_pitch)
    s12, c12 = math.sin(q_pitch + q_knee), math.cos(q_pitch + q_knee)
    # τ = JᵀF
    tau_hip  = (l2 * s1 + l3 * s12) * fx + (-l2 * c1 - l3 * c12) * fy
    tau_knee = (l3 * s12) * fx + (-l3 * c12) * fy
    return tau_hip, tau_knee

class JumpController:
    """Encapsulates jump-related control logic for the leg GUI.

    This class does not create UI. It expects a `gui` object (the MotorGUI
    instance) and uses the GUI's public helpers:
      - gui.enabled (tk.BooleanVar)
      - gui.mode (tk.StringVar)
      - gui.t1, gui.t2, gui.t3 (tk.DoubleVar set/get)
      - gui._jump_status (tk.StringVar)
      - gui._jump_phase (string or None)
      - gui.jump_* timing vars (jump_hold_s, jump_crouch_s, jump_explode_s)
      - gui._measured_joint_angles()
      - gui.send_torque_only(tau_dict)

    The original constants (scripted joint values and optional tau ff) are
    passed at construction so this module avoids importing `gui_leg` and
    creating circular imports.
    """
    def __init__(self, gui, script_joints=None, explode_tau_ff=0.0):
        self.gui = gui
        if script_joints is None:
            # fallbacks match previous GUI defaults
            script_joints = (0.0, -1.557, -0.746, 2.262, 1.621)
        (self.SCRIPT_JOINT0,
         self.SCRIPT_J1_START, self.SCRIPT_J1_END,
         self.SCRIPT_J2_START, self.SCRIPT_J2_END) = script_joints
        self.EXPLODE_TAU_FF = explode_tau_ff

        self._spring_running = False
        self._force_running = False

    # -------- spring / scripted jump --------
    def start_spring_jump(self):
        if not self.gui.enabled.get():
            self.gui.enabled.set(True)
        if getattr(self, "_spring_running", False):
            return
        self.gui.mode.set("Angles")  # drive joints directly
        self._spring_running = True
        self.gui._jump_status.set("Jump: hold")
        threading.Thread(target=self._run_spring_jump, daemon=True).start()

    def _run_spring_jump(self):
        try:
            j0 = self.SCRIPT_JOINT0
            j1s, j1e = self.SCRIPT_J1_START, self.SCRIPT_J1_END
            j2s, j2e = self.SCRIPT_J2_START, self.SCRIPT_J2_END

            hold_s    = float(self.gui.jump_hold_s.get())
            crouch_s  = float(self.gui.jump_crouch_s.get())
            explode_s = float(self.gui.jump_explode_s.get())

            # 0) ensure current pose is target joint0
            self.gui.t1.set(j0)

            # 1) hold to settle (stiff-ish)
            self.gui._jump_phase = "crouch"
            t0 = time.time()
            while time.time() - t0 < max(0.0, hold_s):
                # keep crouch setpoints during hold (start pose)
                self.gui.t2.set(j1s); self.gui.t3.set(j2s)
                time.sleep(DT)

            # 2) go-to crouch smoothly
            self.gui._jump_status.set("Jump: crouch")
            self.gui._jump_phase = "crouch"
            steps = max(1, int(crouch_s / max(DT, 1e-3)))
            j1_init, j2_init = float(self.gui.t2.get()), float(self.gui.t3.get())
            for k in range(steps):
                a = (k + 1) / steps
                self.gui.t1.set(j0)
                self.gui.t2.set(lerp(j1_init, j1s, a))
                self.gui.t3.set(lerp(j2_init, j2s, a))
                time.sleep(DT)

            time.sleep(0.03)  # tiny settle

            # 3) explode: ramp to extended pose; hybrid PD acts like a spring
            self.gui._jump_status.set("Jump: explode")
            self.gui._jump_phase = "explode"
            steps = max(1, int(explode_s / max(DT, 1e-3)))
            for k in range(steps):
                a = (k + 1) / steps
                self.gui.t1.set(j0)
                self.gui.t2.set(lerp(j1s, j1e, a))
                self.gui.t3.set(lerp(j2s, j2e, a))
                time.sleep(DT)

            # 4) relax (drop to base gains)
            self.gui._jump_phase = None
            self.gui._jump_status.set("Jump: flight/relax")
            time.sleep(0.10)

            self.gui._jump_status.set("Idle")
        finally:
            self._spring_running = False

    # -------- force-profile jump (Overleap-style) --------
    def start_force_jump(self):
        if not self.gui.enabled.get():
            self.gui.enabled.set(True)
        if getattr(self, "_force_running", False):
            return
        self._force_running = True
        self.gui._jump_status.set("Jump: force")
        threading.Thread(target=self._run_force_jump, daemon=True).start()

    def _run_force_jump(self):
        try:
            # optional: ensure we're in a crouch-ish stance first (light PD via Angles mode)
            self.gui.mode.set("Angles")
            self.gui._jump_phase = "crouch"
            self.gui.t1.set(self.SCRIPT_JOINT0)
            self.gui.t2.set(self.SCRIPT_J1_START)
            self.gui.t3.set(self.SCRIPT_J2_START)
            time.sleep(0.20)

            # switch to force push: foot assumed stationary during short burst
            prof = ForceProfile(duration_ms=200, impulse=2500.0, fx_max=0.0, fx_prop=0.0)
            t0 = time.time()
            while True:
                t_ms = (time.time() - t0) * 1000.0
                if t_ms > prof.duration_ms:
                    break

                jt = self.gui._measured_joint_angles()
                if jt is None:
                    break
                # joints: 0=roll, 1=pitch, 2=knee
                q_roll, q_pitch, q_knee = jt

                fx, fy = prof.sample(t_ms)
                tau_pitch, tau_knee = torques_from_foot_force(q_pitch, q_knee, fx, fy)

                self.gui.send_torque_only({
                    1: tau_pitch,
                    2: tau_knee,
                })

                time.sleep(max(0.002, DT))

            # zero torques
            self.gui.send_torque_only({1: 0.0, 2: 0.0})
            self.gui._jump_phase = None
            self.gui._jump_status.set("Idle")
        finally:
            self._force_running = False