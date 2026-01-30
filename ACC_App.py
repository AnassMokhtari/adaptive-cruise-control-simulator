# main.py
# ============================================================
# Premium ACC (Adaptive Cruise Control) Simulator (Single-file)
# PySide6 / Qt - 3 Views: Dashboard / CAN Monitor / Scenario Explorer
# ECU separation via Virtual CAN bus (DriverInputs, Radar, ACC, Engine, Brake, Cluster)
# ============================================================

import sys
import sys
import math
import time
import random
import struct
from dataclasses import dataclass
from enum import IntEnum
from collections import deque
from typing import Dict, Optional, List, Tuple, Callable

from PySide6.QtCore import (
    Qt, QTimer, QObject, Signal, QSize, QElapsedTimer, QRectF, QPointF, QPropertyAnimation, QEasingCurve
)
from PySide6.QtGui import (
    QColor, QPainter, QPen, QBrush, QFont, QPainterPath, QLinearGradient, QRadialGradient
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QStackedWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QLabel, QPushButton, QFrame, QComboBox, QLineEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QSpinBox, QDoubleSpinBox, QCheckBox, QSlider, QSplitter, QTextEdit,
    QTreeWidget, QTreeWidgetItem, QGroupBox, QTabWidget, QMessageBox, QGraphicsOpacityEffect,
    QSizePolicy
)
from PySide6.QtWidgets import QGraphicsDropShadowEffect


# ============================================================
# --- Constants & Helpers
# ============================================================

G = 9.81
ACC_DECEL_AUTHORITY = 0.2 * G  # 0.2g (m/s^2) -> 1.962
ACC_MAX_ACCEL = 1.8           # comfort accel
SIM_FIXED_DT = 0.02           # 50 Hz fixed-step physics/control
UI_FRAME_MS = 16              # ~60 FPS render loop
CAN_MONITOR_MS = 100          # 10 Hz for CAN table refresh
SCENARIO_TICK_MS = 50         # scenario scheduler tick
EGO_LENGTH_M = 4.6
LEAD_LENGTH_M = 4.6

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def kmh_to_mps(kmh): return kmh / 3.6
def mps_to_kmh(mps): return mps * 3.6

class Smooth1P:
    """1st-order low-pass filter: y += alpha*(x-y), alpha = dt/(tau+dt)."""
    def __init__(self, tau: float, y0: float = 0.0):
        self.tau = max(1e-6, tau)
        self.y = y0

    def step(self, x: float, dt: float) -> float:
        a = dt / (self.tau + dt)
        self.y += a * (x - self.y)
        return self.y


# ============================================================
# --- CAN: Frames, Codec, Bus
# ============================================================

@dataclass
class CANFrame:
    seq: int
    ts: float
    arb_id: int
    data: bytes
    src: str = ""
    name: str = ""

def _u16(x): return struct.pack("<H", int(clamp(x, 0, 65535)))
def _i16(x): return struct.pack("<h", int(clamp(x, -32768, 32767)))
def _u32(x): return struct.pack("<I", int(clamp(x, 0, 4294967295)))

def _ru16(b, o): return struct.unpack_from("<H", b, o)[0]
def _ri16(b, o): return struct.unpack_from("<h", b, o)[0]
def _ru32(b, o): return struct.unpack_from("<I", b, o)[0]

class AccState(IntEnum):
    ACC_OFF = 0
    ACC_STANDBY = 1
    ACC_ACTIVE_SPEED_CONTROL = 2
    ACC_ACTIVE_TIME_GAP_CONTROL = 3
    DRIVER_INTERVENTION_REQUIRED = 4

class SimType(IntEnum):
    FREE_DRIVE = 0
    CONSTANT_LEAD_FOLLOW = 1
    STOP_AND_GO = 2
    CUT_IN = 3
    LEAD_HARD_BRAKE = 4
    TRAFFIC_JAM = 5

# Arbitration IDs (simple, consistent, 8-byte payloads)
ID_DRIVER_INPUTS = 0x101
ID_LEAD_CTRL     = 0x102
ID_EGO_STATE     = 0x110
ID_LEAD_STATE    = 0x111
ID_RADAR_OBJECT  = 0x120
ID_ACC_STATUS    = 0x130
ID_ENGINE_CMD    = 0x140
ID_BRAKE_CMD     = 0x150
ID_CLUSTER       = 0x160

BTN_ON        = 1 << 0
BTN_OFF       = 1 << 1
BTN_SET_PLUS  = 1 << 2
BTN_COAST     = 1 << 3
BTN_RESUME    = 1 << 4
BTN_TG_PLUS   = 1 << 5
BTN_TG_MINUS  = 1 << 6

BS1_BIT = 1 << 0
BS2_BIT = 1 << 1

# Message text codes (short & deterministic)
MSG_NONE = 0
MSG_ACTIVATION_DENIED = 1
MSG_ACTIVATED = 2
MSG_CANCEL_BRAKE = 3
MSG_CANCEL_SPEED_LOW = 4
MSG_CANCEL_OFF = 5
MSG_LEAD_DETECTED = 6
MSG_LEAD_LOST = 7
MSG_INTERVENTION_REQUIRED = 8
MSG_RESUMED = 9
MSG_SET_SPEED_UPDATED = 10
MSG_TIMEGAP_UPDATED = 11

MSG_TEXT = {
    MSG_NONE: "",
    MSG_ACTIVATION_DENIED: "Activation denied: need ego ≥ 40 km/h and BS1=0 & BS2=0",
    MSG_ACTIVATED: "ACC activated",
    MSG_CANCEL_BRAKE: "ACC cancelled by brake (BS1/BS2)",
    MSG_CANCEL_SPEED_LOW: "ACC cancelled: speed < 25 km/h",
    MSG_CANCEL_OFF: "ACC OFF",
    MSG_LEAD_DETECTED: "Lead detected: switching to time-gap control",
    MSG_LEAD_LOST: "Lead lost: switching to speed control",
    MSG_INTERVENTION_REQUIRED: "Driver intervention required: decel demand > 0.2g",
    MSG_RESUMED: "Resume: returning to active control",
    MSG_SET_SPEED_UPDATED: "Set speed updated",
    MSG_TIMEGAP_UPDATED: "Time gap updated",
}

class CANCodec:
    @staticmethod
    def enc_driver_inputs(btn_flags: int, bs_flags: int, driver_throttle_pct: int, driver_brake_pct: int, counter: int) -> bytes:
        b0 = btn_flags & 0xFF
        b1 = (btn_flags >> 8) & 0xFF
        b2 = bs_flags & 0xFF
        b3 = int(clamp(driver_throttle_pct, 0, 100))
        b4 = int(clamp(driver_brake_pct, 0, 100))
        b5 = 0
        b6 = 0
        b7 = counter & 0xFF
        return bytes([b0, b1, b2, b3, b4, b5, b6, b7])

    @staticmethod
    def dec_driver_inputs(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        btn_flags = data[0] | (data[1] << 8)
        bs_flags  = data[2]
        thr = data[3]
        brk = data[4]
        ctr = data[7]
        return {"btn": btn_flags, "bs": bs_flags, "thr": thr, "brk": brk, "ctr": ctr}

    @staticmethod
    def enc_lead_ctrl(sim_type: int, present: int, lead_speed_kmh: float, lead_distance_m: float) -> bytes:
        # [u8 sim_type][u8 present][u16 spd*10][u16 dist*10][u16 reserved]
        spd = int(clamp(lead_speed_kmh * 10.0, 0, 65535))
        dist = int(clamp(lead_distance_m * 10.0, 0, 65535))
        return struct.pack("<BBHHH", int(sim_type) & 0xFF, int(present) & 0xFF, spd, dist, 0)

    @staticmethod
    def dec_lead_ctrl(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        sim_type, present, spd_x10, dist_x10, _ = struct.unpack("<BBHHH", data[:8])
        return {"sim_type": sim_type, "present": bool(present), "lead_speed_kmh": spd_x10 / 10.0, "lead_distance_m": dist_x10 / 10.0}

    @staticmethod
    def enc_ego_state(speed_kmh: float, accel_mps2: float, pos_m: float) -> bytes:
        # [u16 spd*10][i16 acc*100][u32 pos*10]
        spd = int(clamp(speed_kmh * 10.0, 0, 65535))
        acc = int(clamp(accel_mps2 * 100.0, -32768, 32767))
        pos = int(clamp(pos_m * 10.0, 0, 4294967295))
        return struct.pack("<HhI", spd, acc, pos)

    @staticmethod
    def dec_ego_state(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        spd_x10, acc_x100, pos_x10 = struct.unpack("<HhI", data[:8])
        return {"speed_kmh": spd_x10 / 10.0, "accel_mps2": acc_x100 / 100.0, "pos_m": pos_x10 / 10.0}

    @staticmethod
    def enc_lead_state(present: bool, speed_kmh: float, pos_m: float) -> bytes:
        # [u8 present][u8 res][u16 spd*10][u32 pos*10]
        spd = int(clamp(speed_kmh * 10.0, 0, 65535))
        pos = int(clamp(pos_m * 10.0, 0, 4294967295))
        return struct.pack("<BBHI", 1 if present else 0, 0, spd, pos)

    @staticmethod
    def dec_lead_state(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        present, _, spd_x10, pos_x10 = struct.unpack("<BBHI", data[:8])
        return {"present": bool(present), "speed_kmh": spd_x10 / 10.0, "pos_m": pos_x10 / 10.0}

    @staticmethod
    def enc_radar_object(present: bool, dist_m: float, rel_speed_mps: float, lead_speed_kmh: float) -> bytes:
        # [u8 present][u16 dist*10][i16 relspd*10][u16 leadspd*10][u8 res]
        dist = int(clamp(dist_m * 10.0, 0, 65535))
        rel  = int(clamp(rel_speed_mps * 10.0, -32768, 32767))
        lspd = int(clamp(lead_speed_kmh * 10.0, 0, 65535))
        return struct.pack("<BHhHB", 1 if present else 0, dist, rel, lspd, 0)

    @staticmethod
    def dec_radar_object(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        present, dist_x10, rel_x10, lspd_x10, _ = struct.unpack("<BHhHB", data[:8])
        return {"present": bool(present), "dist_m": dist_x10 / 10.0, "rel_speed_mps": rel_x10 / 10.0, "lead_speed_kmh": lspd_x10 / 10.0}

    @staticmethod
    def enc_acc_status(state: int, msg_code: int, set_speed_kmh: float, time_gap_s: float, acc_cmd_mps2: float) -> bytes:
        # [u8 state][u8 msg][u16 setspd*10][u16 tg*10][i16 acc*100]
        spd = int(clamp(set_speed_kmh * 10.0, 0, 65535))
        tg  = int(clamp(time_gap_s * 10.0, 0, 65535))
        ac  = int(clamp(acc_cmd_mps2 * 100.0, -32768, 32767))
        return struct.pack("<BBHHh", int(state) & 0xFF, int(msg_code) & 0xFF, spd, tg, ac)

    @staticmethod
    def dec_acc_status(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        st, msg, spd_x10, tg_x10, acc_x100 = struct.unpack("<BBHHh", data[:8])
        return {"state": st, "msg": msg, "set_speed_kmh": spd_x10 / 10.0, "time_gap_s": tg_x10 / 10.0, "acc_cmd_mps2": acc_x100 / 100.0}

    @staticmethod
    def enc_engine_cmd(throttle_pct: float, acc_cmd_mps2: float) -> bytes:
        # [u8 thr][u8 res][i16 acc*100][u32 res]
        thr = int(clamp(throttle_pct, 0, 100))
        ac  = int(clamp(acc_cmd_mps2 * 100.0, -32768, 32767))
        return struct.pack("<BBhI", thr, 0, ac, 0)

    @staticmethod
    def dec_engine_cmd(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        thr, _, acc_x100, _ = struct.unpack("<BBhI", data[:8])
        return {"throttle_pct": thr, "acc_cmd_mps2": acc_x100 / 100.0}

    @staticmethod
    def enc_brake_cmd(brake_pct: float, brake_lights: bool, decel_cmd_mps2: float) -> bytes:
        # [u8 brk][u8 lights][i16 dec*100][u32 res]
        brk = int(clamp(brake_pct, 0, 100))
        lights = 1 if brake_lights else 0
        dc = int(clamp(decel_cmd_mps2 * 100.0, -32768, 32767))
        return struct.pack("<BBhI", brk, lights, dc, 0)

    @staticmethod
    def dec_brake_cmd(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        brk, lights, dec_x100, _ = struct.unpack("<BBhI", data[:8])
        return {"brake_pct": brk, "brake_lights": bool(lights), "decel_cmd_mps2": dec_x100 / 100.0}

    @staticmethod
    def enc_cluster(speed_kmh: float, dist_m: float, rel_kmh: float, state: int, lights: bool) -> bytes:
        # [u16 spd*10][u16 dist*10][i16 rel*10][u8 state][u8 lights]
        spd = int(clamp(speed_kmh * 10.0, 0, 65535))
        dist = int(clamp(dist_m * 10.0, 0, 65535))
        rel  = int(clamp(rel_kmh * 10.0, -32768, 32767))
        return struct.pack("<HHhBB", spd, dist, rel, int(state) & 0xFF, 1 if lights else 0)

    @staticmethod
    def dec_cluster(data: bytes):
        if len(data) < 8: data = data.ljust(8, b"\x00")
        spd_x10, dist_x10, rel_x10, st, lights = struct.unpack("<HHhBB", data[:8])
        return {"speed_kmh": spd_x10 / 10.0, "dist_m": dist_x10 / 10.0, "rel_kmh": rel_x10 / 10.0, "state": st, "brake_lights": bool(lights)}

MSG_NAMES = {
    ID_DRIVER_INPUTS: "DRIVER_INPUTS",
    ID_LEAD_CTRL:     "LEAD_CTRL",
    ID_EGO_STATE:     "EGO_STATE",
    ID_LEAD_STATE:    "LEAD_STATE",
    ID_RADAR_OBJECT:  "RADAR_OBJECT",
    ID_ACC_STATUS:    "ACC_STATUS",
    ID_ENGINE_CMD:    "ENGINE_CMD",
    ID_BRAKE_CMD:     "BRAKE_CMD",
    ID_CLUSTER:       "CLUSTER",
}

def decode_frame(fid: int, data: bytes) -> Dict:
    try:
        if fid == ID_DRIVER_INPUTS: return CANCodec.dec_driver_inputs(data)
        if fid == ID_LEAD_CTRL:     return CANCodec.dec_lead_ctrl(data)
        if fid == ID_EGO_STATE:     return CANCodec.dec_ego_state(data)
        if fid == ID_LEAD_STATE:    return CANCodec.dec_lead_state(data)
        if fid == ID_RADAR_OBJECT:  return CANCodec.dec_radar_object(data)
        if fid == ID_ACC_STATUS:    return CANCodec.dec_acc_status(data)
        if fid == ID_ENGINE_CMD:    return CANCodec.dec_engine_cmd(data)
        if fid == ID_BRAKE_CMD:     return CANCodec.dec_brake_cmd(data)
        if fid == ID_CLUSTER:       return CANCodec.dec_cluster(data)
    except Exception:
        return {}
    return {}

class VirtualCANBus:
    def __init__(self, max_log: int = 5000):
        self._seq = 0
        self._log = deque(maxlen=max_log)  # CANFrame
        self._last: Dict[int, CANFrame] = {}

    def send(self, arb_id: int, data: bytes, src: str = "") -> CANFrame:
        self._seq += 1
        fr = CANFrame(
            seq=self._seq,
            ts=time.time(),
            arb_id=int(arb_id),
            data=(data[:8].ljust(8, b"\x00")),
            src=src,
            name=MSG_NAMES.get(int(arb_id), "")
        )
        self._log.append(fr)
        self._last[int(arb_id)] = fr
        return fr

    def last(self, arb_id: int) -> Optional[CANFrame]:
        return self._last.get(int(arb_id))

    def consume_since(self, last_seq: int) -> Tuple[int, List[CANFrame]]:
        out = []
        max_seq = last_seq
        for fr in self._log:
            if fr.seq > last_seq:
                out.append(fr)
                if fr.seq > max_seq:
                    max_seq = fr.seq
        return max_seq, out

    def last_seq(self) -> int:
        return self._seq


# ============================================================
# --- Plant Models (Vehicle + Lead)
# ============================================================

@dataclass
class VehicleState:
    pos_m: float = 0.0
    v_mps: float = 0.0
    a_mps2: float = 0.0

class EgoVehiclePlant:
    """
    Longitudinal plant with drag + rolling resistance + jerk-limited acceleration,
    throttle/brake blending through EngineCmd & BrakeCmd (from CAN).
    """
    def __init__(self):
        self.state = VehicleState(pos_m=0.0, v_mps=kmh_to_mps(0.0), a_mps2=0.0)
        self.mass = 1500.0
        self.rho = 1.225
        self.CdA = 0.65
        self.Crr = 0.010
        self.F_drive_max = 4200.0
        self.a_brake_max = 7.0
        self.jerk_max = 2.5
        self._a_smooth = Smooth1P(tau=0.12, y0=0.0)

    def reset(self, speed_kmh: float = 0.0):
        self.state = VehicleState(pos_m=0.0, v_mps=kmh_to_mps(speed_kmh), a_mps2=0.0)
        self._a_smooth.y = 0.0

    def step(self, dt: float, throttle_pct: float, brake_pct: float):
        s = self.state
        v = max(0.0, s.v_mps)
        throttle = clamp(throttle_pct, 0.0, 100.0) / 100.0
        brake = clamp(brake_pct, 0.0, 100.0) / 100.0

        F_drive = throttle * self.F_drive_max
        a_drive = F_drive / self.mass

        a_brake = brake * self.a_brake_max  # positive magnitude
        a_drag = (0.5 * self.rho * self.CdA * v * v) / self.mass
        a_rr = (self.Crr * self.mass * G) / self.mass

        a_target = a_drive - a_brake - a_drag - a_rr
        a_target = clamp(a_target, -self.a_brake_max, +ACC_MAX_ACCEL + 1.5)

        # jerk limiting (no snaps)
        a_sm = self._a_smooth.step(a_target, dt)
        da = clamp(a_sm - s.a_mps2, -self.jerk_max * dt, self.jerk_max * dt)
        a = s.a_mps2 + da

        v2 = v + a * dt
        if v2 < 0.0:
            v2 = 0.0
            a = 0.0

        s.v_mps = v2
        s.a_mps2 = a
        s.pos_m += v2 * dt

class LeadVehiclePlant:
    def __init__(self):
        self.present = False
        self.state = VehicleState(pos_m=50.0, v_mps=kmh_to_mps(60.0), a_mps2=0.0)
        self.jerk_max = 3.5
        self.a_max = 3.0
        self.brake_max = 7.5
        self._a_smooth = Smooth1P(tau=0.20, y0=0.0)

        # behavior internal
        self._t = 0.0
        self._stopgo_phase = 0.0
        self._cutin_done = False
        self._hardbrake_done = False

    def reset(self, present: bool, distance_ahead_m: float, speed_kmh: float, ego_pos_m: float):
        self.present = present
        self.state.pos_m = ego_pos_m + max(10.0, distance_ahead_m + EGO_LENGTH_M)
        self.state.v_mps = kmh_to_mps(speed_kmh)
        self.state.a_mps2 = 0.0
        self._a_smooth.y = 0.0
        self._t = 0.0
        self._stopgo_phase = 0.0
        self._cutin_done = False
        self._hardbrake_done = False

    def step(self, dt: float, sim_type: SimType, target_speed_kmh: float, ego_pos_m: float, ego_speed_mps: float):
        self._t += dt
        if not self.present:
            return

        tgt_v = kmh_to_mps(target_speed_kmh)

        if sim_type == SimType.CONSTANT_LEAD_FOLLOW:
            pass

        elif sim_type == SimType.STOP_AND_GO:
            # Cycle: cruise -> slow -> stop -> launch
            # 0-6s cruise, 6-12 decel to 0, 12-16 stopped, 16-22 accel to target
            t = self._t % 22.0
            if t < 6.0:
                tgt_v = kmh_to_mps(target_speed_kmh)
            elif t < 12.0:
                tgt_v = 0.0
            elif t < 16.0:
                tgt_v = 0.0
            else:
                tgt_v = kmh_to_mps(target_speed_kmh)

        elif sim_type == SimType.CUT_IN:
            # Start without lead; in cut-in, lead "appears" at ~2s ahead at ~30m
            if not self._cutin_done:
                # Cut-in happens by environment toggling present; keep mild dynamics here.
                self._cutin_done = True

        elif sim_type == SimType.LEAD_HARD_BRAKE:
            # After 4s, lead brakes hard to low speed for a short time, then recovers.
            if self._t > 4.0 and not self._hardbrake_done:
                tgt_v = kmh_to_mps(20.0)
                if self.state.v_mps <= tgt_v + 0.5:
                    self._hardbrake_done = True
            elif self._hardbrake_done and self._t > 10.0:
                tgt_v = kmh_to_mps(target_speed_kmh)

        elif sim_type == SimType.FREE_DRIVE:
            # lead present is typically false, but if forced present -> behave constant
            pass
        elif sim_type == SimType.TRAFFIC_JAM:
            # oscillating low-speed traffic wave
            base_kmh = target_speed_kmh
            wave = 16.0 * (0.5 + 0.5 * math.sin(self._t * 0.9))
            tgt_v = kmh_to_mps(max(10.0, base_kmh - 25.0 + wave))

        # Simple speed tracking with accel limits + jerk limits
        v = self.state.v_mps
        dv = tgt_v - v
        a_target = clamp(dv * 0.8, -self.brake_max, +self.a_max)
        a_sm = self._a_smooth.step(a_target, dt)
        da = clamp(a_sm - self.state.a_mps2, -self.jerk_max * dt, self.jerk_max * dt)
        a = self.state.a_mps2 + da
        v2 = max(0.0, v + a * dt)

        self.state.v_mps = v2
        self.state.a_mps2 = a
        self.state.pos_m += v2 * dt

        # ensure lead stays ahead (no weird overlaps in the 2D view)
        min_ahead = ego_pos_m + EGO_LENGTH_M + 5.0
        if self.state.pos_m < min_ahead:
            self.state.pos_m = min_ahead
            self.state.v_mps = max(self.state.v_mps, ego_speed_mps * 0.8)


# ============================================================
# --- ECUs
# ============================================================

class ECUBase:
    def __init__(self, bus: VirtualCANBus, name: str):
        self.bus = bus
        self.name = name

    def tick(self, dt: float):
        pass

class DriverInputsECU(ECUBase):
    """
    Holds HMI button pulses + driver pedals + brake switch flags.
    Sends a heartbeat frame periodically. Button presses are sent as 1-cycle pulses.
    """
    def __init__(self, bus: VirtualCANBus):
        super().__init__(bus, "DriverInputsECU")
        self.bs1 = 0
        self.bs2 = 0
        self.driver_throttle = 0
        self.driver_brake = 0
        self._pulse_btn = 0
        self._ctr = 0

    def pulse(self, btn_flag: int):
        self._pulse_btn |= btn_flag

    def set_bs(self, bs1: bool, bs2: bool):
        self.bs1 = 1 if bs1 else 0
        self.bs2 = 1 if bs2 else 0

    def set_pedals(self, throttle_pct: int, brake_pct: int):
        self.driver_throttle = int(clamp(throttle_pct, 0, 100))
        self.driver_brake = int(clamp(brake_pct, 0, 100))

    def tick(self, dt: float):
        self._ctr = (self._ctr + 1) & 0xFF
        bs_flags = (BS1_BIT if self.bs1 else 0) | (BS2_BIT if self.bs2 else 0)
        data = CANCodec.enc_driver_inputs(self._pulse_btn, bs_flags, self.driver_throttle, self.driver_brake, self._ctr)
        self.bus.send(ID_DRIVER_INPUTS, data, src=self.name)
        self._pulse_btn = 0  # clear pulses after send

class LeadControlECU(ECUBase):
    """
    Lets HMI define simulation type and basic lead settings via CAN, so the environment reads it via bus.
    """
    def __init__(self, bus: VirtualCANBus):
        super().__init__(bus, "LeadControlECU")
        self.sim_type = SimType.FREE_DRIVE
        self.lead_present = False
        self.lead_speed_kmh = 70.0
        self.lead_distance_m = 60.0

    def tick(self, dt: float):
        data = CANCodec.enc_lead_ctrl(int(self.sim_type), 1 if self.lead_present else 0, self.lead_speed_kmh, self.lead_distance_m)
        self.bus.send(ID_LEAD_CTRL, data, src=self.name)

class RadarECU(ECUBase):
    """
    Reads EGO_STATE + LEAD_STATE and publishes RADAR_OBJECT with optional noise/latency.
    """
    def __init__(self, bus: VirtualCANBus):
        super().__init__(bus, "RadarECU")
        self._delay_queue = deque(maxlen=200)
        self._rng = random.Random(42)

    def tick(self, dt: float):
        ego_fr = self.bus.last(ID_EGO_STATE)
        lead_fr = self.bus.last(ID_LEAD_STATE)
        ctrl_fr = self.bus.last(ID_LEAD_CTRL)

        if not ego_fr or not lead_fr:
            return

        ego = CANCodec.dec_ego_state(ego_fr.data)
        lead = CANCodec.dec_lead_state(lead_fr.data)
        ctrl = CANCodec.dec_lead_ctrl(ctrl_fr.data) if ctrl_fr else {"sim_type": int(SimType.FREE_DRIVE)}

        sim_type = SimType(int(ctrl.get("sim_type", 0)))

        ego_pos = ego["pos_m"]
        ego_v = kmh_to_mps(ego["speed_kmh"])
        lead_pos = lead["pos_m"]
        lead_v = kmh_to_mps(lead["speed_kmh"])

        present = lead["present"]
        dist = (lead_pos - ego_pos) - EGO_LENGTH_M
        if not present or dist < 0.0:
            present = False
            dist = 0.0

        rel_v = (lead_v - ego_v)

        # Noise/latency mode
        latency_s = 0.0
        noise_dist = 0.0
        noise_rel  = 0.0
        dropout = False

        meas_present = present and not dropout
        meas_dist = max(0.0, dist + noise_dist)
        meas_rel  = rel_v + noise_rel
        meas_lead_speed_kmh = mps_to_kmh(lead_v)

        payload = (meas_present, meas_dist, meas_rel, meas_lead_speed_kmh)

        data = CANCodec.enc_radar_object(meas_present, meas_dist, meas_rel, meas_lead_speed_kmh)
        self.bus.send(ID_RADAR_OBJECT, data, src=self.name)

class ACCECU(ECUBase):
    """
    ACC state machine + control.
    Requirements enforced:
      - Activation only if ego speed ≥ 40 km/h and BS1=0 & BS2=0
      - Cancellation if brake pressed (BS1/BS2) or speed < 25 km/h or OFF
      - Auto switch to time-gap control when lead detected
      - Decel authority limited to 0.2g; if demand exceeds => DRIVER_INTERVENTION_REQUIRED
    """
    def __init__(self, bus: VirtualCANBus):
        super().__init__(bus, "ACCECU")
        self.state = AccState.ACC_OFF
        self.set_speed_kmh = 0.0
        self.time_gap_s = 1.6
        self._msg = MSG_NONE
        self._last_active_set_speed = 0.0
        self._intervention_hold = 0.0

        # filters for smooth control
        self._acc_cmd_f = Smooth1P(tau=0.18, y0=0.0)

    def tick(self, dt: float):
        self._msg = MSG_NONE

        ego_fr = self.bus.last(ID_EGO_STATE)
        inp_fr = self.bus.last(ID_DRIVER_INPUTS)
        rad_fr = self.bus.last(ID_RADAR_OBJECT)

        if not ego_fr or not inp_fr:
            return

        ego = CANCodec.dec_ego_state(ego_fr.data)
        inp = CANCodec.dec_driver_inputs(inp_fr.data)
        rad = CANCodec.dec_radar_object(rad_fr.data) if rad_fr else {"present": False, "dist_m": 0.0, "rel_speed_mps": 0.0}

        ego_speed_kmh = ego["speed_kmh"]
        ego_speed_mps = kmh_to_mps(ego_speed_kmh)

        btn = inp["btn"]
        bs = inp["bs"]

        brake_pressed = ((bs & BS1_BIT) != 0) or ((bs & BS2_BIT) != 0) or (inp["brk"] > 2)
        activation_ok = (ego_speed_kmh >= 40.0) and ((bs & (BS1_BIT | BS2_BIT)) == 0)

        # Button pulses
        on_p      = (btn & BTN_ON) != 0
        off_p     = (btn & BTN_OFF) != 0
        setp_p    = (btn & BTN_SET_PLUS) != 0
        coast_p   = (btn & BTN_COAST) != 0
        resume_p  = (btn & BTN_RESUME) != 0
        tg_p      = (btn & BTN_TG_PLUS) != 0
        tg_m      = (btn & BTN_TG_MINUS) != 0

        # Cancellation conditions (speed < 25 in active)
        if self.state in (AccState.ACC_ACTIVE_SPEED_CONTROL, AccState.ACC_ACTIVE_TIME_GAP_CONTROL, AccState.DRIVER_INTERVENTION_REQUIRED):
            if off_p:
                self.state = AccState.ACC_OFF
                self._msg = MSG_CANCEL_OFF
            elif brake_pressed:
                self.state = AccState.ACC_STANDBY
                self._msg = MSG_CANCEL_BRAKE
            elif ego_speed_kmh < 25.0:
                self.state = AccState.ACC_STANDBY
                self._msg = MSG_CANCEL_SPEED_LOW

        # OFF handling from anywhere
        if off_p and self.state != AccState.ACC_OFF:
            self.state = AccState.ACC_OFF
            self._msg = MSG_CANCEL_OFF

        # State transitions
        if self.state == AccState.ACC_OFF:
            if on_p:
                if activation_ok:
                    self.state = AccState.ACC_STANDBY
                    self._msg = MSG_ACTIVATED
                else:
                    self._msg = MSG_ACTIVATION_DENIED

        elif self.state == AccState.ACC_STANDBY:
            if on_p:
                if not activation_ok:
                    self._msg = MSG_ACTIVATION_DENIED
            if setp_p:
                if activation_ok:
                    self.set_speed_kmh = max(40.0, ego_speed_kmh)
                    self._last_active_set_speed = self.set_speed_kmh
                    self.state = AccState.ACC_ACTIVE_SPEED_CONTROL
                    self._msg = MSG_ACTIVATED
                else:
                    self._msg = MSG_ACTIVATION_DENIED
            elif resume_p:
                if activation_ok and self._last_active_set_speed >= 40.0:
                    self.set_speed_kmh = self._last_active_set_speed
                    self.state = AccState.ACC_ACTIVE_SPEED_CONTROL
                    self._msg = MSG_RESUMED
                elif resume_p:
                    self._msg = MSG_ACTIVATION_DENIED

        # Adjust set speed / time gap in active states
        if self.state in (AccState.ACC_ACTIVE_SPEED_CONTROL, AccState.ACC_ACTIVE_TIME_GAP_CONTROL):
            if setp_p:
                self.set_speed_kmh = clamp(self.set_speed_kmh + 5.0, 40.0, 180.0)
                self._last_active_set_speed = self.set_speed_kmh
                self._msg = MSG_SET_SPEED_UPDATED
            if coast_p:
                self.set_speed_kmh = clamp(self.set_speed_kmh - 5.0, 40.0, 180.0)
                self._last_active_set_speed = self.set_speed_kmh
                self._msg = MSG_SET_SPEED_UPDATED
            if tg_p:
                self.time_gap_s = clamp(self.time_gap_s + 0.2, 1.0, 2.6)
                self._msg = MSG_TIMEGAP_UPDATED
            if tg_m:
                self.time_gap_s = clamp(self.time_gap_s - 0.2, 1.0, 2.6)
                self._msg = MSG_TIMEGAP_UPDATED

        lead_present = bool(rad.get("present", False))
        dist_m = float(rad.get("dist_m", 0.0))
        rel_v = float(rad.get("rel_speed_mps", 0.0))

        # Auto switch between speed/time-gap
        if self.state == AccState.ACC_ACTIVE_SPEED_CONTROL and lead_present:
            self.state = AccState.ACC_ACTIVE_TIME_GAP_CONTROL
            self._msg = MSG_LEAD_DETECTED
        elif self.state == AccState.ACC_ACTIVE_TIME_GAP_CONTROL and (not lead_present):
            self.state = AccState.ACC_ACTIVE_SPEED_CONTROL
            self._msg = MSG_LEAD_LOST

        # Control law -> desired acceleration
        acc_cmd = 0.0
        if self.state == AccState.ACC_ACTIVE_SPEED_CONTROL:
            # simple speed controller with gentle decel, no aggressive braking
            speed_err = kmh_to_mps(self.set_speed_kmh) - ego_speed_mps
            acc_cmd = clamp(0.65 * speed_err, -1.2, +ACC_MAX_ACCEL)
        elif self.state == AccState.ACC_ACTIVE_TIME_GAP_CONTROL:
            # time gap control: target distance = d0 + tg*v
            d0 = 8.0
            target = d0 + self.time_gap_s * max(ego_speed_mps, 0.0)
            gap_err = (dist_m - target)
            # PD in distance domain + relative speed damping
            acc_cmd = 0.05 * gap_err + 0.65 * rel_v
            # do not exceed set speed (cap accel if near/above)
            if ego_speed_kmh > self.set_speed_kmh + 1.0:
                acc_cmd = min(acc_cmd, 0.0)
            acc_cmd = clamp(acc_cmd, -4.0, +ACC_MAX_ACCEL)

            # If very close, demand stronger braking (may exceed authority -> triggers intervention)
            if dist_m < max(5.0, 0.6 * target):
                # heuristic emergency-like demand (still limited by authority)
                closeness = max(0.0, (max(5.0, 0.6 * target) - dist_m))
                acc_cmd -= 0.35 * closeness

        elif self.state == AccState.DRIVER_INTERVENTION_REQUIRED:
            # stay at max allowed decel until situation becomes safe enough
            self._intervention_hold = max(0.0, self._intervention_hold - dt)
            # recompute desired in time-gap if lead present else speed
            if lead_present:
                d0 = 8.0
                target = d0 + self.time_gap_s * max(ego_speed_mps, 0.0)
                gap_err = (dist_m - target)
                acc_want = 0.05 * gap_err + 0.65 * rel_v
                if dist_m < max(5.0, 0.6 * target):
                    closeness = max(0.0, (max(5.0, 0.6 * target) - dist_m))
                    acc_want -= 0.35 * closeness
            else:
                speed_err = kmh_to_mps(self.set_speed_kmh) - ego_speed_mps
                acc_want = clamp(0.65 * speed_err, -1.2, +ACC_MAX_ACCEL)

            # Exit if safe for ~1s and within authority
            if acc_want >= -ACC_DECEL_AUTHORITY * 0.95:
                if self._intervention_hold <= 0.0:
                    self.state = AccState.ACC_ACTIVE_TIME_GAP_CONTROL if lead_present else AccState.ACC_ACTIVE_SPEED_CONTROL
                    self._msg = MSG_LEAD_DETECTED if lead_present else MSG_LEAD_LOST
                acc_cmd = max(acc_want, -ACC_DECEL_AUTHORITY)
            else:
                acc_cmd = -ACC_DECEL_AUTHORITY
                self._msg = MSG_INTERVENTION_REQUIRED

        # Authority enforcement
        if self.state in (AccState.ACC_ACTIVE_SPEED_CONTROL, AccState.ACC_ACTIVE_TIME_GAP_CONTROL):
            if acc_cmd < -ACC_DECEL_AUTHORITY:
                self.state = AccState.DRIVER_INTERVENTION_REQUIRED
                self._intervention_hold = 1.0
                self._msg = MSG_INTERVENTION_REQUIRED
                acc_cmd = -ACC_DECEL_AUTHORITY

        acc_cmd = clamp(acc_cmd, -ACC_DECEL_AUTHORITY, +ACC_MAX_ACCEL)
        acc_cmd = self._acc_cmd_f.step(acc_cmd, dt)

        data = CANCodec.enc_acc_status(int(self.state), int(self._msg), self.set_speed_kmh, self.time_gap_s, acc_cmd)
        self.bus.send(ID_ACC_STATUS, data, src=self.name)

class EngineECU(ECUBase):
    def __init__(self, bus: VirtualCANBus):
        super().__init__(bus, "EngineECU")
        self._thr_f = Smooth1P(tau=0.25, y0=0.0)

    def tick(self, dt: float):
        acc_fr = self.bus.last(ID_ACC_STATUS)
        in_fr  = self.bus.last(ID_DRIVER_INPUTS)
        if not acc_fr or not in_fr:
            return
        acc = CANCodec.dec_acc_status(acc_fr.data)
        inp = CANCodec.dec_driver_inputs(in_fr.data)

        acc_cmd = acc["acc_cmd_mps2"]
        driver_thr = float(inp["thr"])

        thr_acc = 0.0
        if acc_cmd > 0.0:
            thr_acc = clamp((acc_cmd / ACC_MAX_ACCEL) * 100.0, 0.0, 100.0)

        thr = max(driver_thr, thr_acc)
        thr = self._thr_f.step(thr, dt)

        data = CANCodec.enc_engine_cmd(thr, acc_cmd)
        self.bus.send(ID_ENGINE_CMD, data, src=self.name)

class BrakeECU(ECUBase):
    def __init__(self, bus: VirtualCANBus):
        super().__init__(bus, "BrakeECU")
        self._brk_f = Smooth1P(tau=0.22, y0=0.0)

    def tick(self, dt: float):
        acc_fr = self.bus.last(ID_ACC_STATUS)
        in_fr  = self.bus.last(ID_DRIVER_INPUTS)
        if not acc_fr or not in_fr:
            return
        acc = CANCodec.dec_acc_status(acc_fr.data)
        inp = CANCodec.dec_driver_inputs(in_fr.data)

        acc_cmd = acc["acc_cmd_mps2"]
        driver_brk = float(inp["brk"])

        brk_acc = 0.0
        if acc_cmd < 0.0:
            brk_acc = clamp((-acc_cmd / ACC_DECEL_AUTHORITY) * 100.0, 0.0, 100.0)

        brk = max(driver_brk, brk_acc)
        brk = self._brk_f.step(brk, dt)

        brake_lights = brk >= 2.0
        decel_cmd = - (brk / 100.0) * 7.0

        data = CANCodec.enc_brake_cmd(brk, brake_lights, decel_cmd)
        self.bus.send(ID_BRAKE_CMD, data, src=self.name)

class ClusterECU(ECUBase):
    def __init__(self, bus: VirtualCANBus):
        super().__init__(bus, "ClusterECU")

    def tick(self, dt: float):
        ego_fr = self.bus.last(ID_EGO_STATE)
        rad_fr = self.bus.last(ID_RADAR_OBJECT)
        acc_fr = self.bus.last(ID_ACC_STATUS)
        brk_fr = self.bus.last(ID_BRAKE_CMD)
        if not ego_fr:
            return
        ego = CANCodec.dec_ego_state(ego_fr.data)
        rad = CANCodec.dec_radar_object(rad_fr.data) if rad_fr else {"present": False, "dist_m": 0.0, "rel_speed_mps": 0.0}
        acc = CANCodec.dec_acc_status(acc_fr.data) if acc_fr else {"state": int(AccState.ACC_OFF)}
        brk = CANCodec.dec_brake_cmd(brk_fr.data) if brk_fr else {"brake_lights": False}

        dist = rad["dist_m"] if rad.get("present", False) else 0.0
        rel_kmh = mps_to_kmh(rad.get("rel_speed_mps", 0.0))
        data = CANCodec.enc_cluster(ego["speed_kmh"], dist, rel_kmh, int(acc.get("state", 0)), bool(brk.get("brake_lights", False)))
        self.bus.send(ID_CLUSTER, data, src=self.name)


# ============================================================
# --- Simulation Core (Environment + ECU scheduling)
# ============================================================

class SimCore(QObject):
    updated = Signal()

    def __init__(self):
        super().__init__()
        self.bus = VirtualCANBus()
        self.driver = DriverInputsECU(self.bus)
        self.leadctrl = LeadControlECU(self.bus)
        self.plant = EgoVehiclePlant()
        self.lead = LeadVehiclePlant()
        self.radar = RadarECU(self.bus)
        self.acc = ACCECU(self.bus)
        self.engine = EngineECU(self.bus)
        self.brake = BrakeECU(self.bus)
        self.cluster = ClusterECU(self.bus)

        self._accumulator = 0.0
        self._timer = QElapsedTimer()
        self._timer.start()
        self._last_real = self._timer.elapsed() / 1000.0

        # caches for UI
        self.ui_cache = {
            "speed_kmh": 0.0, "acc_cmd": 0.0, "throttle_pct": 0.0, "brake_pct": 0.0,
            "brake_lights": False, "acc_state": int(AccState.ACC_OFF), "msg": "",
            "set_speed_kmh": 0.0, "time_gap_s": 1.6, "lead_present": False, "dist_m": 0.0, "rel_speed_kmh": 0.0
        }

        # initialize CAN with baseline frames
        self._publish_environment_frames()

    def reset_world(self, ego_speed_kmh: float, lead_present: bool, lead_speed_kmh: float, lead_dist_m: float):
        self.plant.reset(speed_kmh=ego_speed_kmh)
        self.lead.reset(present=lead_present, distance_ahead_m=lead_dist_m, speed_kmh=lead_speed_kmh, ego_pos_m=self.plant.state.pos_m)

    def _publish_environment_frames(self):
        # EGO
        s = self.plant.state
        self.bus.send(ID_EGO_STATE, CANCodec.enc_ego_state(mps_to_kmh(s.v_mps), s.a_mps2, s.pos_m), src="ENV")
        # LEAD
        if self.lead.present:
            ls = self.lead.state
            self.bus.send(ID_LEAD_STATE, CANCodec.enc_lead_state(True, mps_to_kmh(ls.v_mps), ls.pos_m), src="ENV")
        else:
            self.bus.send(ID_LEAD_STATE, CANCodec.enc_lead_state(False, 0.0, 0.0), src="ENV")

    def step_real_time(self):
        now = self._timer.elapsed() / 1000.0
        dt_real = clamp(now - self._last_real, 0.0, 0.05)  # avoid spiral
        self._last_real = now
        self._accumulator += dt_real

        while self._accumulator >= SIM_FIXED_DT:
            self._fixed_step(SIM_FIXED_DT)
            self._accumulator -= SIM_FIXED_DT

        self._update_ui_cache()
        self.updated.emit()

    def _fixed_step(self, dt: float):
        # HMI -> lead control broadcast
        self.leadctrl.tick(dt)
        self.driver.tick(dt)

        # Read lead control (environment uses CAN only)
        ctrl_fr = self.bus.last(ID_LEAD_CTRL)
        ctrl = CANCodec.dec_lead_ctrl(ctrl_fr.data) if ctrl_fr else {"sim_type": int(SimType.FREE_DRIVE), "present": False, "lead_speed_kmh": 0.0, "lead_distance_m": 60.0}
        sim_type = SimType(int(ctrl.get("sim_type", 0)))

        # Manage lead presence in special sim types (Cut-in)
        if sim_type == SimType.CUT_IN:
            # Lead appears after ~2 seconds if not already present
            # (using environment time based on lead internal t)
            if not self.lead.present:
                # keep lead "hidden", then spawn
                # Use a simple counter in lead plant
                self.lead._t += dt
                if self.lead._t > 2.0:
                    self.lead.reset(True, ctrl.get("lead_distance_m", 30.0), ctrl.get("lead_speed_kmh", 60.0), self.plant.state.pos_m)
        else:
            self.lead.present = bool(ctrl.get("present", False))

        if self.lead.present and (self.bus.last(ID_LEAD_STATE) is None):
            self.lead.reset(True, ctrl.get("lead_distance_m", 60.0), ctrl.get("lead_speed_kmh", 70.0), self.plant.state.pos_m)

        # Step lead
        self.lead.step(
            dt=dt,
            sim_type=sim_type,
            target_speed_kmh=float(ctrl.get("lead_speed_kmh", 70.0)),
            ego_pos_m=self.plant.state.pos_m,
            ego_speed_mps=self.plant.state.v_mps
        )

        # Publish environment states
        self._publish_environment_frames()

        # Sensors/control/actuators (via CAN only)
        self.radar.tick(dt)
        self.acc.tick(dt)
        self.engine.tick(dt)
        self.brake.tick(dt)

        # Apply commands to plant (read from CAN)
        eng_fr = self.bus.last(ID_ENGINE_CMD)
        brk_fr = self.bus.last(ID_BRAKE_CMD)
        thr = CANCodec.dec_engine_cmd(eng_fr.data)["throttle_pct"] if eng_fr else 0.0
        brk = CANCodec.dec_brake_cmd(brk_fr.data)["brake_pct"] if brk_fr else 0.0

        self.plant.step(dt, throttle_pct=thr, brake_pct=brk)

        # Cluster output (for clean HMI signal)
        self.cluster.tick(dt)

    def _update_ui_cache(self):
        ego_fr = self.bus.last(ID_EGO_STATE)
        rad_fr = self.bus.last(ID_RADAR_OBJECT)
        acc_fr = self.bus.last(ID_ACC_STATUS)
        eng_fr = self.bus.last(ID_ENGINE_CMD)
        brk_fr = self.bus.last(ID_BRAKE_CMD)

        if ego_fr:
            ego = CANCodec.dec_ego_state(ego_fr.data)
            self.ui_cache["speed_kmh"] = ego["speed_kmh"]
        if rad_fr:
            rad = CANCodec.dec_radar_object(rad_fr.data)
            self.ui_cache["lead_present"] = bool(rad["present"])
            self.ui_cache["dist_m"] = float(rad["dist_m"]) if rad["present"] else 0.0
            self.ui_cache["rel_speed_kmh"] = mps_to_kmh(float(rad["rel_speed_mps"])) if rad["present"] else 0.0
        if acc_fr:
            acc = CANCodec.dec_acc_status(acc_fr.data)
            self.ui_cache["acc_state"] = int(acc["state"])
            self.ui_cache["set_speed_kmh"] = float(acc["set_speed_kmh"])
            self.ui_cache["time_gap_s"] = float(acc["time_gap_s"])
            self.ui_cache["acc_cmd"] = float(acc["acc_cmd_mps2"])
            self.ui_cache["msg"] = MSG_TEXT.get(int(acc["msg"]), "")
        if eng_fr:
            eng = CANCodec.dec_engine_cmd(eng_fr.data)
            self.ui_cache["throttle_pct"] = float(eng["throttle_pct"])
        if brk_fr:
            brk = CANCodec.dec_brake_cmd(brk_fr.data)
            self.ui_cache["brake_pct"] = float(brk["brake_pct"])
            self.ui_cache["brake_lights"] = bool(brk["brake_lights"])


# ============================================================
# --- Premium UI Components (Cards, Gauges, Road View)
# ============================================================

DARK_QSS = """
* { font-family: "Segoe UI"; }
QMainWindow { background: #0b0f14; }
QWidget { color: #d7e0ea; }
QFrame#Card {
    background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #111826, stop:1 #0c121d);
    border: 1px solid rgba(255,255,255,0.06);
    border-radius: 16px;
}
QLabel#Title { color: #eaf2ff; font-size: 14px; font-weight: 700; }
QLabel#BigValue { font-size: 44px; font-weight: 800; letter-spacing: 0.5px; }
QLabel#Subtle { color: rgba(215,224,234,0.65); }
QPushButton {
    background: rgba(255,255,255,0.06);
    border: 1px solid rgba(255,255,255,0.08);
    border-radius: 12px;
    padding: 8px 10px;
}
QPushButton:hover { background: rgba(255,255,255,0.09); }
QPushButton:pressed { background: rgba(255,255,255,0.12); }
QPushButton#Primary {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:1, stop:0 #1b5cff, stop:1 #35d1ff);
    border: 1px solid rgba(255,255,255,0.10);
    color: #071018;
    font-weight: 800;
}
QComboBox {
    background: rgba(255,255,255,0.06);
    border: 1px solid rgba(255,255,255,0.08);
    border-radius: 10px;
    padding: 6px 10px;
}
QComboBox QAbstractItemView {
    background: #0f1623;
    border: 1px solid rgba(255,255,255,0.10);
    selection-background-color: rgba(53,209,255,0.18);
}
QLineEdit {
    background: rgba(255,255,255,0.05);
    border: 1px solid rgba(255,255,255,0.08);
    border-radius: 10px;
    padding: 6px 10px;
}
QCheckBox::indicator {
    width: 18px; height: 18px;
}
QCheckBox::indicator:unchecked {
    border-radius: 6px;
    border: 1px solid rgba(255,255,255,0.18);
    background: rgba(255,255,255,0.04);
}
QCheckBox::indicator:checked {
    border-radius: 6px;
    border: 1px solid rgba(53,209,255,0.50);
    background: rgba(53,209,255,0.30);
}
QTableWidget {
    background: rgba(255,255,255,0.03);
    border: 1px solid rgba(255,255,255,0.08);
    border-radius: 14px;
    gridline-color: rgba(255,255,255,0.06);
}
QHeaderView::section {
    background: rgba(255,255,255,0.05);
    padding: 6px;
    border: none;
    color: rgba(215,224,234,0.70);
}
QTextEdit {
    background: rgba(255,255,255,0.03);
    border: 1px solid rgba(255,255,255,0.08);
    border-radius: 14px;
}
"""

def make_shadow(widget: QWidget, radius=24, dx=0, dy=10, alpha=90):
    sh = QGraphicsDropShadowEffect()
    sh.setBlurRadius(radius)
    sh.setOffset(dx, dy)
    sh.setColor(QColor(0, 0, 0, alpha))
    widget.setGraphicsEffect(sh)

class Card(QFrame):
    def __init__(self, title: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("Card")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 14, 14, 14)
        lay.setSpacing(10)
        if title:
            t = QLabel(title)
            t.setObjectName("Title")
            lay.addWidget(t)
        self.body = QWidget(self)
        lay.addWidget(self.body, 1)
        make_shadow(self)

class SpeedGauge(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self._speed = 0.0
        self._smooth = Smooth1P(tau=0.12, y0=0.0)

    def set_speed(self, speed_kmh: float, dt: float):
        self._speed = self._smooth.step(speed_kmh, dt)
        self.update()

    def paintEvent(self, e):
        w, h = self.width(), self.height()
        r = min(w, h) * 0.42
        cx, cy = w * 0.50, h * 0.53

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        # subtle radial background
        bg = QRadialGradient(QPointF(cx, cy), r * 1.35)
        bg.setColorAt(0.0, QColor(40, 60, 90, 90))
        bg.setColorAt(1.0, QColor(10, 14, 20, 0))
        p.setBrush(bg)
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(cx, cy), r * 1.20, r * 1.20)

        # arc
        start_deg = 225
        span_deg = 270
        max_speed = 220.0
        frac = clamp(self._speed / max_speed, 0.0, 1.0)
        span = span_deg * frac

        base_pen = QPen(QColor(255, 255, 255, 18), r * 0.14, Qt.SolidLine, Qt.RoundCap)
        p.setPen(base_pen)
        p.setBrush(Qt.NoBrush)
        p.drawArc(QRectF(cx - r, cy - r, 2*r, 2*r), int(start_deg * 16), int(-span_deg * 16))

        glow = QPen(QColor(53, 209, 255, 200), r * 0.14, Qt.SolidLine, Qt.RoundCap)
        p.setPen(glow)
        p.drawArc(QRectF(cx - r, cy - r, 2*r, 2*r), int(start_deg * 16), int(-span * 16))

        # ticks
        p.setPen(QPen(QColor(255, 255, 255, 26), 2))
        for s in range(0, 221, 20):
            a = (start_deg - (span_deg * (s / max_speed)))
            rad = math.radians(a)
            x1 = cx + math.cos(rad) * (r * 0.92)
            y1 = cy - math.sin(rad) * (r * 0.92)
            x2 = cx + math.cos(rad) * (r * 1.05)
            y2 = cy - math.sin(rad) * (r * 1.05)
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # needle
        needle_angle = start_deg - span_deg * frac
        rad = math.radians(needle_angle)
        nx = cx + math.cos(rad) * (r * 0.80)
        ny = cy - math.sin(rad) * (r * 0.80)
        p.setPen(QPen(QColor(234, 242, 255, 210), 3, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(cx, cy), QPointF(nx, ny))

        p.setBrush(QColor(234, 242, 255, 220))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(cx, cy), r * 0.08, r * 0.08)

        # text
        p.setPen(QColor(234, 242, 255))
        f = QFont("Segoe UI", 28, QFont.Weight.Bold)
        p.setFont(f)
        p.drawText(QRectF(0, h*0.50, w, h*0.30), Qt.AlignHCenter | Qt.AlignTop, f"{int(round(self._speed))}")
        p.setPen(QColor(215, 224, 234, 160))
        p.setFont(QFont("Segoe UI", 11, QFont.Weight.DemiBold))
        p.drawText(QRectF(0, h*0.78, w, h*0.12), Qt.AlignHCenter | Qt.AlignTop, "km/h")

class BarIndicator(QWidget):
    def __init__(self, label: str, color: QColor, parent=None):
        super().__init__(parent)
        self.label = label
        self.color = color
        self._val = 0.0
        self._smooth = Smooth1P(tau=0.14, y0=0.0)
        self.setMinimumHeight(44)

    def set_value(self, pct: float, dt: float):
        self._val = self._smooth.step(clamp(pct, 0.0, 100.0), dt)
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        rect = self.rect().adjusted(6, 6, -6, -6)
        radius = 12

        # background
        p.setPen(QPen(QColor(255, 255, 255, 18), 1))
        p.setBrush(QColor(255, 255, 255, 10))
        p.drawRoundedRect(rect, radius, radius)

        # fill
        fill_w = rect.width() * (self._val / 100.0)
        fill = QRectF(rect.left(), rect.top(), fill_w, rect.height())
        grad = QLinearGradient(fill.topLeft(), fill.topRight())
        grad.setColorAt(0.0, QColor(self.color.red(), self.color.green(), self.color.blue(), 220))
        grad.setColorAt(1.0, QColor(self.color.red(), self.color.green(), self.color.blue(), 90))
        p.setBrush(grad)
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(fill, radius, radius)

        # text
        p.setPen(QColor(234, 242, 255))
        p.setFont(QFont("Segoe UI", 10, QFont.Weight.DemiBold))
        p.drawText(rect.adjusted(10, 0, -10, 0), Qt.AlignVCenter | Qt.AlignLeft, self.label)
        p.setPen(QColor(234, 242, 255, 210))
        p.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        p.drawText(rect.adjusted(10, 0, -10, 0), Qt.AlignVCenter | Qt.AlignRight, f"{int(self._val)}%")

class RoadView(QWidget):
    """
    Top-down, vertical road with scrolling effect. Ego fixed near bottom; lead drawn ahead.
    Includes radar cone, distance & relative speed overlays, stylized cars with gradients and brake lights.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(520, 680)
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)

        self.speed_kmh = 0.0
        self.dist_m = 0.0
        self.rel_kmh = 0.0
        self.lead_present = False
        self.brake_lights = False

        self._scroll = 0.0
        self._scroll_smooth = Smooth1P(tau=0.25, y0=0.0)
        self._dist_smooth = Smooth1P(tau=0.18, y0=60.0)

    def update_signals(self, speed_kmh: float, lead_present: bool, dist_m: float, rel_kmh: float, brake_lights: bool, dt: float):
        self.speed_kmh = speed_kmh
        self.lead_present = lead_present
        self.dist_m = self._dist_smooth.step(dist_m, dt)
        self.rel_kmh = rel_kmh
        self.brake_lights = brake_lights

        scroll_target = kmh_to_mps(speed_kmh) * dt * 45.0
        self._scroll = self._scroll_smooth.step(self._scroll + scroll_target, dt)
        self.update()

    def _draw_road(self, p: QPainter, w: int, h: int):
        # road base
        road_w = min(w * 0.72, w - 40)
        road_x = (w - road_w) * 0.5

        # asphalt gradient
        grad = QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0.0, QColor(25, 30, 40))
        grad.setColorAt(1.0, QColor(12, 16, 22))
        p.setBrush(grad)
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(QRectF(road_x, -20, road_w, h + 40), 22, 22)

        # shoulders
        sh = QPen(QColor(255, 255, 255, 22), 8)
        p.setPen(sh)
        p.drawLine(QPointF(road_x + 6, 0), QPointF(road_x + 6, h))
        p.drawLine(QPointF(road_x + road_w - 6, 0), QPointF(road_x + road_w - 6, h))

        # lane markings (two lanes)
        mid = road_x + road_w * 0.5
        dash_h = 26
        gap = 18
        y = -((self._scroll) % (dash_h + gap))
        p.setPen(QPen(QColor(235, 240, 250, 160), 4, Qt.SolidLine, Qt.RoundCap))
        while y < h:
            p.drawLine(QPointF(mid, y), QPointF(mid, y + dash_h))
            y += dash_h + gap

        # subtle vignette
        vig = QRadialGradient(QPointF(w*0.5, h*0.65), w*0.7)
        vig.setColorAt(0.0, QColor(0, 0, 0, 0))
        vig.setColorAt(1.0, QColor(0, 0, 0, 140))
        p.setBrush(vig)
        p.setPen(Qt.NoPen)
        p.drawRect(0, 0, w, h)

    def _car_path(self, x: float, y: float, car_w: float, car_h: float) -> QPainterPath:
        r = car_w * 0.18
        path = QPainterPath()
        path.addRoundedRect(QRectF(x - car_w/2, y - car_h/2, car_w, car_h), r, r)

        # roof
        roof_w = car_w * 0.58
        roof_h = car_h * 0.42
        rx = x - roof_w/2
        ry = y - car_h*0.18 - roof_h/2
        path.addRoundedRect(QRectF(rx, ry, roof_w, roof_h), r*0.8, r*0.8)
        return path

    def _draw_car(self, p: QPainter, x: float, y: float, car_w: float, car_h: float, color1: QColor, color2: QColor, brake_lights: bool, glow: bool):
        path = self._car_path(x, y, car_w, car_h)
        grad = QLinearGradient(QPointF(x - car_w/2, y - car_h/2), QPointF(x + car_w/2, y + car_h/2))
        grad.setColorAt(0.0, color1)
        grad.setColorAt(1.0, color2)

        p.setPen(QPen(QColor(255, 255, 255, 26), 1))
        p.setBrush(grad)
        p.drawPath(path)

        # reflection stripe
        p.setPen(Qt.NoPen)
        refl = QPainterPath()
        refl.addRoundedRect(QRectF(x - car_w*0.28, y - car_h*0.35, car_w*0.22, car_h*0.70), 10, 10)
        p.setBrush(QColor(255, 255, 255, 18))
        p.drawPath(refl)

        # wheels
        wheel_w = car_w * 0.16
        wheel_h = car_h * 0.18
        p.setBrush(QColor(10, 12, 16, 220))
        p.drawRoundedRect(QRectF(x - car_w*0.55, y - car_h*0.30, wheel_w, wheel_h), 6, 6)
        p.drawRoundedRect(QRectF(x + car_w*0.39, y - car_h*0.30, wheel_w, wheel_h), 6, 6)
        p.drawRoundedRect(QRectF(x - car_w*0.55, y + car_h*0.12, wheel_w, wheel_h), 6, 6)
        p.drawRoundedRect(QRectF(x + car_w*0.39, y + car_h*0.12, wheel_w, wheel_h), 6, 6)

        # brake lights
        if brake_lights:
            p.setBrush(QColor(255, 60, 60, 210))
            p.setPen(Qt.NoPen)
            lw = car_w * 0.18
            lh = car_h * 0.10
            p.drawRoundedRect(QRectF(x - car_w*0.36, y + car_h*0.38, lw, lh), 6, 6)
            p.drawRoundedRect(QRectF(x + car_w*0.18, y + car_h*0.38, lw, lh), 6, 6)
            if glow:
                p.setBrush(QColor(255, 60, 60, 70))
                p.drawEllipse(QPointF(x - car_w*0.27, y + car_h*0.43), lw*0.9, lh*1.4)
                p.drawEllipse(QPointF(x + car_w*0.27, y + car_h*0.43), lw*0.9, lh*1.4)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()

        # background sky-ish
        bg = QLinearGradient(0, 0, 0, h)
        bg.setColorAt(0.0, QColor(10, 14, 20))
        bg.setColorAt(1.0, QColor(6, 8, 12))
        p.fillRect(0, 0, w, h, bg)

        self._draw_road(p, w, h)

        road_w = min(w * 0.72, w - 40)
        road_x = (w - road_w) * 0.5
        lane_center_x = road_x + road_w * 0.5

        # radar cone
        cone = QPainterPath()
        ego_y = h * 0.82
        ego_x = lane_center_x
        cone.moveTo(ego_x, ego_y - 20)
        cone.arcTo(QRectF(ego_x - road_w*0.38, ego_y - road_w*0.70, road_w*0.76, road_w*0.76), 42, 96)
        cone.closeSubpath()
        p.setBrush(QColor(53, 209, 255, 28))
        p.setPen(QPen(QColor(53, 209, 255, 90), 2))
        p.drawPath(cone)

        # lead car position mapping
        car_w = road_w * 0.11
        car_h = car_w * 1.60

        # ego car (fixed)
        self._draw_car(
            p, ego_x, ego_y, car_w, car_h,
            QColor(30, 140, 255, 210), QColor(10, 40, 80, 240),
            brake_lights=self.brake_lights, glow=True
        )

        if self.lead_present:
            # Map distance to screen: closer -> lower; far -> near top
            # keep minimum spacing so cars don't overlap visually
            d = clamp(self.dist_m, 12.0, 200.0)
            t = 1.0 - (d / 200.0)
            lead_y = (h * 0.12) + (h * 0.55) * t
            lead_x = ego_x

            self._draw_car(
                p, lead_x, lead_y, car_w, car_h,
                QColor(180, 185, 195, 210), QColor(80, 85, 95, 240),
                brake_lights=False, glow=False
            )

            # overlay: distance + relative speed
            p.setPen(QColor(234, 242, 255, 220))
            p.setFont(QFont("Segoe UI", 10, QFont.Weight.DemiBold))
            box = QRectF(lead_x - 120, lead_y - car_h*0.80, 240, 36)
            p.setBrush(QColor(0, 0, 0, 110))
            p.setPen(QPen(QColor(255, 255, 255, 20), 1))
            p.drawRoundedRect(box, 10, 10)

            p.setPen(QColor(234, 242, 255, 220))
            txt = f"{self.dist_m:0.1f} m   |   Δv {self.rel_kmh:+0.1f} km/h"
            p.drawText(box, Qt.AlignCenter, txt)

        # subtle HUD top-left
        hud = QRectF(16, 16, 220, 62)
        p.setBrush(QColor(0, 0, 0, 110))
        p.setPen(QPen(QColor(255, 255, 255, 18), 1))
        p.drawRoundedRect(hud, 14, 14)
        p.setPen(QColor(234, 242, 255, 220))
        p.setFont(QFont("Segoe UI", 10, QFont.Weight.DemiBold))
        p.drawText(hud.adjusted(14, 10, -14, -10), Qt.AlignLeft | Qt.AlignTop, "RADAR VIEW")
        p.setPen(QColor(215, 224, 234, 170))
        p.setFont(QFont("Segoe UI", 9))
        p.drawText(hud.adjusted(14, 30, -14, -10), Qt.AlignLeft | Qt.AlignTop, "Top-down scene • stable fixed-step physics")


# ============================================================
# --- Dashboard View
# ============================================================

class DashboardView(QWidget):
    def __init__(self, core: SimCore, parent=None):
        super().__init__(parent)
        self.core = core
        self._last_ui = time.time()

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        # Top control row: dropdowns (Simulation Type + Scenario Type)
        top = QHBoxLayout()
        top.setSpacing(10)

        self.cmb_sim = QComboBox()
        self.cmb_sim.addItems([
            "Free Drive",
            "Constant Lead Follow",
            "Stop-and-Go",
            "Cut-In",
            "Lead Hard Brake",
            "Traffic Jam",
        ])

        self.btn_run = QPushButton("Run Scenario")
        self.btn_run.setObjectName("Primary")

        top.addWidget(QLabel("Simulation Type:"))
        top.addWidget(self.cmb_sim, 1)
        top.addWidget(self.btn_run)

        root.addLayout(top)

        # Main layout: left instruments, center road, right indicators+controls
        main = QHBoxLayout()
        main.setSpacing(12)

        # Left: speed gauge card
        c1 = Card("Speed")
        c1l = QVBoxLayout(c1.body)
        c1l.setContentsMargins(0, 0, 0, 0)
        c1l.setSpacing(10)
        self.gauge = SpeedGauge()
        self.lbl_speed = QLabel("0 km/h")
        self.lbl_speed.setObjectName("Subtle")
        self.lbl_speed.setAlignment(Qt.AlignHCenter)
        c1l.addWidget(self.gauge, 1, Qt.AlignHCenter)
        c1l.addWidget(self.lbl_speed)

        # Left: ACC status card
        c2 = Card("ACC Status")
        c2l = QVBoxLayout(c2.body)
        c2l.setContentsMargins(0, 0, 0, 0)
        c2l.setSpacing(10)
        self.lbl_state = QLabel("ACC_OFF")
        self.lbl_state.setFont(QFont("Segoe UI", 13, QFont.Weight.Bold))
        self.lbl_state.setAlignment(Qt.AlignHCenter)
        self.lbl_state.setFixedWidth(240)
        self.lbl_state.setFixedHeight(38)
        self.lbl_state.setStyleSheet("""
            color: rgba(234,242,255,0.92);
            padding: 6px 10px;
            border-radius: 10px;
            background: rgba(255,255,255,0.05);
        """)
        self.lbl_set = QLabel("Set speed: --")
        self.lbl_gap = QLabel("Time gap: --")
        self.lbl_msg = QLabel("")
        self.lbl_msg.setWordWrap(True)
        self.lbl_msg.setStyleSheet("color: rgba(53,209,255,0.85);")
        c2l.addWidget(self.lbl_state)
        c2l.addWidget(self.lbl_set)
        c2l.addWidget(self.lbl_gap)
        c2l.addSpacing(8)
        c2l.addWidget(self.lbl_msg, 1)

        # Center: road view
        c_road = Card("Simulation")
        road_l = QVBoxLayout(c_road.body)
        road_l.setContentsMargins(0, 0, 0, 0)
        road_l.setSpacing(0)
        self.road = RoadView()
        c_road.setMinimumHeight(720)
        c_road.setSizePolicy(c_road.sizePolicy().horizontalPolicy(), QSizePolicy.Expanding)
        road_l.addWidget(self.road, 1)

        # Right: throttle/brake indicators
        c3 = Card("Actuation")
        c3l = QVBoxLayout(c3.body)
        c3l.setContentsMargins(0, 0, 0, 0)
        c3l.setSpacing(10)
        self.bar_thr = BarIndicator("Throttle (Gas pedal)", QColor(53, 209, 255))
        self.bar_brk = BarIndicator("Brake request", QColor(255, 80, 80))
        self.lbl_bl = QLabel("Brake lights: OFF")
        self.lbl_bl.setObjectName("Subtle")
        c3l.addWidget(self.bar_thr)
        c3l.addWidget(self.bar_brk)
        c3l.addWidget(self.lbl_bl)

        # Right: driver inputs / buttons
        c4 = Card("Driver Controls (CAN)")
        c4l = QGridLayout(c4.body)
        c4l.setContentsMargins(0, 0, 0, 0)
        c4l.setHorizontalSpacing(8)
        c4l.setVerticalSpacing(8)

        def mk_btn(text, primary=False):
            b = QPushButton(text)
            if primary:
                b.setObjectName("Primary")
            return b

        self.btn_on = mk_btn("ON")
        self.btn_off = mk_btn("OFF")
        self.btn_set = mk_btn("SET+")
        self.btn_coast = mk_btn("COAST")
        self.btn_res = mk_btn("RESUME")
        self.btn_tgp = mk_btn("TG+")
        self.btn_tgm = mk_btn("TG-")

        c4l.addWidget(self.btn_on,   0, 0)
        c4l.addWidget(self.btn_off,  0, 1)
        c4l.addWidget(self.btn_set,  1, 0)
        c4l.addWidget(self.btn_coast,1, 1)
        c4l.addWidget(self.btn_res,  2, 0)
        c4l.addWidget(self.btn_tgp,  2, 1)
        c4l.addWidget(self.btn_tgm,  3, 1)

        self.chk_bs1 = QCheckBox("BS1")
        self.chk_bs2 = QCheckBox("BS2")
        c4l.addWidget(self.chk_bs1, 3, 0)
        c4l.addWidget(self.chk_bs2, 4, 0)

        c4l.addWidget(QLabel("Driver throttle"), 4, 1)
        self.sld_thr = QSlider(Qt.Horizontal)
        self.sld_thr.setRange(0, 100)
        self.sld_thr.setValue(0)
        c4l.addWidget(self.sld_thr, 5, 0, 1, 2)

        c4l.addWidget(QLabel("Driver brake"), 6, 1)
        self.sld_brk = QSlider(Qt.Horizontal)
        self.sld_brk.setRange(0, 100)
        self.sld_brk.setValue(0)
        c4l.addWidget(self.sld_brk, 7, 0, 1, 2)

        # Right: lead controls
        c5 = Card("Lead Vehicle (CAN)")
        c5l = QGridLayout(c5.body)
        c5l.setContentsMargins(0, 0, 0, 0)
        c5l.setHorizontalSpacing(8)
        c5l.setVerticalSpacing(8)

        self.chk_lead = QCheckBox("Lead present")
        self.spn_lead_speed = QDoubleSpinBox()
        self.spn_lead_speed.setRange(0.0, 180.0)
        self.spn_lead_speed.setSingleStep(5.0)
        self.spn_lead_speed.setValue(70.0)

        self.spn_lead_dist = QDoubleSpinBox()
        self.spn_lead_dist.setRange(5.0, 140.0)
        self.spn_lead_dist.setSingleStep(5.0)
        self.spn_lead_dist.setValue(60.0)

        c5l.addWidget(self.chk_lead, 0, 0, 1, 2)
        c5l.addWidget(QLabel("Lead speed (km/h)"), 1, 0)
        c5l.addWidget(self.spn_lead_speed, 1, 1)
        c5l.addWidget(QLabel("Initial distance (m)"), 2, 0)
        c5l.addWidget(self.spn_lead_dist, 2, 1)

        # Placement using columns so the center road can dominate height
        left_col = QVBoxLayout()
        left_col.setSpacing(10)
        left_col.addWidget(c1)
        left_col.addWidget(c2)
        left_col.addStretch(1)

        right_col = QVBoxLayout()
        right_col.setSpacing(10)
        right_col.addWidget(c3)
        right_col.addWidget(c4)
        right_col.addWidget(c5)
        right_col.addStretch(1)

        main.addLayout(left_col, 0)
        main.addWidget(c_road, 1)
        main.addLayout(right_col, 0)
        main.setStretchFactor(c_road, 3)
        main.setStretch(0, 0)
        main.setStretch(2, 0)

        root.addLayout(main, 1)

        # Wire controls -> ECUs
        self.btn_on.clicked.connect(lambda: self.core.driver.pulse(BTN_ON))
        self.btn_off.clicked.connect(lambda: self.core.driver.pulse(BTN_OFF))
        self.btn_set.clicked.connect(lambda: self.core.driver.pulse(BTN_SET_PLUS))
        self.btn_coast.clicked.connect(lambda: self.core.driver.pulse(BTN_COAST))
        self.btn_res.clicked.connect(lambda: self.core.driver.pulse(BTN_RESUME))
        self.btn_tgp.clicked.connect(lambda: self.core.driver.pulse(BTN_TG_PLUS))
        self.btn_tgm.clicked.connect(lambda: self.core.driver.pulse(BTN_TG_MINUS))

        self.chk_bs1.toggled.connect(self._update_bs)
        self.chk_bs2.toggled.connect(self._update_bs)
        self.sld_thr.valueChanged.connect(self._update_pedals)
        self.sld_brk.valueChanged.connect(self._update_pedals)

        self.cmb_sim.currentIndexChanged.connect(self._update_sim_type)
        self.chk_lead.toggled.connect(self._update_lead_ctrl)
        self.spn_lead_speed.valueChanged.connect(self._update_lead_ctrl)
        self.spn_lead_dist.valueChanged.connect(self._update_lead_ctrl)

        self.btn_run.clicked.connect(self._run_selected_scenario)

        self._update_sim_type()
        self._update_lead_ctrl()

        # Update loop
        self.core.updated.connect(self.on_core_updated)

    def _update_bs(self):
        self.core.driver.set_bs(self.chk_bs1.isChecked(), self.chk_bs2.isChecked())

    def _update_pedals(self):
        self.core.driver.set_pedals(self.sld_thr.value(), self.sld_brk.value())

        # Optional realism: if brake pedal is pressed, BS1+BS2 typically go high.
        if self.sld_brk.value() > 2:
            self.chk_bs1.setChecked(True)
            self.chk_bs2.setChecked(True)

    def _update_sim_type(self):
        idx = self.cmb_sim.currentIndex()
        self.core.leadctrl.sim_type = SimType(idx)

    def _update_lead_ctrl(self):
        self.core.leadctrl.lead_present = self.chk_lead.isChecked()
        self.core.leadctrl.lead_speed_kmh = float(self.spn_lead_speed.value())
        self.core.leadctrl.lead_distance_m = float(self.spn_lead_dist.value())

    def on_core_updated(self):
        now = time.time()
        dt = clamp(now - self._last_ui, 0.0, 0.05)
        self._last_ui = now

        c = self.core.ui_cache
        speed = c["speed_kmh"]
        self.gauge.set_speed(speed, dt)
        self.lbl_speed.setText(f"{speed:0.1f} km/h")

        st = AccState(int(c["acc_state"]))
        state_colors = {
            AccState.ACC_OFF: "#9ca3af",
            AccState.ACC_STANDBY: "#fbbf24",
            AccState.ACC_ACTIVE_SPEED_CONTROL: "#22c55e",
            AccState.ACC_ACTIVE_TIME_GAP_CONTROL: "#22c55e",
            AccState.DRIVER_INTERVENTION_REQUIRED: "#ef4444",
        }
        col = state_colors.get(st, "#eaf2ff")
        friendly = {
            AccState.ACC_OFF: "OFF",
            AccState.ACC_STANDBY: "STANDBY",
            AccState.ACC_ACTIVE_SPEED_CONTROL: "ACTIVE SPEED",
            AccState.ACC_ACTIVE_TIME_GAP_CONTROL: "ACTIVE GAP",
            AccState.DRIVER_INTERVENTION_REQUIRED: "INTERVENTION",
        }.get(st, st.name)
        self.lbl_state.setText(friendly)
        self.lbl_state.setStyleSheet(f"""
            color: {col};
            padding: 6px 10px;
            border-radius: 10px;
            background: rgba(255,255,255,0.06);
            border: 1px solid rgba(255,255,255,0.08);
        """)
        self.lbl_set.setText(f"Set speed: {c['set_speed_kmh']:.0f} km/h" if c["set_speed_kmh"] > 0 else "Set speed: --")
        self.lbl_gap.setText(f"Time gap: {c['time_gap_s']:.1f} s")
        self.lbl_msg.setText(c["msg"])

        self.bar_thr.set_value(c["throttle_pct"], dt)
        self.bar_brk.set_value(c["brake_pct"], dt)
        self.lbl_bl.setText("Brake lights: ON" if c["brake_lights"] else "Brake lights: OFF")

        self.road.update_signals(
            speed_kmh=speed,
            lead_present=c["lead_present"],
            dist_m=c["dist_m"],
            rel_kmh=c["rel_speed_kmh"],
            brake_lights=c["brake_lights"],
            dt=dt
        )

    # ---------------- Scenario binding (simple, visual scripts) ----------------

    def _run_selected_scenario(self):
        idx = self.cmb_sim.currentIndex()
        main = self.window()
        if hasattr(main, "run_dashboard_scenario"):
            main.run_dashboard_scenario(idx)


# ============================================================
# --- CAN Monitor View
# ============================================================

class CANMonitorView(QWidget):
    def __init__(self, core: SimCore, parent=None):
        super().__init__(parent)
        self.core = core
        self._paused = False
        self._last_seq = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        top = QHBoxLayout()
        self.ed_filter = QLineEdit()
        self.ed_filter.setPlaceholderText("Filter (e.g. 0x130 or ACC_STATUS or Brake)")
        self.btn_pause = QPushButton("Pause")
        self.btn_clear = QPushButton("Clear")
        top.addWidget(QLabel("CAN Filter:"))
        top.addWidget(self.ed_filter, 1)
        top.addWidget(self.btn_pause)
        top.addWidget(self.btn_clear)
        root.addLayout(top)

        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["t (s)", "ID", "Name", "Src", "Data (hex)", "Decoded", "Seq"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(6, QHeaderView.ResizeToContents)
        self.table.setAlternatingRowColors(True)
        root.addWidget(self.table, 1)

        inj = Card("Frame Injection")
        inj_l = QGridLayout(inj.body)
        inj_l.setContentsMargins(0, 0, 0, 0)
        inj_l.setHorizontalSpacing(8)
        inj_l.setVerticalSpacing(8)

        self.cmb_inj = QComboBox()
        self.cmb_inj.addItems(list(MSG_NAMES.values()))
        self.ed_inj_id = QLineEdit("0x130")
        self.ed_inj_data = QLineEdit("00 00 00 00 00 00 00 00")
        self.btn_send = QPushButton("Send Injected Frame")
        self.btn_send.setObjectName("Primary")

        inj_l.addWidget(QLabel("Message"), 0, 0)
        inj_l.addWidget(self.cmb_inj, 0, 1)
        inj_l.addWidget(QLabel("Arb ID (hex)"), 1, 0)
        inj_l.addWidget(self.ed_inj_id, 1, 1)
        inj_l.addWidget(QLabel("Data (8 bytes hex)"), 2, 0)
        inj_l.addWidget(self.ed_inj_data, 2, 1)
        inj_l.addWidget(self.btn_send, 3, 0, 1, 2)

        root.addWidget(inj)

        self.btn_pause.clicked.connect(self._toggle_pause)
        self.btn_clear.clicked.connect(self._clear)
        self.btn_send.clicked.connect(self._inject_send)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(CAN_MONITOR_MS)

    def showEvent(self, e):
        super().showEvent(e)
        self._paused = False
        self.btn_pause.setText("Pause")
        self._last_seq = 0
        self._tick()

    def refresh_now(self):
        self._paused = False
        self.btn_pause.setText("Pause")
        self.table.setRowCount(0)
        self._last_seq = 0
        self._tick()

    def _toggle_pause(self):
        self._paused = not self._paused
        self.btn_pause.setText("Resume" if self._paused else "Pause")

    def _clear(self):
        self.table.setRowCount(0)
        self._last_seq = self.core.bus.last_seq()

    def _inject_send(self):
        try:
            s = self.ed_inj_id.text().strip().lower()
            arb = int(s, 16) if s.startswith("0x") else int(s)
            parts = self.ed_inj_data.text().strip().split()
            data = bytes(int(x, 16) for x in parts)
            data = data[:8].ljust(8, b"\x00")
            self.core.bus.send(arb, data, src="INJECT")
        except Exception as ex:
            QMessageBox.warning(self, "Injection error", f"Invalid frame: {ex}")

    def _match_filter(self, fr: CANFrame, f: str) -> bool:
        if not f:
            return True
        f = f.lower()
        if f.startswith("0x"):
            try:
                want = int(f, 16)
                return fr.arb_id == want
            except Exception:
                pass
        if f in fr.name.lower():
            return True
        if f in fr.src.lower():
            return True
        if f in f"{fr.arb_id:#x}":
            return True
        return False

    def _tick(self):
        if self._paused:
            return
        self._last_seq, frames = self.core.bus.consume_since(self._last_seq)
        if not frames:
            return

        filt = self.ed_filter.text().strip()
        t0 = frames[0].ts if frames else time.time()

        for fr in frames[-250:]:
            if not self._match_filter(fr, filt):
                continue
            decoded = decode_frame(fr.arb_id, fr.data)
            decoded_str = ""
            if decoded:
                # keep concise
                items = []
                for k, v in decoded.items():
                    if isinstance(v, float):
                        items.append(f"{k}={v:.2f}")
                    else:
                        items.append(f"{k}={v}")
                decoded_str = ", ".join(items)

            r = self.table.rowCount()
            self.table.insertRow(r)
            self.table.setItem(r, 0, QTableWidgetItem(f"{fr.ts - t0:0.3f}"))
            self.table.setItem(r, 1, QTableWidgetItem(f"{fr.arb_id:#04x}"))
            self.table.setItem(r, 2, QTableWidgetItem(fr.name))
            self.table.setItem(r, 3, QTableWidgetItem(fr.src))
            self.table.setItem(r, 4, QTableWidgetItem(" ".join(f"{b:02X}" for b in fr.data)))
            self.table.setItem(r, 5, QTableWidgetItem(decoded_str))
            self.table.setItem(r, 6, QTableWidgetItem(str(fr.seq)))

        if self.table.rowCount() > 1200:
            overflow = self.table.rowCount() - 1200
            for _ in range(overflow):
                self.table.removeRow(0)
        if frames:
            self.table.scrollToBottom()


# ============================================================
# --- Scenario Explorer View + Scenario Runner
# ============================================================

@dataclass
class ScenarioDef:
    key: str
    group: str
    title: str
    description: str
    reset: Callable[[], None]
    tick: Callable[[float, float], None]
    duration: float = 20.0

class ScenarioRunner(QObject):
    finished = Signal()

    def __init__(self, core: SimCore):
        super().__init__()
        self.core = core
        self._active = False
        self._t0 = 0.0
        self._last_t = 0.0
        self._scn: Optional[ScenarioDef] = None
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._auto_engage = True
        self._engage_last = 0.0
        self._engage_timeout = 8.0

    def is_active(self): return self._active

    def stop(self):
        self._active = False
        self._timer.stop()
        self._scn = None
        self.core.driver.set_pedals(0, 0)
        self.core.driver.set_bs(False, False)
        self.finished.emit()

    def start(self, scn: ScenarioDef):
        self.stop()
        self._scn = scn
        try:
            scn.reset()
        except Exception:
            pass
        self._active = True
        self._t0 = time.time()
        self._last_t = self._t0
        self._auto_engage = True
        self._engage_last = 0.0
        self._timer.start(SCENARIO_TICK_MS)

    def _maybe_engage_acc(self, now: float):
        if not self._auto_engage or not self._scn:
            return
        # avoid spamming if below activation speed
        ego_speed_kmh = mps_to_kmh(self.core.plant.state.v_mps)
        if ego_speed_kmh < 40.0:
            return
        st = self.core.acc.state
        if st in (AccState.ACC_ACTIVE_SPEED_CONTROL, AccState.ACC_ACTIVE_TIME_GAP_CONTROL):
            self._auto_engage = False
            return
        if (now - self._engage_last) > 0.6:
            self.core.driver.pulse(BTN_ON)
            self.core.driver.pulse(BTN_SET_PLUS)
            self._engage_last = now
        if (now - self._t0) > self._engage_timeout:
            self._auto_engage = False

    def _tick(self):
        if not (self._active and self._scn):
            return
        now = time.time()
        t = now - self._t0
        dt = clamp(now - self._last_t, 0.0, 0.25)
        self._last_t = now

        self._maybe_engage_acc(now)

        try:
            self._scn.tick(t, dt)
        except Exception:
            pass

        if self._scn.duration and t > (self._scn.duration + 2.0):
            self.stop()

class ScenarioExplorerView(QWidget):
    def __init__(self, core: SimCore, scenarios: List[ScenarioDef], runner: ScenarioRunner, parent=None):
        super().__init__(parent)
        self.core = core
        self.scenarios = scenarios
        self.runner = runner
        self._selected: Optional[ScenarioDef] = None

        root = QHBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        left = Card("Scenario Library")
        left_l = QVBoxLayout(left.body)
        left_l.setContentsMargins(0, 0, 0, 0)
        left_l.setSpacing(8)

        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        left_l.addWidget(self.tree, 1)

        right = QVBoxLayout()
        right.setSpacing(12)

        det = Card("Scenario Details")
        det_l = QVBoxLayout(det.body)
        det_l.setContentsMargins(0, 0, 0, 0)
        det_l.setSpacing(10)

        self.lbl_title = QLabel("Select a scenario")
        self.lbl_title.setFont(QFont("Segoe UI", 16, QFont.Weight.Bold))
        self.txt_desc = QTextEdit()
        self.txt_desc.setReadOnly(True)
        det_l.addWidget(self.lbl_title)
        det_l.addWidget(self.txt_desc, 1)

        ctl = Card("Runner")
        ctl_l = QHBoxLayout(ctl.body)
        ctl_l.setContentsMargins(0, 0, 0, 0)
        self.btn_run = QPushButton("Run")
        self.btn_run.setObjectName("Primary")
        self.btn_stop = QPushButton("Stop")
        self.lbl_status = QLabel("Idle")
        self.lbl_status.setObjectName("Subtle")
        ctl_l.addWidget(self.btn_run)
        ctl_l.addWidget(self.btn_stop)
        ctl_l.addWidget(self.lbl_status, 1)

        right.addWidget(det, 1)
        right.addWidget(ctl, 0)

        root.addWidget(left, 0)
        root.addLayout(right, 1)

        self._populate_tree()
        self.tree.itemSelectionChanged.connect(self._on_select)
        self.btn_run.clicked.connect(self._run_selected)
        self.btn_stop.clicked.connect(self.runner.stop)
        self.runner.finished.connect(lambda: self.lbl_status.setText("Idle"))

    def _populate_tree(self):
        by_group: Dict[str, List[ScenarioDef]] = {}
        for s in self.scenarios:
            by_group.setdefault(s.group, []).append(s)

        for grp, items in by_group.items():
            gitem = QTreeWidgetItem([grp])
            gitem.setExpanded(True)
            self.tree.addTopLevelItem(gitem)
            for scn in items:
                it = QTreeWidgetItem([scn.title])
                it.setData(0, Qt.UserRole, scn.key)
                gitem.addChild(it)

    def _on_select(self):
        items = self.tree.selectedItems()
        if not items:
            return
        it = items[0]
        key = it.data(0, Qt.UserRole)
        if not key:
            return
        scn = next((s for s in self.scenarios if s.key == key), None)
        if not scn:
            return
        self._selected = scn
        self.lbl_title.setText(scn.title)
        self.txt_desc.setPlainText(
            f"{scn.description}\n\n"
            f"Group: {scn.group}\n"
            f"Estimated duration: ~{scn.duration:.0f} s\n"
            f"ACC auto-engages when ego speed is >= 40 km/h."
        )

    def _run_selected(self):
        if not self._selected:
            return
        self.lbl_status.setText("Running…")
        self.runner.start(self._selected)


# ============================================================
# --- Main Window with Navigation + Transitions
# ============================================================

class NavButton(QPushButton):
    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setCheckable(True)
        self.setMinimumHeight(44)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ACC Simulator — Premium HMI (PySide6)")
        self.setMinimumSize(1280, 760)

        self.core = SimCore()

        # Central layout: left nav + stacked views
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(12)

        nav = QFrame()
        nav.setObjectName("Card")
        nav_l = QVBoxLayout(nav)
        nav_l.setContentsMargins(14, 14, 14, 14)
        nav_l.setSpacing(10)

        title = QLabel("ACC HMI")
        title.setFont(QFont("Segoe UI", 18, QFont.Weight.Bold))
        title.setStyleSheet("color: rgba(234,242,255,0.95);")
        sub = QLabel("ADAS / CAN / HMI demo")
        sub.setObjectName("Subtle")
        nav_l.addWidget(title)
        nav_l.addWidget(sub)
        nav_l.addSpacing(12)

        self.btn_dash = NavButton("Simulation Dashboard")
        self.btn_can  = NavButton("CAN Monitor")
        nav_l.addWidget(self.btn_dash)
        nav_l.addWidget(self.btn_can)
        nav_l.addStretch(1)

        hint = QLabel("Tip: pick a Simulation Type and hit Run to auto-engage ACC; open CAN Monitor to see frames.")
        hint.setWordWrap(True)
        hint.setObjectName("Subtle")
        nav_l.addWidget(hint)

        make_shadow(nav)

        self.stack = QStackedWidget()
        self.view_dash = DashboardView(self.core, parent=self)
        self.view_can  = CANMonitorView(self.core, parent=self)
        self.stack.addWidget(self.view_dash)
        self.stack.addWidget(self.view_can)

        root.addWidget(nav, 0)
        root.addWidget(self.stack, 1)

        self.btn_dash.clicked.connect(lambda: self._switch(0))
        self.btn_can.clicked.connect(lambda: self._switch(1))

        self._switch(0, animate=False)

        # Global UI update timer (fixed-step handled inside core)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.core.step_real_time)
        self.timer.start(UI_FRAME_MS)

        # Apply theme
        self.setStyleSheet(DARK_QSS)

        # init environment
        self.core.reset_world(ego_speed_kmh=0.0, lead_present=False, lead_speed_kmh=70.0, lead_dist_m=60.0)

    def _switch(self, idx: int, animate: bool = True):
        idx = 0 if idx not in (0, 1) else idx
        for b in (self.btn_dash, self.btn_can):
            b.setChecked(False)
        [self.btn_dash, self.btn_can][idx].setChecked(True)

        # Keep it robust: switch immediately, no fade animations
        self.stack.setCurrentIndex(idx)
        if idx == 1:
            self.view_can.refresh_now()

    # Called by DashboardView when "Run Scenario" pressed
    def run_dashboard_scenario(self, idx: int):
        presets = {
            SimType.FREE_DRIVE:            (90.0, False, 0.0, 70.0),
            SimType.CONSTANT_LEAD_FOLLOW:  (90.0, True,  70.0, 70.0),
            SimType.STOP_AND_GO:           (65.0, True,  45.0, 50.0),
            SimType.CUT_IN:                (95.0, False, 94.0, 40.0),  # lead will spawn after 2s
            SimType.LEAD_HARD_BRAKE:       (95.0, True,  90.0, 60.0),
            SimType.TRAFFIC_JAM:           (80.0, True,  50.0, 55.0),
        }
        st = SimType(idx) if idx in range(len(SimType)) else SimType.FREE_DRIVE
        ego_kmh, lead_present, lead_spd, lead_dist = presets.get(st, presets[SimType.FREE_DRIVE])

        self.core.leadctrl.sim_type = st
        self.core.leadctrl.lead_present = lead_present
        self.core.leadctrl.lead_speed_kmh = lead_spd
        self.core.leadctrl.lead_distance_m = lead_dist
        self.core.driver.set_bs(False, False)
        self.core.driver.set_pedals(0, 0)
        self.core.reset_world(ego_kmh, lead_present, lead_spd, lead_dist)

        # Auto-engage ACC when above activation speed so button immediately does something visible.
        if ego_kmh >= 40.0:
            self.core.driver.pulse(BTN_ON)
            self.core.driver.pulse(BTN_SET_PLUS)

        self._switch(0, animate=False)

    # ---------------- Scenario definitions ----------------

    def _build_scenarios(self) -> List[ScenarioDef]:
        c = self.core

        def reset_env(ego_kmh: float, lead_present: bool, lead_kmh: float, lead_dist: float):
            def _():
                c.leadctrl.sim_type = SimType.FREE_DRIVE
                c.leadctrl.lead_present = lead_present
                c.leadctrl.lead_speed_kmh = float(lead_kmh)
                c.leadctrl.lead_distance_m = float(lead_dist)
                c.driver.set_bs(False, False)
                c.driver.set_pedals(0, 0)
                c.reset_world(ego_kmh, lead_present, lead_kmh, lead_dist)
            return _

        def spawn_lead(distance_m: float, speed_kmh: float):
            c.leadctrl.lead_present = True
            c.leadctrl.lead_distance_m = float(distance_m)
            c.leadctrl.lead_speed_kmh = float(speed_kmh)
            c.lead.reset(True, distance_m, speed_kmh, c.plant.state.pos_m)

        def remove_lead():
            c.leadctrl.lead_present = False
            c.lead.present = False

        def set_lead_speed(speed_kmh: float):
            c.leadctrl.lead_speed_kmh = float(speed_kmh)

        scenarios: List[ScenarioDef] = []

        # 1) Follow vehicle
        def tick_follow(t, dt):
            set_lead_speed(72.0 + 2.5 * math.sin(t * 0.25))
        scenarios.append(ScenarioDef(
            key='follow',
            group='Core Driving',
            title='Follow vehicle',
            description='Lead cruising ahead at ~85 m with slight speed variation to exercise gap keeping.',
            reset=reset_env(90.0, True, 72.0, 85.0),
            tick=tick_follow,
            duration=18.0,
        ))

        # 2) Free road
        def tick_free(t, dt):
            remove_lead()
        scenarios.append(ScenarioDef(
            key='free',
            group='Core Driving',
            title='Free road (no target)',
            description='Ego runs speed-control with no radar target in lane.',
            reset=reset_env(95.0, False, 0.0, 80.0),
            tick=tick_free,
            duration=16.0,
        ))

        # 3) Cut-in (left/right)
        def make_cutin(key: str, title: str, dist: float):
            spawned = False
            def reset():
                reset_env(97.0, False, 94.0, 120.0)()
            def tick(t, dt):
                nonlocal spawned
                if (not spawned) and t >= 6.0:
                    spawn_lead(dist, 94.0)
                    spawned = True
                if spawned:
                    set_lead_speed(90.0)
            return ScenarioDef(
                key=key,
                group='Cut-in',
                title=title,
                description='Vehicle in adjacent lane cuts in near 35-40 m ahead at ~6 s.',
                reset=reset,
                tick=tick,
                duration=16.0,
            )
        scenarios.append(make_cutin('cutin_left', 'Cut-in from LEFT', 40.0))
        scenarios.append(make_cutin('cutin_right', 'Cut-in from RIGHT', 38.0))

        # 4) Lead sudden brake
        def tick_sudden_brake(t, dt):
            if 4.0 <= t <= 6.0:
                set_lead_speed(35.0)
            elif t > 6.0:
                set_lead_speed(95.0)
        scenarios.append(ScenarioDef(
            key='sudden_brake',
            group='Safety',
            title='Lead sudden brake',
            description='Lead decelerates hard between 4-6 s then recovers; checks intervention handling.',
            reset=reset_env(100.0, True, 95.0, 70.0),
            tick=tick_sudden_brake,
            duration=18.0,
        ))

        # 5) Stop & Go
        def tick_stop_go(t, dt):
            if 3.0 <= t < 7.0:
                set_lead_speed(28.0)
            elif 9.0 <= t < 13.0:
                set_lead_speed(70.0)
            else:
                set_lead_speed(50.0)
        scenarios.append(ScenarioDef(
            key='stop_go',
            group='Traffic',
            title='Stop & Go wave',
            description='Lead slows nearly to crawl then accelerates again, repeating once.',
            reset=reset_env(70.0, True, 60.0, 60.0),
            tick=tick_stop_go,
            duration=18.0,
        ))

        # 6) Lead disappears
        def tick_disappear(t, dt):
            if 8.0 <= t < 8.6:
                remove_lead()
        scenarios.append(ScenarioDef(
            key='disappear',
            group='Traffic',
            title='Lead disappears',
            description='Lead leaves lane around 8 s to test ACC handoff back to speed control.',
            reset=reset_env(90.0, True, 80.0, 85.0),
            tick=tick_disappear,
            duration=16.0,
        ))

        # 7) Overtaking maneuver
        def reset_overtake():
            reset_env(95.0, True, 75.0, 35.0)()
        def tick_overtake(t, dt):
            if 2.0 <= t < 2.3:
                remove_lead()
                c.driver.pulse(BTN_SET_PLUS)
            if 2.0 < t < 6.0:
                c.driver.set_pedals(18, 0)
            else:
                c.driver.set_pedals(0, 0)
            if 7.0 <= t < 7.4 and not c.lead.present:
                spawn_lead(70.0, 90.0)
        scenarios.append(ScenarioDef(
            key='overtake',
            group='Maneuvers',
            title='Overtaking',
            description='Ego accelerates to pass (lead removed), then a lead reappears farther ahead.',
            reset=reset_overtake,
            tick=tick_overtake,
            duration=20.0,
        ))

        # 8) Traffic jam waves
        def tick_traffic_jam(t, dt):
            base = 32.0
            wave = 16.0 * (0.5 + 0.5 * math.sin(t * 0.9))
            set_lead_speed(base + wave)
        scenarios.append(ScenarioDef(
            key='traffic_jam',
            group='Traffic',
            title='Traffic jam waves',
            description='Lead speed oscillates to emulate slow-moving congestion without leaving the lane.',
            reset=reset_env(80.0, True, 50.0, 55.0),
            tick=tick_traffic_jam,
            duration=24.0,
        ))

        return scenarios


# ============================================================
# --- Entry Point
# ============================================================

def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
