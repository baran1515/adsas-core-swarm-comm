#!/usr/bin/env python3
"""Bridge between ArduPilot SITL vehicles and A.D.S.A.S. nodes.

For every vehicle i (0-based):
  * MAVLink:   tcp:127.0.0.1:5760+10*i   (sim_vehicle.py -I i --no-mavproxy)
  * A.D.S.A.S. node (id=i+1) listens on UDP 127.0.0.1:<node_port_base>+i+1

Data flow
  SITL --GLOBAL_POSITION_INT--> bridge --"POS x y alt"--> node
  node --"ROUTE epoch state done total x,y ..."--> bridge --GUIDED goto--> SITL

The node decides when a cell is reached and which cells it owns; the bridge
only converts coordinates and commands the vehicle. If a node stops sending
ROUTE messages (node lost), the vehicle is switched to LAND.

requires: pip install pymavlink
"""
import argparse
import math
import socket
import time

from pymavlink import mavutil

EARTH_M_PER_DEG = 111319.5
STATE_RTL = 2
POS_ONLY_MASK = 0b110111111000  # use x/y/z position fields only


class Vehicle:
    def __init__(self, idx, args):
        self.idx = idx
        self.node_id = idx + 1
        self.args = args
        url = f"tcp:127.0.0.1:{5760 + 10 * idx}"
        print(f"[V{self.node_id}] connecting {url}")
        self.mav = mavutil.mavlink_connection(url, source_system=250 + idx)
        self.mav.wait_heartbeat(timeout=60)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.node_addr = ("127.0.0.1", args.node_port_base + self.node_id)
        self.pos = None
        self.last_route_t = time.time()
        self.target = None
        self.mode = None

    # -- coordinate conversion (flat-earth, fine for a few km) --------------
    def to_local_dm(self, lat, lon):
        lat0, lon0 = self.args.origin
        x = (lon - lon0) * EARTH_M_PER_DEG * math.cos(math.radians(lat0))
        y = (lat - lat0) * EARTH_M_PER_DEG
        return int(x * 10), int(y * 10)

    def to_global(self, x_dm, y_dm):
        lat0, lon0 = self.args.origin
        lat = lat0 + (y_dm / 10) / EARTH_M_PER_DEG
        lon = lon0 + (x_dm / 10) / (EARTH_M_PER_DEG * math.cos(math.radians(lat0)))
        return lat, lon

    # -- vehicle commands ----------------------------------------------------
    def set_mode(self, mode):
        if self.mode != mode:
            self.mav.set_mode(mode)
            self.mode = mode
            print(f"[V{self.node_id}] mode -> {mode}")

    def arm_and_takeoff(self):
        self.set_mode("GUIDED")
        deadline = time.time() + 120
        while not self.mav.motors_armed():
            if time.time() > deadline:
                raise RuntimeError(f"V{self.node_id}: could not arm (EKF/GPS not ready?)")
            self.mav.arducopter_arm()
            self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                       0, 0, 0, 0, 0, 0, 0, self.args.alt)
        print(f"[V{self.node_id}] armed, takeoff to {self.args.alt} m")

    def goto(self, x_dm, y_dm):
        if self.target == (x_dm, y_dm):
            return
        self.target = (x_dm, y_dm)
        lat, lon = self.to_global(x_dm, y_dm)
        self.mav.mav.set_position_target_global_int_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POS_ONLY_MASK,
            int(lat * 1e7), int(lon * 1e7), self.args.alt, 0, 0, 0, 0, 0, 0, 0, 0)

    # -- periodic work -------------------------------------------------------
    def step(self):
        msg = self.mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        while msg is not None:
            self.pos = msg
            msg = self.mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if self.pos is not None:
            x, y = self.to_local_dm(self.pos.lat / 1e7, self.pos.lon / 1e7)
            alt_dm = self.pos.relative_alt // 100  # mm -> dm
            self.sock.sendto(f"POS {x} {y} {alt_dm}".encode(), self.node_addr)

        try:
            while True:
                data, _ = self.sock.recvfrom(2048)
                self.handle_route(data.decode(errors="ignore").split())
        except (BlockingIOError, ConnectionResetError):
            pass

        if time.time() - self.last_route_t > self.args.node_timeout and self.mode != "LAND":
            print(f"[V{self.node_id}] node silent > {self.args.node_timeout}s -> LAND (vehicle lost)")
            self.set_mode("LAND")

    def handle_route(self, parts):
        if len(parts) < 5 or parts[0] != "ROUTE":
            return
        self.last_route_t = time.time()
        state = int(parts[2])
        points = [tuple(map(int, p.split(","))) for p in parts[5:]]
        if state == STATE_RTL:
            self.set_mode("RTL")
        elif points:
            self.goto(*points[0])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--instances", type=int, default=4)
    ap.add_argument("--origin", type=lambda s: tuple(map(float, s.split(","))),
                    default=(-35.363261, 149.165230), help="lat,lon of the area origin")
    ap.add_argument("--alt", type=float, default=20.0, help="mission altitude AGL [m]")
    ap.add_argument("--node-port-base", type=int, default=48000)
    ap.add_argument("--node-timeout", type=float, default=3.0)
    args = ap.parse_args()

    vehicles = [Vehicle(i, args) for i in range(args.instances)]
    for v in vehicles:
        v.arm_and_takeoff()
    time.sleep(8)  # climb before starting the coverage
    for v in vehicles:
        v.last_route_t = time.time()

    print("bridge running - Ctrl+C to stop")
    try:
        while True:
            for v in vehicles:
                v.step()
            time.sleep(0.1)
    except KeyboardInterrupt:
        for v in vehicles:
            v.set_mode("RTL")


if __name__ == "__main__":
    main()
