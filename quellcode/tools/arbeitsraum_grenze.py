#!/usr/bin/env python3
"""Aeussere und innere Reichweitengrenze in EINER Hoehe bestimmen - ohne den Roboter zu bewegen.

Fuer jeden Azimut wird per /compute_ik gesucht, ab welchem Radius der TCP die Ebene
noch erreichen kann (senkrechter Greifer, Finger nach unten). Ergebnis ist der Ring,
den man auf die Platte zeichnen kann: innen zu nah an der eigenen Basis, aussen ausser
Reichweite.

Warum eine feste Hoehe: der Arbeitsraum ist ein Keil - je weiter aussen, desto
niedriger die Decke (siehe messungen_arbeitsraum_2026-09-09.txt). Ein Ring gilt also
immer nur fuer die Hoehe, in der er gemessen wurde.

    python3 tools/arbeitsraum_grenze.py [--z 0.015] [--schritt 15] [--out DATEI]
"""
import argparse, json, math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration

# Senkrechter Griff: tcp +Y -> (0,0,-1) in robot_base.
Q_REF = (-0.583486105248945, 0.3994295494594975, -0.3994295494594975, 0.583486105248945)

def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)

def qz(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))

ap = argparse.ArgumentParser()
ap.add_argument("--z", type=float, default=0.015, help="Hoehe ueber robot_base [m]")
ap.add_argument("--schritt", type=float, default=15.0, help="Azimut-Schrittweite [Grad]")
ap.add_argument("--rmin", type=float, default=60.0)
ap.add_argument("--rmax", type=float, default=320.0)
ap.add_argument("--out", default="arbeitsraum_ring.json")
a = ap.parse_args()

rclpy.init(); node = Node("arbeitsraum_grenze")
cli = node.create_client(GetPositionIK, "/compute_ik")
if not cli.wait_for_service(timeout_sec=15.0):
    print("!! /compute_ik nicht da"); sys.exit(1)

_cache = {}
def erreichbar(r_mm, th):
    """Ist (r, th) in der Hoehe z erreichbar? Zwei Gierwinkel, Kollision beruecksichtigt."""
    key = (round(r_mm, 1), round(th, 1))
    if key in _cache: return _cache[key]
    x = r_mm/1000.0*math.cos(math.radians(th)); y = r_mm/1000.0*math.sin(math.radians(th))
    ok = False
    for dy in (0.0, 90.0):
        q = qmul(qz(th + dy), Q_REF)
        rq = GetPositionIK.Request()
        rq.ik_request.group_name = "arm"; rq.ik_request.ik_link_name = "tcp"
        rq.ik_request.avoid_collisions = True
        rq.ik_request.timeout = Duration(sec=1)
        ps = PoseStamped(); ps.header.frame_id = "robot_base"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, a.z
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        rq.ik_request.pose_stamped = ps
        f = cli.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=8.0)
        if f.result() and f.result().error_code.val == 1:
            ok = True; break
    _cache[key] = ok
    return ok

def bisect(lo, hi, th, ziel_ok_bei_hi):
    """Grenze zwischen lo und hi auf 1 mm einschachteln."""
    for _ in range(8):
        if hi - lo <= 1.0: break
        mid = (lo + hi) / 2.0
        if erreichbar(mid, th) == ziel_ok_bei_hi: hi = mid
        else: lo = mid
    return (lo + hi) / 2.0

t0 = time.time()
ths = [i * a.schritt for i in range(int(round(360.0 / a.schritt)))]
ring = []
print(f"Hoehe z = {a.z*1000:.0f} mm, Azimut-Schritt {a.schritt:.0f} Grad\n")
print("  Azimut   r_innen   r_aussen")
print("  -------  --------  --------")
for th in ths:
    grob = [r for r in range(int(a.rmin), int(a.rmax) + 1, 20) if erreichbar(r, th)]
    if not grob:
        ring.append({"theta": th, "r_min": None, "r_max": None})
        print(f"  {th:6.1f}   ---       ---   (nirgends erreichbar)")
        continue
    lo_ok, hi_ok = grob[0], grob[-1]
    r_in  = bisect(max(a.rmin, lo_ok - 20), lo_ok, th, True)
    r_out = bisect(min(a.rmax, hi_ok + 20), hi_ok, th, True)
    ring.append({"theta": th, "r_min": round(r_in, 1), "r_max": round(r_out, 1)})
    print(f"  {th:6.1f}   {r_in:6.1f}    {r_out:6.1f}")

with open(a.out, "w") as f:
    json.dump({"z": a.z, "frame": "robot_base", "orientierung": "senkrecht (tcp+Y -> -z)",
               "ring": ring, "gemessen": time.strftime("%Y-%m-%d %H:%M")}, f, indent=2)
print(f"\n  {len(_cache)} IK-Abfragen, {time.time()-t0:.0f} s  ->  {a.out}")
node.destroy_node(); rclpy.shutdown()
