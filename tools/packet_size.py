#!/usr/bin/env python3
"""Compare the on-air size of one heartbeat in different encodings.

The binary layout mirrors include/adsas/protocol.h (12 B header + 17 B payload
+ 2 B CRC). At 5 Hz per node in a 4-node full mesh, every node transmits
3 copies of its heartbeat (TCP unicast to each peer), so link load is
    size * 5 Hz * 3 peers  per node.
"""
import json
import struct

HEADER = struct.Struct("<BBBBHHI")       # magic, ver|type, src, dst, seq, len, t_ms
HEARTBEAT = struct.Struct("<BBiihHHB")   # state, batt, x, y, alt, epoch, done, flags

assert HEADER.size == 12 and HEARTBEAT.size == 17

sample = dict(state=1, battery=87, x_dm=-1234, y_dm=5678, alt_dm=200,
              epoch=1, cells_done=7, flags=0)

binary = HEADER.size + HEARTBEAT.size + 2
as_json = len(json.dumps({"type": "heartbeat", "src": 2, "dst": 255, "seq": 1043,
                          "t_ms": 123456, **sample}).encode())
as_csv = len("HB,2,255,1043,123456,1,87,-1234,5678,200,1,7,0\n".encode())

RATE_HZ, PEERS = 5, 3
print(f"{'encoding':<10}{'bytes':>8}{'bytes/s/node':>15}{'vs binary':>12}")
for name, size in (("binary", binary), ("csv-text", as_csv), ("json", as_json)):
    print(f"{name:<10}{size:>8}{size * RATE_HZ * PEERS:>15}{size / binary:>11.1f}x")
