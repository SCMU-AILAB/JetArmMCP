#!/usr/bin/env python3
"""用官方 SDK 的 buf_write 生成协议测试向量（不打开真实串口）。
运行：python3 tests/sdk_vectors.py"""
import sys, pathlib, types
sys.modules.setdefault("serial", types.ModuleType("serial"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "reference/hiwonder_jetarm_ros_orin_nx_ros2"))
import ros_robot_controller_sdk as sdk

class Fake(sdk.Board):
    def __init__(self):
        self.out = []
    class _P:
        def __init__(s, o): s.o = o
        def write(s, b): s.o.append(bytes(b))
    @property
    def port(self): return Fake._P(self.out)

b = Fake()
b.bus_servo_set_position(1.5, [[1, 500], [2, 761]])
b.out.append(None); b.buf_write(sdk.PacketFunction.PACKET_FUNC_BUS_SERVO, [0x05, 3])
b.bus_servo_stop([1, 2, 3])
b.set_buzzer(1900, 0.1, 0.05, 1)
for x in b.out:
    if x is not None: print(", ".join(f"0x{v:02X}" for v in x))
print("crc_tail", sdk.crc8_table[248:])
