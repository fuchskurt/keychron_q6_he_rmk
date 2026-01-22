#!/usr/bin/env python3
import math

VALID_RAW_MIN = 1200
VALID_RAW_MAX = 3500
REF_ZERO_TRAVEL = 3121

STEP = 1
Q = 8
SCALE = 1 << Q

A = 426.88962
B = -0.48358
C = 2.04637e-4
D = -2.99368e-8

def f(x: float) -> float:
    x = max(VALID_RAW_MIN, min(VALID_RAW_MAX, x))
    return A + B*x + C*(x*x) + D*(x*x*x)

def rust_i16_hex(v: int) -> str:
    """Format a signed i16 as a Rust hex literal."""
    # v is already clamped to i16 range below, but keep it robust:
    if v < -32768:
        v = -32768
    elif v > 32767:
        v = 32767

    if v < 0:
        return f"-0x{(-v):04X}"
    return f"0x{v:04X}"

ref = f(float(REF_ZERO_TRAVEL))

vals = []
for x in range(VALID_RAW_MIN, VALID_RAW_MAX + 1, STEP):
    delta = f(float(x)) - ref
    qv = int(round(delta * SCALE))
    qv = max(-32768, min(32767, qv))
    vals.append(qv)

print(f"// Auto-generated LUT: delta_from_ref(x) in Q{Q} (scale={SCALE})")
print(f"pub const LUT_MIN: u16 = {VALID_RAW_MIN};")
print(f"pub const LUT_MAX: u16 = {VALID_RAW_MAX};")
print(f"pub const LUT_STEP: u16 = {STEP};")
print(f"pub const LUT_Q: u8 = {Q};")
print(f"pub const LUT_SCALE: i32 = {SCALE};")
print(f"pub const LUT_LEN: usize = {len(vals)};")
print(f"pub const LUT_DELTA_Q{Q}: [i16; LUT_LEN] = [")

for i in range(0, len(vals), 12):
    chunk = ", ".join(rust_i16_hex(v) for v in vals[i:i+12])
    print(f"    {chunk},")
print("];")
