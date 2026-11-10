#!/usr/bin/env python3
"""Write the two floor textures used by the converted worlds (pure Python, no PIL).

checker_floor.png : one 1 m repeat of a 0.5 m grey checkerboard (stands in for the Webots 7 Floor)
metric_floor.png  : one 1 m repeat with 10 cm grid lines and a bold 1 m border (MetricFloor)

Usage: make_textures.py OUTPUT_DIR
"""
import os
import struct
import sys
import zlib


def write_png(path, width, height, pixel):
    rows = []
    for y in range(height):
        row = bytearray([0])  # filter type 0
        for x in range(width):
            row.extend(pixel(x, y))
        rows.append(bytes(row))

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)

    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(b''.join(rows), 9)) + chunk(b'IEND', b'')
    with open(path, 'wb') as f:
        f.write(png)


def checker(x, y, n=256):
    light = ((x // (n // 2)) + (y // (n // 2))) % 2 == 0
    return (200, 200, 200) if light else (150, 150, 150)


def metric(x, y, n=500):
    if x < 3 or y < 3 or x >= n - 3 or y >= n - 3:
        return (60, 60, 60)          # 1 m line
    if x % 50 in (0, 49) or y % 50 in (0, 49):
        return (130, 130, 130)       # 10 cm line
    return (225, 225, 220)


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    write_png(os.path.join(out, 'checker_floor.png'), 256, 256, checker)
    write_png(os.path.join(out, 'metric_floor.png'), 500, 500, metric)
