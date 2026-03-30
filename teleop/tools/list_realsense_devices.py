#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""List Intel RealSense devices and print serial numbers.

用途：当你不知道 RealSense 序列号时，用它来枚举当前机器连接的所有 RealSense 相机。

优先方案：使用 pyrealsense2 (与本项目 image_server.py 一致)。
备选方案：若 pyrealsense2 不可用，会给出 rs-enumerate-devices / lsusb 的提示。

运行：
  python3 teleop/tools/list_realsense_devices.py
"""

from __future__ import annotations


def _try_pyrealsense2():
    try:
        import pyrealsense2 as rs
    except Exception as e:
        return None, e
    return rs, None


def main():
    rs, err = _try_pyrealsense2()
    if rs is None:
        print("pyrealsense2 not available.")
        print(f"Import error: {err}")
        print("\nTry one of the following on Linux:\n")
        print("  1) rs-enumerate-devices")
        print("  2) realsense-viewer")
        print("  3) lsusb | grep -iE 'intel|realsense'")
        return

    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("No RealSense devices found.")
        return

    print(f"Found {len(devices)} RealSense device(s):\n")

    def _info(dev, key):
        try:
            if dev.supports(key):
                return dev.get_info(key)
        except Exception:
            pass
        return None

    for i, dev in enumerate(devices):
        sn = _info(dev, rs.camera_info.serial_number)
        name = _info(dev, rs.camera_info.name)
        pid = _info(dev, rs.camera_info.product_id)
        fw = _info(dev, rs.camera_info.firmware_version)
        usb = _info(dev, rs.camera_info.usb_type_descriptor)
        port = _info(dev, rs.camera_info.physical_port)

        print(f"[{i}] name: {name}")
        print(f"    serial_number: {sn}")
        print(f"    product_id: {pid}")
        print(f"    firmware_version: {fw}")
        print(f"    usb_type: {usb}")
        print(f"    physical_port: {port}\n")


if __name__ == "__main__":
    main()
