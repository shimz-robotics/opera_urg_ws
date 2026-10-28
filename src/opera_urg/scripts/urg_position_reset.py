#!/usr/bin/env python3
"""urg_position.yaml の x, y, yaw を 0 にする。IP は残す。"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def load_positions(path):
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    entries = []
    for item in data.get("urgs") or []:
        entries.append({
            "ip": str(item["ip"]),
            "x": float(item["x"]),
            "y": float(item["y"]),
            "yaw": float(item["yaw"]),
        })
    if not entries:
        raise SystemExit("%s に URG の位置がありません。" % path)
    return entries


def write_positions(path, entries):
    lines = [
        "# x, y はメートル。yaw は URG の X 軸が世界の +X となす角。反時計回りが正。",
        "# 算出プログラムが同意後にこのファイルを更新する。",
        "urgs:",
    ]
    for entry in entries:
        lines.append("  - ip: \"%s\"" % entry["ip"])
        lines.append("    x: %.6f" % entry["x"])
        lines.append("    y: %.6f" % entry["y"])
        lines.append("    yaw: %.6f" % entry["yaw"])
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def main():
    share = get_package_share_directory("opera_urg")
    path = os.path.join(share, "config", "urg_position.yaml")
    entries = load_positions(path)
    for entry in entries:
        print(
            "%s  (%.6f, %.6f, yaw %.6f) -> (0, 0, yaw 0)"
            % (entry["ip"], entry["x"], entry["y"], entry["yaw"])
        )
    answer = input("x, y, yaw を 0 にリセットしますか? [y/N]: ").strip()
    if answer not in ("y", "Y"):
        print("リセットしませんでした。")
        return
    for entry in entries:
        entry["x"] = 0.0
        entry["y"] = 0.0
        entry["yaw"] = 0.0
    write_positions(path, entries)
    print("書き込みました: %s" % path)
    print("配信側を起動し直すと、リセットした位置が使われます。")


if __name__ == "__main__":
    main()
