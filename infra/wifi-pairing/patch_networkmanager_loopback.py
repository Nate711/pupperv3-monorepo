#!/usr/bin/env python3
"""Patch python3-networkmanager for NetworkManager 1.42+ loopback devices.

NetworkManager >= 1.42 manages the loopback interface as device type 32
(NM_DEVICE_TYPE_LOOPBACK). The python3-networkmanager library (2.2) predates
this: its device_class() does a bare dict lookup with no default, so it raises
`KeyError: 32` while enumerating devices. That crash-loops comitup (it never
claims its D-Bus name), so the WiFi hotspot never comes up.

This script adds the loopback constant + class + map entry, and makes unknown
device types degrade to the base Device class instead of raising. It is
idempotent: running it again is a no-op.

Run as root (the target file lives under /usr/lib).
"""
import sys

TARGET = "/usr/lib/python3/dist-packages/NetworkManager.py"

EDITS = [
    (
        "NM_DEVICE_TYPE_VRF = 31\n",
        "NM_DEVICE_TYPE_VRF = 31\nNM_DEVICE_TYPE_LOOPBACK = 32\n",
    ),
    (
        "class Generic(Device): pass\n",
        "class Generic(Device): pass\nclass Loopback(Device): pass\n",
    ),
    (
        "        NM_DEVICE_TYPE_WIFI_P2P: WifiP2p,\n    }[typ]",
        "        NM_DEVICE_TYPE_WIFI_P2P: WifiP2p,\n"
        "        NM_DEVICE_TYPE_LOOPBACK: Loopback,\n    }.get(typ, Device)",
    ),
]


def main() -> int:
    try:
        src = open(TARGET).read()
    except FileNotFoundError:
        print(f"{TARGET} not found — is python3-networkmanager installed?", file=sys.stderr)
        return 1

    if "NM_DEVICE_TYPE_LOOPBACK" in src:
        print("already patched — nothing to do")
        return 0

    for old, new in EDITS:
        n = src.count(old)
        if n != 1:
            print(
                f"unexpected content: found {n} matches (expected 1) for a patch "
                f"anchor. NetworkManager.py version may differ; not modifying.",
                file=sys.stderr,
            )
            return 1
        src = src.replace(old, new)

    # Back up once, then write.
    backup = TARGET + ".orig"
    try:
        open(backup, "x").write(open(TARGET).read())
    except FileExistsError:
        pass

    open(TARGET, "w").write(src)

    import ast
    ast.parse(src)  # fail loudly if we produced invalid Python
    print(f"patched {TARGET} (backup at {backup})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
