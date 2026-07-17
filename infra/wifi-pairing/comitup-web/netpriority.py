# Manage NetworkManager saved-wifi autoconnect priorities for comitup-web.
# Added as a local customization (not part of upstream comitup).
#
# NetworkManager picks the saved connection with the highest
# connection.autoconnect-priority when several are in range. This module
# lists saved wifi connections ordered by that priority and rewrites the
# priorities from a user-supplied ordering. All writes go through nmcli,
# which asks the NetworkManager daemon to persist them.

import subprocess

WIFI_TYPE = "802-11-wireless"


def _nmcli(*args):
    return subprocess.run(
        ["nmcli", *args],
        capture_output=True,
        text=True,
        check=True,
    ).stdout


def _get(uuid, field):
    return _nmcli("-g", field, "connection", "show", uuid).strip()


def list_saved_wifi():
    """Return saved wifi connections as dicts, best (highest) priority first."""
    out = _nmcli("-t", "-f", "UUID,TYPE", "connection", "show")
    conns = []
    for line in out.splitlines():
        if not line:
            continue
        uuid, _, ctype = line.partition(":")
        if ctype != WIFI_TYPE:
            continue
        try:
            prio = int(_get(uuid, "connection.autoconnect-priority") or "0")
        except ValueError:
            prio = 0
        conns.append(
            {
                "uuid": uuid,
                "name": _get(uuid, "connection.id"),
                "priority": prio,
                "autoconnect": _get(uuid, "connection.autoconnect") == "yes",
            }
        )
    # Highest priority first; stable tie-break by name for a predictable UI.
    conns.sort(key=lambda c: (-c["priority"], c["name"].lower()))
    return conns


def set_priority_order(ordered_uuids):
    """Assign descending priorities to the given uuids (first = most preferred).

    Only uuids that are currently saved wifi connections are touched, so a
    stale or hostile POST can't modify arbitrary connections. Returns the
    list of (uuid, new_priority) actually applied.
    """
    valid = {c["uuid"] for c in list_saved_wifi()}
    ordered = [u for u in ordered_uuids if u in valid]
    n = len(ordered)
    applied = []
    for index, uuid in enumerate(ordered):
        prio = n - index  # top of the list gets the largest number
        _nmcli(
            "connection",
            "modify",
            uuid,
            "connection.autoconnect-priority",
            str(prio),
        )
        applied.append((uuid, prio))
    return applied


if __name__ == "__main__":
    # Manual self-test: print the current ordering.
    for c in list_saved_wifi():
        print(
            "{priority:>4}  {name}  ({uuid}) auto={autoconnect}".format(**c)
        )
