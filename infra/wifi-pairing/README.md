# Headless WiFi pairing (comitup) for Pupper

Lets you get a Pupper onto a new WiFi network **without a keyboard/mouse**. When
no known network is reachable, the Pi broadcasts a `Pupper-Setup-<nnn>` hotspot;
you join it from a phone, a captive portal opens, you pick the venue's WiFi and
type its password on the phone. The network is saved and auto-joined next time.

Built on [comitup](https://github.com/davesteele/comitup). This directory is the
source-of-truth for the Pupper-specific pieces plus an installer.

## Install

**Easiest — one command** (updates a pupper already running this repo; does the
system side *and* builds/restarts the GUI):

```bash
./deploy_wifi_pairing.sh        # at the repo root; run as 'pi', uses sudo
```

Fresh images get this automatically via the image builder
(`infra/pupper_image_builder`), so `deploy_wifi_pairing.sh` is for updating
puppers already in the field.

<details><summary>Or run the two halves manually</summary>

```bash
infra/wifi-pairing/install_wifi_pairing.sh                     # system side
cd pupper-rs && cargo build --release --target aarch64-unknown-linux-gnu
pupper-rs/install_service.sh && sudo systemctl restart pupper-gui   # GUI button
```
</details>

## Contents

| Path | Purpose |
|------|---------|
| `install_wifi_pairing.sh` | Idempotent installer (backs up originals as `*.orig`) |
| `comitup.conf` | comitup config — hotspot name `Pupper-Setup-<nnn>`, AP password |
| `comitup-web/` | comitup web-portal customization: **Saved networks & priority** page (`netpriority.py`, `known.html`) + patched `comitupweb.py` / `index.html` |
| `pupper-pairing` | Helper (`on`/`off`/`status`) — drops `wlan0` so comitup raises the hotspot; keeps saved networks |
| `sudoers.d/pupper-pairing` | Scoped NOPASSWD so the GUI can call the helper |
| `patch_networkmanager_loopback.py` | Idempotent fix for the comitup crash loop on NM 1.42+ |

## How the pairing button works

The GUI's **WiFi** button runs `sudo pupper-pairing on`, which does
`nmcli device disconnect wlan0`. comitup sees the link drop and brings up the
hotspot **without deleting saved networks** (its `delete_connection` D-Bus call
is destructive, so we avoid it). comitup holds the hotspot while a phone is
connected; with no client it retries known networks after ~180 s.

## Notes / caveats

- **NetworkManager 1.42+ loopback bug (critical):** NM manages `lo` as device
  type 32, which the packaged `python3-networkmanager` (2.2) doesn't know — its
  `device_class()` raised `KeyError: 32` and crash-looped comitup so the hotspot
  never came up. `patch_networkmanager_loopback.py` fixes it and also degrades
  future unknown device types to the base class. Backed up as
  `NetworkManager.py.orig`. An `apt` upgrade of `python3-networkmanager` would
  overwrite it — re-run the installer.
- The `comitup-web` files match comitup `1.38-2~deb12u1` (Bookworm). If comitup
  is upgraded, re-diff `comitupweb.py` / `index.html` against the new package.
- comitup manages `wlan0`; don't trigger pairing mode over a WiFi-based SSH/
  Tailscale session — it drops the link. Test at the robot.
