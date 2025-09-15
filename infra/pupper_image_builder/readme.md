# pupper_image_builder

## Description
An automated OS image builder for the Pupper robot. Pulls the latest Ubuntu image, applies all the necessary configurations, and installs ROS2 and Pupper-specific packages.

## How to use
### Prerequisites
1. Install Docker Desktop or some Docker engine
2. Make sure your Docker is up-to-date. After upgrading MacOS, older versions of Docker will not work.


### Build PiOS image
```
./make_pios_base_image.sh
./make_pios_full_image.sh
./make_pios_ai_image.sh
```
In order to use LLM capabilities, you must create a `.env.local` in the `pupper_image_builder` directory based on `.env.example` and then run with the flag `--include-keys`
```
./make_pios_full_image.sh --include-keys
```

Sometimes you will get lfs errors which is due to rate limiting. Wait a bit a try again.


### Default credentials
Hostname: `pupper`

User: `pi`

Password: `rhea123`

### First boot
1. Disable a NetworkManager service to decrease boot time from ~90s to ~30s:

`sudo systemctl disable NetworkManager-wait-online.service`

2. Pair PS4/5 controller over bluetooth.
3. Optionally connect to WIFI
4. Optionally enable Pi to automatically log in to the `pi` user on boot by writing the following to `/etc/gdm3/custom.conf`:
```
[daemon]
# Enabling automatic login
AutomaticLoginEnable=true
AutomaticLogin=pi
```
(TODO: add this to some startup script)

## Operating systems
* Ubuntu - see above.
* PiOS - checkout `pios` branch. 

## Troubleshooting
### No sound from Pupper speaker when using PiOS
* You need to blacklist the DualSense controller audio device
* sudo nano /etc/modprobe.d/blacklist.conf
* Add: "blacklist snd_usb_audio"

### Out of memory / build taking a long time
* Increase RAM memory limit in the Docker Desktop application (go to settings (gear icon) -> resources). Primarily needed to building ROS on PiOS.

## Issues
* `NetworkManager-wait-online.service` adds 1 min to startup time
    * Have to run `sudo systemctl disable NetworkManager-wait-online.service` after booting actual RPi. Could not disable it for some reason in the provisioning scripts.
* Sometimes get errors like `arm.ubuntu:         <urlopen error <urlopen error [Errno -3] Temporary failure in name resolution> (https://raw.githubusercontent.com/ros/rosdistro/master/humble/distribution.yaml)>` during image build where the container can't properly get web resources. 
* Using the pre-built Ubuntu Desktop image does not work. Any window takes a very long time to open. So instead we install ubuntu-desktop on top of the pre-built Ubuntu Server image.
* Sometimes user-data is not correctly installed. You'll know if the Pi boots and you don't see the pi user to log in to. PiOS uses a system daemon to run a script on first boot. Maybe cloud-init isn't working. Can try to put setup script in cloud-init per-boot folder or once folder.



# PiOS automatic settings
cmdline.txt
```
video=HDMI-A-1:720x720M@60D,rotate=270 console=serial0,115200 console=tty1 root=PARTUUID=75d6d1b4-02 rootfstype=ext4 fsck.repair=yes rootwait quiet init=/usr/lib/raspberrypi-sys-mods/firstboot splash plymouth.ignore-serial-consoles cfg80211.ieee80211_regdom=JP systemd.run=/boot/firstrun.sh systemd.run_success_action=reboot systemd.unit=kernel-command-line.target
```

firstrun.sh
```
#!/bin/bash

set +e

CURRENT_HOSTNAME=`cat /etc/hostname | tr -d " \t\n\r"`
if [ -f /usr/lib/raspberrypi-sys-mods/imager_custom ]; then
   /usr/lib/raspberrypi-sys-mods/imager_custom set_hostname pupper
else
   echo pupper >/etc/hostname
   sed -i "s/127.0.1.1.*$CURRENT_HOSTNAME/127.0.1.1\tpupper/g" /etc/hosts
fi
FIRSTUSER=`getent passwd 1000 | cut -d: -f1`
FIRSTUSERHOME=`getent passwd 1000 | cut -d: -f6`
if [ -f /usr/lib/raspberrypi-sys-mods/imager_custom ]; then
   /usr/lib/raspberrypi-sys-mods/imager_custom enable_ssh
else
   systemctl enable ssh
fi
if [ -f /usr/lib/userconf-pi/userconf ]; then
   /usr/lib/userconf-pi/userconf 'pi' '$5$bHlIjffqCc$8sCEUcNlyls7Qiy8HQMwEqCSTJgjukrZEV9zzPDbgF/'
else
   echo "$FIRSTUSER:"'$5$bHlIjffqCc$8sCEUcNlyls7Qiy8HQMwEqCSTJgjukrZEV9zzPDbgF/' | chpasswd -e
   if [ "$FIRSTUSER" != "pi" ]; then
      usermod -l "pi" "$FIRSTUSER"
      usermod -m -d "/home/pi" "pi"
      groupmod -n "pi" "$FIRSTUSER"
      if grep -q "^autologin-user=" /etc/lightdm/lightdm.conf ; then
         sed /etc/lightdm/lightdm.conf -i -e "s/^autologin-user=.*/autologin-user=pi/"
      fi
      if [ -f /etc/systemd/system/getty@tty1.service.d/autologin.conf ]; then
         sed /etc/systemd/system/getty@tty1.service.d/autologin.conf -i -e "s/$FIRSTUSER/pi/"
      fi
      if [ -f /etc/sudoers.d/010_pi-nopasswd ]; then
         sed -i "s/^$FIRSTUSER /pi /" /etc/sudoers.d/010_pi-nopasswd
      fi
   fi
fi
if [ -f /usr/lib/raspberrypi-sys-mods/imager_custom ]; then
   /usr/lib/raspberrypi-sys-mods/imager_custom set_wlan 'SSID-A7F77D' '501e003681562a72c44b34556bf9b2fe16ccc3b7de950d3fa310b804fd94fdd9' 'JP'
else
cat >/etc/wpa_supplicant/wpa_supplicant.conf <<'WPAEOF'
country=JP
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
ap_scan=1

update_config=1
network={
	ssid="SSID-A7F77D"
	psk=501e003681562a72c44b34556bf9b2fe16ccc3b7de950d3fa310b804fd94fdd9
}

WPAEOF
   chmod 600 /etc/wpa_supplicant/wpa_supplicant.conf
   rfkill unblock wifi
   for filename in /var/lib/systemd/rfkill/*:wlan ; do
       echo 0 > $filename
   done
fi
if [ -f /usr/lib/raspberrypi-sys-mods/imager_custom ]; then
   /usr/lib/raspberrypi-sys-mods/imager_custom set_keymap 'us'
   /usr/lib/raspberrypi-sys-mods/imager_custom set_timezone 'Asia/Tokyo'
else
   rm -f /etc/localtime
   echo "Asia/Tokyo" >/etc/timezone
   dpkg-reconfigure -f noninteractive tzdata
cat >/etc/default/keyboard <<'KBEOF'
XKBMODEL="pc105"
XKBLAYOUT="us"
XKBVARIANT=""
XKBOPTIONS=""

KBEOF
   dpkg-reconfigure -f noninteractive keyboard-configuration
fi
rm -f /boot/firstrun.sh
sed -i 's| systemd.run.*||g' /boot/cmdline.txt
exit 0
```
