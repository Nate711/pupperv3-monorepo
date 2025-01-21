packer {
  required_plugins {
    git = {
      version = ">=v0.3.2"
      source  = "github.com/ethanmdavidson/git"
    }
  }
}

source "arm" "ubuntu" {
  file_urls             = ["./pupOS_ubuntu_desktop_ros_base.img"]
  file_checksum_type    = "none"
  file_target_extension = "img"
  image_build_method    = "resize"
  image_path            = "pupOS_ubuntu_desktop_full.img"
  image_size            = "12G"
  image_type            = "dos"
  image_partitions {
    name         = "boot"
    type         = "c"
    start_sector = "2048"
    filesystem   = "fat"
    size         = "256M"
    mountpoint   = "/boot/firmware"
  }
  image_partitions {
    name         = "root"
    type         = "83"
    start_sector = "526336"
    filesystem   = "ext4"
    size         = "0"
    mountpoint   = "/"
  }
  image_chroot_env             = ["PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin"]
  qemu_binary_source_path      = "/usr/bin/qemu-aarch64-static"
  qemu_binary_destination_path = "/usr/bin/qemu-aarch64-static"
}

build {
  sources = ["source.arm.ubuntu"]

  provisioner "file" {
    source      = "user-data"
    destination = "/boot/firmware/user-data"
  }

  # Add temporary DNS for internet
  provisioner "shell" {
    inline = [
      "sudo mv /etc/resolv.conf /etc/resolv.conf.bk",
      "sudo echo 'nameserver 8.8.8.8' > /etc/resolv.conf",
      "sudo echo 'nameserver 1.1.1.1' > /etc/resolv.conf",
    ]
  }

  # Set hostname to 'pupper'
  # Does not fix sudo: unable to resolve host 02eafc09d153: Name or service not known
  provisioner "shell" {
    script = "setup_scripts/set_hostname.sh"
  }

  # Fix ubuntu sources
  provisioner "shell" {
    script = "setup_scripts/fix_ubuntu_sources.sh"
  }

  provisioner "shell" {
    script = "setup_scripts/provision.sh"
  }

  # Already installed in base image
  # Install ros jazzy desktop
#   provisioner "shell" {
#     script = "setup_scripts/install_ros_desktop.sh"
#   }

  provisioner "shell" {
    script = "setup_scripts/install_libcamera_rpicam_apps.sh"
  }

  provisioner "shell" {
    script = "setup_scripts/install_pupper_ros_packages.sh"
  }

  provisioner "file" {
    source      = "resources/"
    destination = "/home/pi/resources/"
  }

  provisioner "shell" {
    script = "setup_scripts/install_hailo.sh"
  }

  provisioner "shell" {
    script = "setup_scripts/install_chromium.sh"
  }

  provisioner "shell" {
    script = "setup_scripts/set_nameservers.sh"
  }

}
