packer {
  required_plugins {
    git = {
      version = ">=v0.3.2"
      source  = "github.com/ethanmdavidson/git"
    }
  }
}

source "arm" "ubuntu" {
  file_urls             = ["https://cdimage.ubuntu.com/releases/24.04.1/release/ubuntu-24.04.1-preinstalled-server-arm64+raspi.img.xz"]
  file_checksum_url     = "https://cdimage.ubuntu.com/releases/24.04.1/release/SHA256SUMS"
  file_checksum_type    = "sha256"
  file_target_extension = "xz"
  file_unarchive_cmd    = ["xz", "--decompress", "$ARCHIVE_PATH"]
  image_build_method    = "resize"
  image_path            = "pupOS_ubuntu_desktop_ros_base.img"
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

  # Fix ubuntu sources
  provisioner "shell" {
    script = "setup_scripts/fix_ubuntu_sources.sh"
  }
  
  # Set up DNS and install ubuntu-desktop
  provisioner "shell" {
    inline = [
      "rm -f /etc/resolv.conf",
      "echo 'nameserver 1.1.1.1' > /etc/resolv.conf",
      "echo 'nameserver 8.8.8.8' >> /etc/resolv.conf",
      "apt update",
      "apt install -y ubuntu-desktop",
      "apt upgrade --yes --option=Dpkg::Options::=--force-confdef",
      "mkdir -p /home/pi"
    ]
  }
  
  # Install GRUB and configure the Linux kernel for low latency
  provisioner "shell" {
    script = "setup_scripts/setup_grub_and_configure_kernel.sh"
  }

  # Use 8.8.8.8 and 1.1.1.1 DNS
  provisioner "shell" {
    script = "setup_scripts/set_nameservers.sh"
  }

  provisioner "shell" {
    script = "setup_scripts/install_ros_desktop.sh"
  }
  
  # Clean up
  provisioner "shell" {
    inline = [
      "apt -y autoremove && apt -y clean",
    ]
  }
}
