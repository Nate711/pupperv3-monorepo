packer {
  required_plugins {
    git = {
      version = ">=v0.3.2"
      source  = "github.com/ethanmdavidson/git"
    }
  }
}

source "arm" "raspbian" {
  file_urls             = ["./pupOS_pios_full.img"]
  file_checksum_type    = "none"
  file_target_extension = "img"
  image_build_method    = "resize"
  image_path            = "pupOS_pios_ai.img"
  image_size            = "18G"
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

# Single build with two sources; use only on provisioners to inject keys
build {
  name    = "ai"
  sources = [
    "source.arm.raspbian",
  ]

  # Add a second build using the same source but with a local name
  # This produces two builds: ai.arm.raspbian and ai.with-keys
  source "source.arm.raspbian" {
    name = "with-keys"
  }

  # Required to get internet access (common)
  provisioner "shell" {
    inline = [
      "sudo mv /etc/resolv.conf /etc/resolv.conf.bk",
      "echo 'nameserver 8.8.8.8' | sudo tee /etc/resolv.conf",
      "echo 'nameserver 1.1.1.1' | sudo tee -a /etc/resolv.conf",
    ]
  }

  # Common provisioning script
  provisioner "shell" {
    script = "provision_pios_ai.sh"
  }

  # Ensure destination directory exists (with-keys only)
  provisioner "shell" {
    only   = ["arm.with-keys"]
    inline = [
      "mkdir -p /home/pi/pupperv3-monorepo/ai/llm-ui/agent-starter-python",
    ]
  }

  # Copy the .env.local file (with-keys only)
  provisioner "file" {
    only        = ["arm.with-keys"]
    source      = ".env.local"
    destination = "/home/pi/pupperv3-monorepo/ai/llm-ui/agent-starter-python/.env.local"
  }

  # Set secure permissions and ownership (with-keys only)
  provisioner "shell" {
    only   = ["arm.with-keys"]
    inline = [
      "sudo chown pi:pi /home/pi/pupperv3-monorepo/ai/llm-ui/agent-starter-python/.env.local",
      "sudo chmod 600 /home/pi/pupperv3-monorepo/ai/llm-ui/agent-starter-python/.env.local",
    ]
  }

  # Providing a kanshi config file prevents the screen configuration menu on pios from opening
  provisioner "shell" {
    inline = [
      "mkdir -p /home/pi/.config/kanshi",
      "sudo chown -R pi:pi /home/pi/.config/kanshi",
    ]
  }

  provisioner "file" {
    source      = "resources/config"
    destination = "/home/pi/.config/kanshi/config"
  }

  provisioner "shell" {
    inline = [
      "sudo chown pi:pi /home/pi/.config/kanshi/config",
      "sudo chmod 666 /home/pi/.config/kanshi/config",
    ]
  }

  # Restore resolv.conf (common)
  provisioner "shell" {
    inline = [
      "sudo mv /etc/resolv.conf.bk /etc/resolv.conf",
    ]
  }
}
