packer {
  required_plugins {
    git = {
      version = ">=v0.3.2"
      source  = "github.com/ethanmdavidson/git"
    }
  }
}

source "arm" "raspbian" {
  file_urls             = ["./pupOS_pios_base.img"]
  file_checksum_type    = "none"
  file_target_extension = "img"
  image_build_method    = "resize"
  image_path            = "pupOS_pios_full.img"
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

variable "OPENAI_API_KEY" {
  type = string
  default = env("PUPPER_OPENAI_API_KEY")
}

build {
  sources = ["source.arm.raspbian"]

  # Required to get internet access
  provisioner "shell" {
    inline = [
        "sudo mv /etc/resolv.conf /etc/resolv.conf.bk",
        "echo 'nameserver 8.8.8.8' | sudo tee /etc/resolv.conf",
        "echo 'nameserver 1.1.1.1' | sudo tee -a /etc/resolv.conf",
    ]
  }

  # Set hostname to 'pupper'
  provisioner "shell" {
    script = "setup_scripts/set_hostname.sh"
  }

  provisioner "shell" {
    script = "provision_pios_full.sh"
  }

  provisioner "file" {
    source      = "asound.conf"
    destination = "/etc/asound.conf"
  }

  provisioner "file" {
    sources      = ["resources/blue_eyes.jpg", "resources/pupperface2.jpg", "resources/pupperface3.jpg", "resources/bmo9.jpg"]
    destination = "/usr/share/rpd-wallpaper/"
  }

  provisioner "shell" {
    inline = [
      "cp /usr/share/rpd-wallpaper/blue_eyes.jpg /usr/share/rpd-wallpaper/fisherman.jpg",
    ]
  }

  provisioner "shell" {
    inline = [
      "echo 'export OPENAI_API_KEY=${var.OPENAI_API_KEY}' >> ~/.bashrc"
    ]
  }


# THIS DOESN'T WORK
# Providing a kanshi config file prevents the screen configuration menu on pios from opening
#   provisioner "shell" {
#     inline = [
#       "mkdir -p /home/pi/.config/kanshi",
#     ]
#   }

#   provisioner "file" {
#     source      = "resources/config"
#     destination = "/home/pi/.config/kanshi/config"
#   }

#   provisioner "shell" {
#     inline = [
#       "sudo chown pi /home/pi/.config/kanshi/config",
#     ]
#   }

# Doesn't work: arm.raspbian: Cannot open display: 
#   provisioner "shell" {
#     inline = [
#       "pcmanfm --set-wallpaper /usr/share/rpd-wallpaper/pupperface3.png",
#     ]
#   }


  provisioner "shell" {
    inline = [
      "sudo mv /etc/resolv.conf.bk /etc/resolv.conf",
    ]
  }
}