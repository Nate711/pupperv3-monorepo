packer {
  required_plugins {
    git = {
      version = ">=v0.3.2"
      source  = "github.com/ethanmdavidson/git"
    }
  }
}

source "arm" "raspbian" {
  file_urls             = ["./pupOS_full_base.img"]
  file_checksum_type    = "none"
  file_target_extension = "img"
  image_build_method    = "resize"
  image_path            = "pupOS_pios_ai.img"
  image_size            = "14G"
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
  default = ""
}

variable "CARTESIA_API_KEY" {
  type = string
  default = ""
}

variable "GOOGLE_API_KEY" {
  type = string
  default = ""
}

variable "LIVEKIT_URL" {
  type = string
  default = ""
}

variable "LIVEKIT_API_KEY" {
  type = string
  default = ""
}

variable "LIVEKIT_API_SECRET" {
  type = string
  default = ""
}

variable "DEEPGRAM_API_KEY" {
  type = string
  default = ""
}

variable "ELEVEN_API_KEY" {
  type = string
  default = ""
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

  provisioner "shell" {
    script = "provision_pios_ai.sh"
  }

  provisioner "shell" {
    inline = [
      "echo 'export OPENAI_API_KEY=${var.OPENAI_API_KEY}' >> ~/.bashrc",
      "echo 'export CARTESIA_API_KEY=${var.CARTESIA_API_KEY}' >> ~/.bashrc",
      "echo 'export GOOGLE_API_KEY=${var.GOOGLE_API_KEY}' >> ~/.bashrc",
      "echo 'export LIVEKIT_URL=${var.LIVEKIT_URL}' >> ~/.bashrc",
      "echo 'export LIVEKIT_API_KEY=${var.LIVEKIT_API_KEY}' >> ~/.bashrc",
      "echo 'export LIVEKIT_API_SECRET=${var.LIVEKIT_API_SECRET}' >> ~/.bashrc",
      "echo 'export DEEPGRAM_API_KEY=${var.DEEPGRAM_API_KEY}' >> ~/.bashrc",
      "echo 'export ELEVEN_API_KEY=${var.ELEVEN_API_KEY}' >> ~/.bashrc",
    ]
  }

  provisioner "shell" {
    inline = [
      "sudo mv /etc/resolv.conf.bk /etc/resolv.conf",
    ]
  }
}