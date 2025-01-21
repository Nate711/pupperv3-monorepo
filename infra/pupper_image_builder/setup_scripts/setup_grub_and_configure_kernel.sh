#!/bin/bash -e

set -x

sudo apt install -y grub2-common cpufrequtils
sudo mkdir /boot/grub

# Set kernel preemption model to full preemption for low-latency
KERNEL_CMD="preempt=full nohz_full=all threadirqs"

# TODO: Observed overheating when using this, commented out for now
# Set CPU governor to performance
# echo "GOVERNOR='performance'" | sudo tee /etc/default/cpufrequtils

# TODO: fix the following (either doesn't persist after reboot or just doesn't work)

# # Disable proactive memory compaction for low latency
# echo "0" | sudo tee /proc/sys/vm/compaction_proactiveness

# # Disable Kernel Samepage Merging (KSM)
# echo "0" | sudo tee /sys/kernel/mm/ksm/run

# # Set a minimum time-to-live (TTL) for LRU generation to mitigate stuttering
# echo "1000" | sudo tee /sys/kernel/mm/lru_gen/min_ttl_ms

# # Reduce kernel memory reclaim settings to improve I/O stability under heavy loads
# echo "5" | sudo tee /proc/sys/vm/dirty_ratio
# echo "5" | sudo tee /proc/sys/vm/dirty_background_ratio

# # Configure RCU to minimize wake-up events
# KERNEL_CMD+=" rcu_nocbs=all rcutree.enable_rcu_lazy=1"

# # Apply kernel boot-time parameters by modifying GRUB configuration
# GRUB_CONFIG="c"
# if grep -q "GRUB_CMDLINE_LINUX_DEFAULT" $GRUB_CONFIG; then
#     sudo sed -i "/GRUB_CMDLINE_LINUX_DEFAULT/c\GRUB_CMDLINE_LINUX_DEFAULT=\"$KERNEL_CMD\"" $GRUB_CONFIG
# else
#     echo "GRUB_CMDLINE_LINUX_DEFAULT=\"$KERNEL_CMD\"" | sudo tee -a $GRUB_CONFIG
# fi

# Update GRUB to apply changes on the next reboot
sudo update-grub
