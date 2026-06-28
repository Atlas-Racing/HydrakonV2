# CAN Bus Setup on Jetson AGX Orin with DSD TECH USB-CAN Adapter

Setting up CAN bus on Jetson AGX Orin using the DSD TECH SH-C30A USB-to-CAN adapter, through the gs_usb driver and SocketCAN, with ROS2 Jazzy on top for sending/receiving frames.

The SH-C30A (like other CANable/candleLight adapters) runs candleLight firmware. Linux picks it up through the gs_usb kernel driver and it shows up as normal SocketCAN interface (can0, can1, etc). For the ROS2 side, ros2_socketcan (Autoware Foundation, has a Jazzy release) bridges the SocketCAN interface into ROS2 topics.

note: check through changes specific to ROS2 Jazzy. (previoualy tested on Humble)
---

## 1. Prerequisites

- Jetson AGX Orin on JetPack 6.0 (kernel 5.15)
- ROS2 Jazzy 
- DSD TECH SH-C30A
  
Install ROS2 SocketCAN bridge:

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros2-socketcan can-utils
```

If `ros-jazzy-ros2-socketcan` isn't in apt mirror yet, build from source:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/autowarefoundation/ros2_socketcan.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros2_socketcan ros2_socketcan_msgs
source install/setup.bash
```

---

## 2. Connect Adapter and Load the Driver

Plug in DSD TECH adapter, then check if detected at USB level:

```bash
lsusb
```

candleLight devices (including the SH-C30A) show up under vendor ID 1d50.

Now load the gs_usb driver:

```bash
sudo modprobe gs_usb
```

### If this fails with "module not found" error

```
modprobe: FATAL: Module gs_usb not found in directory /lib/modules/5.15.136-tegra
```

The kernel patch number might be different (5.15.122-tegra, 5.15.136-tegra, etc) but it'll be a 5.15.x-tegra string on JetPack 6.0.

This is a known issue, well documented on NVIDIA forums, on stock JetPack 6.0. Any gs_usb device (CANable, candleLight, this one) hits the same wall on JetPack 6.x because the module just isn't shipped in the default kernel.

Check whether gs_usb support exists in kernel config:

```bash
zcat /proc/config.gz | grep CONFIG_CAN_GS_USB
```

If that returns `CONFIG_CAN_GS_USB=m` or `=y`, the module should already be there and something else is going on (check Section 7). If it returns nothing, it's confirmed missing, go to Section 3, fix it, then come back and rerun `sudo modprobe gs_usb`.

If modprobe worked fine first time, skip Section 3, move to Section 4.

---

## 3. Fix: Enabling gs_usb on JetPack 6.x
note: after fix, edit to update changes.

Two ways to fix this:

1. Rebuild the kernel module with CONFIG_CAN_GS_USB enabled (Method A, the official NVIDIA way)
2. Use a community script that automates the build for the right L4T version (Method B, quicker)

Method 1 is the proper long term fix. Method B gets there faster.

### Method 1: Official Kernel Rebuild

Steps from NVIDIA's [Kernel Customization guide](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/SD/Kernel/KernelCustomization.html), adapted for gs_usb specifically.

#### 3.1 Install prerequisites

```bash
sudo apt update
sudo apt install -y git-core build-essential bc libssl-dev
```

The Jetson Linux toolchain matching the L4T release is also needed when cross-compiling from an x86_64 host (see the [toolchain page](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/AT/JetsonLinuxToolchain.html)). Building natively on the Jetson itself, the cross toolchain step can be skipped and the onboard GCC used directly.

#### 3.2 Sync kernel sources

From the Linux_for_Tegra install directory (present if flashed via SDK Manager, otherwise grab the public sources separately):

```bash
cd <install-path>/Linux_for_Tegra/source
./source_sync.sh -k -t <release-tag>
```

The release tag has to match the exact L4T version in use, it's listed in the JetPack release notes. Getting this wrong means the module won't load (vermagic mismatch).

No Linux_for_Tegra folder around (building natively without having flashed via host)? Download the public sources for the release in question from the [Jetson Linux archive](https://developer.nvidia.com/embedded/jetson-linux-archive) instead:

```bash
tar xf public_sources.tbz2 -C <install-path>/Linux_for_Tegra/..
cd <install-path>/Linux_for_Tegra/source
tar xf kernel_src.tbz2
tar xf kernel_oot_modules_src.tbz2
```

#### 3.3 Enable CONFIG_CAN_GS_USB

```bash
cd <install-path>/Linux_for_Tegra/source/kernel/kernel-jammy-src/arch/arm64/configs
nano tegra_defconfig
```

Add or change this line:

```
CONFIG_CAN_GS_USB=m
```

Building it as a module (`=m`) instead of built in (`=y`) means it can just be modprobed like normal afterward, no need to reflash the whole kernel image.

#### 3.4 Build and install the module

```bash
cd <install-path>/Linux_for_Tegra/source
export CROSS_COMPILE=<toolchain-path>/bin/aarch64-buildroot-linux-gnu-   # skip if building natively
export KERNEL_HEADERS=$PWD/kernel/kernel-jammy-src
make modules
```

Install it (native build goes into the running rootfs, cross-compiled goes to the target rootfs path):

```bash
export INSTALL_MOD_PATH=<install-path>/Linux_for_Tegra/rootfs/
sudo -E make modules_install
```

Update the initramfs. Native, directly on target:

```bash
sudo nv-update-initrd
```

Cross-compiling from a host:

```bash
cd <install-path>/Linux_for_Tegra
sudo ./tools/l4t_update_initrd.sh
```

#### 3.5 Reboot and verify

```bash
sudo reboot
```

After it comes back up:

```bash
sudo modprobe gs_usb
modinfo gs_usb
```

Output should look something like:

```
filename: /lib/modules/<kernel-version>/kernel/net/can/usb/gs_usb.ko
license: GPL v2
description: Socket CAN device driver for ... USB2.0 to CAN interfaces and candleLight USB CAN interfaces.
depends:
name: gs_usb
vermagic: <kernel-version> SMP preempt mod_unload modversions aarch64
```

No errors means it worked. Go back to Section 2, confirm modprobe works, then move to Section 4.

---

### Method B: Community Build Script

There's a script that automates the whole gs_usb build, built for L4T R36.4.3:

```bash
wget https://github.com/lucianovk/jetson-gs_usb-kernel-builder/raw/main/jetson-gs_usb-kernel-builder.sh
chmod +x jetson-gs_usb-kernel-builder.sh
sudo ./jetson-gs_usb-kernel-builder.sh
```

It downloads matching kernel sources, sets CONFIG_CAN_GS_USB=m, builds just module, installs it, and updates initramfs.

On JetPack 6.0 / L4T R36.3.0 (what this guide is built around), heads up: the script targets R36.4.3 by default, not R36.3.0. The KERNEL_VERSION variable in the script needs editing to point at the R36.3.0 tarball from the [Jetson Linux archive](https://developer.nvidia.com/embedded/linux-tegra), otherwise it's a vermagic mismatch and the module won't load. If editing a third party script feels like too much, Method A is simpler since it always syncs against whatever's actually installed.

Cross-compiling from an x86_64 host is also supported instead of building on the Jetson itself, see the [script's README](https://github.com/lucianovk/jetson-gs_usb-kernel-builder) for that (export /proc/config.gz from the Jetson, copy it over, run the script on the host).

Worth noting this is a third party script, not something NVIDIA maintains, so quick read through before running with sudo -_-

Once done, go back to Section 2, confirm modprobe works, then move to Section 4.

---

## 4. Bringing Up the CAN Interface

With gs_usb loaded and adapter plugged in, check what interface showed up:

```bash
dmesg | grep -i can
ip link show
```

A new can0 (or can1/can2) interface should appear.

One thing to watch for on the AGX Orin devkit: the onboard CAN controllers sometimes already grab can0/can1. If those are enabled, the USB adapter might come in as can2 instead. Whatever name it lands on, keep note of it, the same name gets used when setting up the ROS2 bridge in Section 6.

Bring it up with the bus's bitrate (500 kbit/s here, change to match the actual bus):

```bash
sudo ip link set can0 up type can bitrate 500000
```

(swap in can2 etc depending on what showed up earlier)

Check it's up:

```bash
ip -details link show can0
```

---

## 5. Testing with can-utils

Confirm the bus itself works. This rules out kernel/driver/wiring problems before getting into the ROS2 layer for no reason.

### Receive / monitor traffic

```bash
candump can0
```

Frames scrolling by means the adapter and bus are good end to end.

### Send a test frame

```bash
cansend can0 123#DEADBEEF
```

### Loopback self-test (no external bus needed)

To check the adapter/driver chain works without anything plugged into the CAN side:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 loopback on
sudo ip link set can0 up
candump can0 &
cansend can0 123#1122334455667788
```

The frame just sent should get echoed straight back.

---

## 6. Bringing CAN into ROS2 Jazzy

Bus confirmed working with can-utils, now bridge it into ROS2 with ros2_socketcan (installed back in Section 1).

### 6.1 Launch bridge

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

Use whatever interface name showed up in Section 4. can2 if the onboard controllers took can0/can1.

This brings up two lifecycle nodes:

- socket_can_receiver_node, publishes incoming frames to the from_can_bus topic (can_msgs/msg/Frame)
- socket_can_sender_node, listens on to_can_bus and writes frames out to the bus

### 6.2 Verify

```bash
ros2 topic list
ros2 topic echo /from_can_bus
```

In another terminal, send a frame the same way as Section 5:

```bash
cansend can0 123#DEADBEEF
```

It should show up in the echo terminal as a can_msgs/Frame message.

Sending from ROS2 onto the bus works the same way in reverse:

```bash
ros2 topic pub /to_can_bus can_msgs/msg/Frame "{id: 0x123, is_rtr: false, is_extended: false, is_error: false, dlc: 8, data: [0,0,0,0,0,0,0,0]}"
```

Check it landed with candump can0 running in another terminal.
