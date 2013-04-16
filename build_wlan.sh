ANDROID_DIR=$1

export PATH="$ANDROID_DIR/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$ANDROID_DIR/out/host/linux-x86/bin/:$PATH"
export KERNEL_DIR=$ANDROID_DIR/kernel/samsung/latona

export HOST_PLATFORM=omap
export ARCH=arm
export CROSS_COMPILE=arm-eabi-

cd ./platforms/os/linux

make clean
make CONFIG_FRAME_WARN=8192
