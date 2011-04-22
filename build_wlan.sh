#Please modify KERNEL_DIR, BUILD_DIR
export KERNEL_DIR=kernel_path

export HOST_PLATFORM=msm
export ARCH=arm
export CROSS_COMPILE=arm-eabi-

cd ./platforms/os/linux

make clean
make CONFIG_FRAME_WARN=8192
