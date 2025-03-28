![banner](.images/banner-dark-theme.png#gh-dark-mode-only)
![banner](.images/banner-light-theme.png#gh-light-mode-only)

# micro-ROS module for Raspberry Pi Pico SDK

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## About

Forked from [micro_ros_raspberry_pi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk). This repository uses the same file structure as the example SDK code but modifies the main file for use on the Krysallis Hand. This guide assumes that this repository was pulled as a submodule of the [glove_ROS](https://github.com/JustinChang04/glove_ROS) repository.

### 1. Install Pico SDK
First, make sure the Pico SDK is properly installed and configured:

```bash
# Install dependencies
sudo apt install cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk

# Configure environment
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```

### 2. Compile Example

Once the Pico SDK is ready, compile the example:

```bash
cd $HOME/glove_ROS/RPI_Master
mkdir build
cd build
cmake ..
make
```

To flash, hold the boot button, plug the USB and run:
```
cp main.uf2 /media/$USER/RPI-RP2
```

### 3. Start Micro-ROS Agent
Micro-ROS follows the client-server architecture, so you need to start the Micro-ROS Agent.
```bash
micro-ros-agent serial --dev /dev/ttyACM0 -b 115200
```

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in this repository,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

There are no known limitations.
