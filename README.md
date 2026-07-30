# RealHand Pure Python Teleop

这是一个可以独立上传到 GitHub 的 RealHand 纯 Python 遥操目录。它不依赖
ROS，用 RealForce 或 RealMCG 数据做 retarget，并可通过官方 RealHand Python
SDK 走 SocketCAN 控制 L6、L20、G20 手。

## 目录内容

- `realhand_pure_python/`：纯 Python 包，包含 RealForce/RealMCG 读取、
  retarget、标定加载、URDF/config 解析和 SDK 指令转换。
- `test_glove_teleop.py`：主要入口，支持扫描串口、RealForce 标定、串口遥操、
  MCG 遥操和 `--sdk-send` 发 CAN。
- `control_l20_sdk.py`：直接检查、打开、发送 L20/G20 姿态的 SDK 辅助脚本。
- `dump_serial.py`：USB 串口原始数据调试脚本。
- `REALFORCE_L6_TELEOP_README.md`：L6/L20 的标定和运行命令流程。
- `environment-gloveTeleop.yml`：已验证的
  `gloveTeleop` Conda 环境。

## gloveTeleop 环境安装

下面命令都在仓库根目录运行。推荐使用 Conda 创建名为 `gloveTeleop` 的环境：

```bash
conda env create -f environment-gloveTeleop.yml
conda activate gloveTeleop
python -m pip install -e .
```

如果本机已经有 `gloveTeleop` 环境，用下面命令更新依赖：

```bash
conda env update -n gloveTeleop -f environment-gloveTeleop.yml --prune
conda activate gloveTeleop
python -m pip install -e .
```

验证环境和入口脚本：

```bash
python -c "import realhand_pure_python; print('realhand_pure_python ok')"
python test_glove_teleop.py --help
python test_glove_teleop.py --scan
```

如果当前 shell 没有 `conda activate gloveTeleop`，所有 Python 命令都可以改成：

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py --help
```

`environment-gloveTeleop.yml` 会安装官方 RealHand Python SDK。只有使用
`--sdk-send` 或 `control_l20_sdk.py --send` 真实发送 CAN 指令时才必须连接 SDK
和 CAN 硬件。

也可以用 venv：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install -e .
```

## 快速检查

在本目录下运行：

```bash
python test_glove_teleop.py --help
python test_glove_teleop.py --scan
```

默认双手套串口：

```text
/dev/ttyUSB0
/dev/ttyUSB1
```

未激活环境时：

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py --scan
```

## USB 和 CAN

USB 权限：

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

开启 `can0` / `can1`，波特率 1 Mbps：

```bash
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

ip -details link show can0
ip -details link show can1
```

正常应看到 `UP` 和 `can state ERROR-ACTIVE`。

## L6 标定

```bash
python test_glove_teleop.py \
  --calibrate-realforce \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l6 \
  --right-model l6 \
  --calibration-output realhand_pure_python/config/calibration_l6_dual.yml \
  --calibration-sample-seconds 3 \
  --calibration-ready-seconds 8
```

根据提示依次摆好 `original`、`opose`、`fist` 三个姿态。

## L6 运行

```bash
python test_glove_teleop.py \
  --serial \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l6 \
  --right-model l6 \
  --calibration-path realhand_pure_python/config/calibration_l6_dual.yml \
  --no-load-sample-calibration \
  --sdk-send \
  --left-sdk-model l6 \
  --right-sdk-model l6 \
  --left-can can0 \
  --right-can can1 \
  --send-hz 30 \
  --seconds 999999
```

## L20 标定

```bash
python test_glove_teleop.py \
  --calibrate-realforce \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l20 \
  --right-model l20 \
  --calibration-output realhand_pure_python/config/calibration_l20_dual.yml \
  --calibration-sample-seconds 3 \
  --calibration-ready-seconds 8
```

## L20 运行

```bash
python test_glove_teleop.py \
  --serial \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l20 \
  --right-model l20 \
  --calibration-path realhand_pure_python/config/calibration_l20_dual.yml \
  --no-load-sample-calibration \
  --sdk-send \
  --left-sdk-model l20 \
  --right-sdk-model l20 \
  --left-can can0 \
  --right-can can1 \
  --send-hz 30 \
  --seconds 999999
```

如果实际硬件是 G20，把 retarget 和 SDK 模型都改成 `g20`。

## SDK 检查

不发 CAN，只看将要发送的姿态：

```bash
python control_l20_sdk.py --action open
```

连接 SDK 并读取状态：

```bash
python control_l20_sdk.py --action check --send --read-state
```

发送打开手的姿态：

```bash
python control_l20_sdk.py --action open --send
```

## 注意

- 标定文件和手套、手、左右串口分配、佩戴方式强相关，换设备或换佩戴方式后建议重新标定。
- `--baudrate 0` 是默认值，会自动扫描 RealForce 波特率。
- 如果串口打开后没有数据，可以加 `--serial-debug` 看诊断信息。
- 更底层的 Python API 示例在 `realhand_pure_python/README.md`。
