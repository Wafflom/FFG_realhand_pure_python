# RealForce 手套遥操 L6 / L20 流程

默认连接：

- 左手套：`/dev/ttyUSB0`
- 右手套：`/dev/ttyUSB1`
- 左手：`can0`
- 右手：`can1`

下面所有命令都在仓库根目录 `linkerhand_telop_python` 下运行。

## 1. 开启 USB 口

```bash
ls -l /dev/ttyUSB0
ls -l /dev/ttyUSB1

sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

## 2. 开启 CAN 口

L6 / L20 使用 1 Mbps：

```bash
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

ip -details link show can0
ip -details link show can1
```

正常应看到 `UP` 和 `can state ERROR-ACTIVE`。

## 3. 标定

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py \
  --calibrate-realforce \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l6 \
  --right-model l6 \
  --calibration-output realhand_pure_python/config/calibration_l6_dual.yml \
  --calibration-sample-seconds 3 \
  --calibration-ready-seconds 8
```

根据提示依次摆好 `original`、`opose`、`fist` 三个姿态。采集完成后会生成：

```text
realhand_pure_python/config/calibration_l6_dual.yml
```

## 4. 运行遥操

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py \
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

## 5. 如果使用 L20

L20 不需要修改 Python 代码，只需要把模型参数从 `l6` 改成 `l20`。

### 5.1 L20 标定

建议给 L20 单独保存一份标定文件：

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py \
  --calibrate-realforce \
  --left-port /dev/ttyUSB0 \
  --right-port /dev/ttyUSB1 \
  --left-model l20 \
  --right-model l20 \
  --calibration-output realhand_pure_python/config/calibration_l20_dual.yml \
  --calibration-sample-seconds 3 \
  --calibration-ready-seconds 8
```

采集完成后会生成：

```text
realhand_pure_python/config/calibration_l20_dual.yml
```

### 5.2 L20 运行遥操

```bash
conda run --no-capture-output -n gloveTeleop python test_glove_teleop.py \
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

如果实际硬件是 G20，把上面命令里的模型改成 `g20`：

```text
--left-model g20
--right-model g20
--left-sdk-model g20
--right-sdk-model g20
```
