# README.md
# ESP32-P4 MPU6050 数据采集

### 核心采集规则
1. 采样频率10Hz，单次采集固定10秒，每组生成100条六轴数据；
2. 每采集完一组，人工确认是否保存，仅输入`y`才计入有效数据；
3. 累计采集满50组有效数据后，自动停止整套采集流程；
4. SPIFFS文件系统存储CSV，支持终端清空分区、控制台导出数据；
5. 内置MPU6050静态校准、动态零偏、温度补偿、Mahony姿态解算、低通滤波。

## 硬件清单
1. ESP32-P4开发板
2. MPU6050 六轴加速度陀螺仪模块
3. 杜邦线若干

## 一、硬件接线说明
| ESP32-P4引脚 | MPU6050 |
|------------|---------|
| GPIO7      | SDA     |
| GPIO8      | SCL     |
| 5V(VBUS)   | VCC     |
| GND        | GND     |
> 注意：有些MPU6050VCC接3V3

## 二、编译烧录步骤
1. 克隆仓库到本地
```bash
git clone 你的仓库地址
cd mpu6050_collect
```
2. 配置串口（替换为你的开发板COM号）
```bash
idf.py set-target esp32p4
idf.py -p COM16 flash
```
3. 打开串口监视器查看系统日志
```bash
idf.py -p COM16 monitor
```

## 三、完整采集操作流程

### 1. 上电流程
1 上电自动执行MPU6050校准，保持模块静止；
2 校准完成，串口提示：输入`s`开始10秒单组采集；

### 2. 采集交互指令
- 每组数据保存到临时文件，采集完询问用户是否保存
- `y` / `Y`：本组数据有效，追加保存到CSV，有效组数+1；
- `n` / `N`：舍弃本组，临时文件直接删除，不污染历史数据；

### 3. 终止条件
累计50组有效数据后，采集流程结束，复位可重新全套采集；若采集过程异常中止，文件还会保存在esp中，不会丢失

## 四、SPIFFS分区操作

### 1. 设置SPIFFS分区
在partition.csv设置SPIFFS分区大小，可根据实际FLASH大小更改，但必须保证各分区连续

### 2. 清空SPIFFS分区（删除所有CSV数据，不重烧程序）
将COM16替换为你的串口，在项目ESP-IDF终端执行：
```powershell
python $env:IDF_PATH/components/partition_table/parttool.py --port COM16 erase_partition --partition-name=spiffs
```
执行完成后重启开发板，分区自动格式化，所有历史数据全部清除。

### 3. 全片擦除（会删除固件，擦完需重新烧录）
```powershell
idf.py -p COM16 erase-flash
```

### 4、CSV文件导出方法
1. 打开`idf.py monitor`串口监视器；
2 按下快捷键 `Ctrl + ]` 进入ESP VFS控制台；
3 执行文件导出指令：
```
get /spiffs/imu_data.csv 本地保存路径
```
示例Windows导出：
```
get /spiffs/imu_data.csv C:\data\imu_record.csv
```
4 导出完成关闭控制台，回到正常监视器界面，文件格式：CSV表头：`sample_num,timestamp_ms,acc_x(g),acc_y(g),acc_z(g),gyro_x(LSB),gyro_y(LSB),gyro_z(LSB)`
