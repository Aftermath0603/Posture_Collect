#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "esp_task_wdt.h"
#include <math.h>
#include <sys/time.h>
#include "mpu6050_filter.h"

#define TAG "MPU6050_COLLECT"

/* ==================== 一、基础采样参数固定 ==================== */
/* 硬件配置 */
#define I2C_MASTER_SDA_IO           GPIO_NUM_7      // I2C SDA引脚
#define I2C_MASTER_SCL_IO           GPIO_NUM_8      // I2C SCL引脚
#define I2C_MASTER_NUM              I2C_NUM_0       // I2C控制器编号
#define I2C_MASTER_FREQ_HZ          400000          // I2C时钟频率 400kHz
#define I2C_MASTER_TIMEOUT_MS       500             // I2C通信超时时间

/* MPU6050 寄存器 */
#define MPU6050_ADDRESS             0x68
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

/* ==================== 关键改动1: 10Hz 10秒单组采样计数逻辑 ==================== */
/* 采集参数 */
#define CALIBRATION_DURATION_MS     2000            // 自调零时长 (ms)
#define SAMPLE_INTERVAL_MS          100             // 采样间隔 (ms) = 10Hz
#define SINGLE_COLLECTION_MS        5000           // 单组采集时长:5秒
#define TOTAL_SAMPLES               50              // 有效数据组阈值: 累计50组有效后结束
#define SAMPLES_PER_GROUP           (SINGLE_COLLECTION_MS / SAMPLE_INTERVAL_MS)  // 每组100条数据
#define SPIFFS_FILE_PATH            "/spiffs/imu_data.csv"
#define TEMP_GROUP_FILE             "/spiffs/temp_group.csv"  // 临时文件路径，消除大文件拷贝

/* 软件滤波参数 */
#define LPF_ALPHA                   0.15f           // 低通滤波系数
#define ANGLE_LPF_ALPHA             0.10f           // 姿态角滤波系数

/* 动态零偏补偿参数 */
#define GYRO_AUTO_ZERO_COUNT        100             // 连续静止次数触发微调零
#define MOTION_THRESHOLD_GYRO       50.0f           // gyro运动阈值 (LSB)
#define MOTION_THRESHOLD_ACCEL      0.02f           // 加速度运动阈值 (g)
#define GYRO_ADJUST_RATE            0.05f           // 零偏调整速率

/* 温度补偿系数 */
#define AX_TEMP_COEFF               2.0f
#define AY_TEMP_COEFF               1.2f
#define AZ_TEMP_COEFF               0.8f
#define GX_TEMP_COEFF               0.6f
#define GY_TEMP_COEFF               0.6f
#define GZ_TEMP_COEFF               0.6f

/* ==================== 全局变量 ==================== */
float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
float gyro_x_offset  = 0, gyro_y_offset  = 0, gyro_z_offset  = 0;
float temp_ref = 0;
float ax_temp_bias = 0, ay_temp_bias = 0, az_temp_bias = 0;
float gx_temp_bias = 0, gy_temp_bias = 0, gz_temp_bias = 0;

/* ==================== 关键改动2: 新增全局有效组计数变量 ==================== */
int valid_group_count = 0; // 记录已采纳的有效组数

/* ==================== 读取单个字符 ==================== */
static char read_char(void) {
    return (char)getchar();
}

/* ==================== 等待启动指令 ==================== */
static bool wait_start_command(void) {
    printf("当前已收集 %d/%d 组有效数据，请输入 s 开始本次采集\n", 
           valid_group_count, TOTAL_SAMPLES);
    while (1) {
        char c = read_char();
        if (c == 's' || c == 'S') {
            printf("收到启动指令，开始采集...\n");
            return true;
        }
    }
}

/* ==================== 等待保存/舍弃指令 ==================== */
static bool wait_save_command(void) {
    printf("单组采集完成，当前已收集 %d/%d 组有效数据，请输入 y 保留本次，n 删除本次\n", 
           valid_group_count, TOTAL_SAMPLES);
    while (1) {
        char c = read_char();
        if (c == 'y' || c == 'Y') {
            printf("已保留本次数据\n");
            return true;
        } else if (c == 'n' || c == 'N') {
            printf("已舍弃本次数据\n");
            return false;
        }
    }
}

/* ==================== 将临时文件内容追加到主文件 ==================== */
static esp_err_t append_temp_to_main(void) {
    FILE *temp_file = fopen(TEMP_GROUP_FILE, "r");
    if (temp_file == NULL) {
        printf("临时文件不存在\n");
        return ESP_FAIL;
    }
    
    // 检查主文件是否存在，不存在则新建并写表头
    FILE *main_file = fopen(SPIFFS_FILE_PATH, "r");
    bool main_file_exists = (main_file != NULL);
    if (main_file != NULL) {
        fclose(main_file);
    }
    
    // 打开主文件，追加模式
    main_file = fopen(SPIFFS_FILE_PATH, "a");
    if (main_file == NULL) {
        printf("无法打开主文件\n");
        fclose(temp_file);
        return ESP_FAIL;
    }
    
    // 如果是新文件，先写入表头
    if (!main_file_exists) {
        fprintf(main_file, "sample_num,timestamp_ms,acc_x(g),acc_y(g),acc_z(g),gyro_x(LSB),gyro_y(LSB),gyro_z(LSB)\n");
        printf("新建主文件，已写入表头\n");
    }
    
    // 将临时文件内容逐行复制到主文件
    char buf[256];
    while (fgets(buf, sizeof(buf), temp_file) != NULL) {
        fprintf(main_file, "%s", buf);
    }
    
    fclose(temp_file);
    fclose(main_file);
    printf("临时文件已追加到主文件\n");
    return ESP_OK;
}

/* ==================== I2C 初始化 ==================== */
static esp_err_t i2c_master_init(void) {
    printf("初始化 I2C...\n");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        printf("I2C 初始化失败\n");
        return err;
    }
    printf("I2C 初始化成功\n");
    return ESP_OK;
}

/* ==================== MPU6050 基础操作 ==================== */
static esp_err_t mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDRESS, buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDRESS, &reg, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

/* ==================== MPU6050 初始化 ==================== */
static esp_err_t mpu6050_init(void) {
    printf("初始化 MPU6050...\n");
    esp_err_t ret;
    
    ret = mpu_write(MPU6050_RA_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        printf("MPU6050 唤醒失败\n");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ret = mpu_write(MPU6050_RA_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;
    
    ret = mpu_write(MPU6050_RA_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = mpu_write(MPU6050_RA_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;
    
    printf("MPU6050 初始化成功\n");
    return ESP_OK;
}

/* ==================== 读取温度 ==================== */
float mpu_read_temp(void) {
    uint8_t buf[2];
    mpu_read(MPU6050_RA_TEMP_OUT_H, buf, 2);
    int16_t raw_temp = (buf[0] << 8) | buf[1];
    return (raw_temp / 340.0f) + 36.53f;
}

/* ==================== SPIFFS 初始化 ==================== */
static esp_err_t spiffs_init(void) {
    printf("初始化 SPIFFS 文件系统...\n");
    
    /* 关键改动: 修复 partition_label 为 "spiffs" */
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "spiffs",   // 匹配分区表标签
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            printf("SPIFFS 挂载失败，正在格式化...\n");
            ret = esp_spiffs_format(NULL);
            if (ret != ESP_OK) {
                printf("SPIFFS 格式化失败\n");
                return ret;
            }
            ret = esp_vfs_spiffs_register(&conf);
        }
        if (ret != ESP_OK) {
            printf("SPIFFS 初始化失败: %s\n", esp_err_to_name(ret));
            return ret;
        }
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret == ESP_OK) {
        printf("SPIFFS 总容量: %d 字节, 已使用: %d 字节\n", total, used);
    }
    
    printf("SPIFFS 初始化成功\n");
    return ESP_OK;
}

/* ==================== 获取时间戳 ==================== */
static int64_t get_timestamp_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + (int64_t)tv.tv_usec / 1000;
}

/* ==================== MPU6050 校准 ==================== */
void mpu6050_calibrate(void) {
    printf("=====================================\n");
    printf("  MPU6050 校准中，请保持静止...\n");
    printf("=====================================\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0, gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int total = CALIBRATION_DURATION_MS / SAMPLE_INTERVAL_MS;
    for (int i = 0; i < total; i++) {
        uint8_t buf[14];
        mpu_read(MPU6050_RA_ACCEL_XOUT_H, buf, 14);
        
        int16_t ax = (buf[0] << 8) | buf[1];
        int16_t ay = (buf[2] << 8) | buf[3];
        int16_t az = (buf[4] << 8) | buf[5];
        int16_t gx = (buf[8] << 8) | buf[9];
        int16_t gy = (buf[10] << 8) | buf[11];
        int16_t gz = (buf[12] << 8) | buf[13];
        
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az - 16384; 
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        
        if (i % (total / 5) == 0) {
            printf("校准进度: %d%%\n", i * 100 / total);
        }
        
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
    
    accel_x_offset = (float)ax_sum / total;
    accel_y_offset = (float)ay_sum / total;
    accel_z_offset = (float)az_sum / total;
    gyro_x_offset  = (float)gx_sum / total;
    gyro_y_offset  = (float)gy_sum / total;
    gyro_z_offset  = (float)gz_sum / total;
    
    temp_ref = mpu_read_temp();
    printf("=====================================\n");
    printf("  校准完成！\n");
    printf("  Accel offset: [%.2f, %.2f, %.2f] LSB\n", accel_x_offset, accel_y_offset, accel_z_offset);
    printf("  Gyro offset:  [%.2f, %.2f, %.2f] LSB\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);
    printf("=====================================\n");
}

/* ==================== 温度补偿 ==================== */
void update_temp_comp(void) {
    float new_temp = mpu_read_temp();
    float dt = new_temp - temp_ref;
    
    ax_temp_bias = dt * AX_TEMP_COEFF;   
    ay_temp_bias = dt * AY_TEMP_COEFF;
    az_temp_bias = dt * AZ_TEMP_COEFF;
    gx_temp_bias = dt * GX_TEMP_COEFF;
    gy_temp_bias = dt * GY_TEMP_COEFF;
    gz_temp_bias = dt * GZ_TEMP_COEFF;
}

/* ==================== 读取并校准数据 ==================== */
void mpu6050_get_calibrated(float *out_ax, float *out_ay, float *out_az,
                             float *out_gx, float *out_gy, float *out_gz) {
    uint8_t buf[14];
    mpu_read(MPU6050_RA_ACCEL_XOUT_H, buf, 14);
    
    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];
    
    *out_ax = ((float)ax - accel_x_offset - ax_temp_bias) / 16384.0f;
    *out_ay = ((float)ay - accel_y_offset - ay_temp_bias) / 16384.0f;
    *out_az = ((float)az - accel_z_offset - az_temp_bias) / 16384.0f;
    
    *out_gx = (float)gx - gyro_x_offset - gx_temp_bias;
    *out_gy = (float)gy - gyro_y_offset - gy_temp_bias;
    *out_gz = (float)gz - gyro_z_offset - gz_temp_bias;
}

/* ==================== 主函数 ==================== */
void app_main(void) {
    esp_err_t ret;
    
    /* 直接关闭看门狗 */
    esp_task_wdt_deinit();
    
    printf("\n");
    printf("=====================================\n");
    printf("  ESP32-P4 MPU6050 数据采集系统\n");
    printf("  目标: 收集 %d 组有效数据\n", TOTAL_SAMPLES);
    printf("=====================================\n");
    printf("\n");
    
    /* ==================== 初始化阶段 ==================== */
    /* 1. 初始化 I2C */
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        printf("程序初始化失败，停止运行\n");
        while(1) { vTaskDelay(1000); }
    }
    
    /* 2. 初始化 MPU6050 */
    ret = mpu6050_init();
    if (ret != ESP_OK) {
        printf("程序初始化失败，停止运行\n");
        while(1) { vTaskDelay(1000); }
    }
    
    /* 3. 初始化滤波 */
    filter_init();
    
    /* 4. 初始化 SPIFFS */
    ret = spiffs_init();
    if (ret != ESP_OK) {
        printf("程序初始化失败，停止运行\n");
        while(1) { vTaskDelay(1000); }
    }
    
    /* ==================== 步骤1: 调零只需一次 ==================== */
    mpu6050_calibrate();
    
    /* ==================== 主循环 ==================== */
    while (valid_group_count < TOTAL_SAMPLES) {
        /* ==================== 关键改动3: valid_group_count 满50组终止整套采集 ==================== */
        
        /* ==================== 步骤2: 等待启动指令 ==================== */
        if (!wait_start_command()) {
            continue;
        }
        
        /* ==================== 步骤3: 直接打开临时文件开始采集，无大文件拷贝 ==================== */
        printf("打开临时数据文件...\n");
        FILE *temp_file = fopen(TEMP_GROUP_FILE, "w");  // 模式w清空临时文件
        if (temp_file == NULL) {
            printf("无法打开临时文件，停止本次采集\n");
            continue;
        }
        
        /* ==================== 步骤4: 单组5秒10Hz采集循环 ==================== */
        printf("开始第 %d/%d 组采集，共 %d 条数据...\n", 
               valid_group_count + 1, TOTAL_SAMPLES, SAMPLES_PER_GROUP);
        
        float ax, ay, az, gx, gy, gz;
        float lpf_ax = 0.0f, lpf_ay = 0.0f, lpf_az = 1.0f; 
        float lpf_roll = 0.0f, lpf_pitch = 0.0f, lpf_yaw = 0.0f;
        float prev_ax = 0, prev_ay = 0, prev_az = 0;
        float roll, pitch, yaw;
        float dt = SAMPLE_INTERVAL_MS / 1000.0f;
        int stationary_counter = 0;
        int sample_count = 0;
        int64_t collection_start_time = get_timestamp_ms();
        
        while (sample_count < SAMPLES_PER_GROUP) {
            int64_t current_time = get_timestamp_ms();
            int64_t elapsed_ms = current_time - collection_start_time;
            
            /* 读取数据 */
            mpu6050_get_calibrated(&ax, &ay, &az, &gx, &gy, &gz);
            
            /* 软件滤波 */
            lpf_ax = LPF_ALPHA * ax + (1.0f - LPF_ALPHA) * lpf_ax;
            lpf_ay = LPF_ALPHA * ay + (1.0f - LPF_ALPHA) * lpf_ay;
            lpf_az = LPF_ALPHA * az + (1.0f - LPF_ALPHA) * lpf_az;
            
            /* 运动检测用于动态零偏 */
            float delta_accel = sqrt(pow(lpf_ax - prev_ax, 2) + pow(lpf_ay - prev_ay, 2) + pow(lpf_az - prev_az, 2));
            float gyro_mag = sqrt(pow(gx, 2) + pow(gy, 2) + pow(gz, 2));
            
            prev_ax = lpf_ax; prev_ay = lpf_ay; prev_az = lpf_az;
            
            /* 动态零偏补偿 */
            if (gyro_mag < MOTION_THRESHOLD_GYRO && delta_accel < MOTION_THRESHOLD_ACCEL) { 
                stationary_counter++;
                if (stationary_counter > GYRO_AUTO_ZERO_COUNT) {
                    gyro_x_offset += gx * GYRO_ADJUST_RATE;
                    gyro_y_offset += gy * GYRO_ADJUST_RATE;
                    gyro_z_offset += gz * GYRO_ADJUST_RATE;
                    stationary_counter = 0;
                }
            } else {
                stationary_counter = 0;
            }
            
            /* 姿态解算 */
            float gx_rad = (gx / 131.0f) * M_PI / 180.0f;
            float gy_rad = (gy / 131.0f) * M_PI / 180.0f;
            float gz_rad = (gz / 131.0f) * M_PI / 180.0f;
            
            mahony_update(lpf_ax, lpf_ay, lpf_az, gx_rad, gy_rad, gz_rad, dt);
            get_euler_angles(&roll, &pitch, &yaw);
            
            /* 姿态角二次滤波 */
            lpf_roll  = ANGLE_LPF_ALPHA * roll  + (1.0f - ANGLE_LPF_ALPHA) * lpf_roll;
            lpf_pitch = ANGLE_LPF_ALPHA * pitch + (1.0f - ANGLE_LPF_ALPHA) * lpf_pitch;
            lpf_yaw   = ANGLE_LPF_ALPHA * yaw   + (1.0f - ANGLE_LPF_ALPHA) * lpf_yaw;
            
            /* 写入临时文件 */
            int64_t timestamp = elapsed_ms;
            fprintf(temp_file, "%d,%lld,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f\n",
                    sample_count + 1, timestamp, lpf_ax, lpf_ay, lpf_az, gx, gy, gz);
            
            /* 串口打印 */
            printf("[%d/%d] ax=%.4f ay=%.4f az=%.4f gx=%.1f gy=%.1f gz=%.1f\n", 
                   sample_count + 1, SAMPLES_PER_GROUP, lpf_ax, lpf_ay, lpf_az, gx, gy, gz);
            
            /* 刷新文件 */
            if (sample_count % 10 == 0) {
                fflush(temp_file);
            }
            
            /* 更新温度补偿 */
            update_temp_comp();
            
            /* 计数器增加 */
            sample_count++;
            
            /* 延时保持采样间隔 */
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
        }
        
        /* ==================== 步骤5: 关闭临时文件 ==================== */
        fclose(temp_file);
        printf("单组采集完成，已关闭临时文件\n");
        
        /* ==================== 关键改动2: 每组结束单独y/n确认、删除舍弃组数据 ==================== */
        bool save = wait_save_command();
        
        if (save) {
            /* 保留本组：将临时文件追加到主文件 */
            printf("正在将临时数据追加到主文件...\n");
            if (append_temp_to_main() == ESP_OK) {
                valid_group_count++;
                printf("有效组数 +1，当前已收集 %d/%d 组\n", valid_group_count, TOTAL_SAMPLES);
            }
            /* 追加完成后删除临时文件 */
            remove(TEMP_GROUP_FILE);
        } else {
            /* 舍弃本组：直接删除临时文件，主文件完全不改动 */
            printf("正在删除临时数据文件...\n");
            remove(TEMP_GROUP_FILE);
            printf("舍弃本组数据，有效组数不变，当前 %d/%d 组\n", valid_group_count, TOTAL_SAMPLES);
        }
        
        /* ==================== 关键改动3: 检查是否满50组终止 ==================== */
        if (valid_group_count >= TOTAL_SAMPLES) {
            printf("\n=====================================\n");
            printf("  已集齐50组有效数据，采集流程全部结束！\n");
            printf("  按 Ctrl+] 进入 VFS 控制台导出完整CSV\n");
            printf("  按复位键才能重新从头采集整套流程\n");
            printf("=====================================\n\n");
            
            /* 永久常驻循环 */
            while(1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        
        printf("\n准备进入下一组采集...\n\n");
    }
}
