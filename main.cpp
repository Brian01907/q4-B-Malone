/* Includes */
#include "mbed.h"
#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include "LSM6DSLSensor.h"
#include "lis3mdl_class.h"
#include "VL53L0X.h"
#include "UnbufferedSerial.h"

static DevI2C devI2c(PB_11, PB_10);
static LPS22HBSensor press_temp(&devI2c);
static HTS221Sensor hum_temp(&devI2c);
static LSM6DSLSensor acc_gyro(&devI2c, 0xD4, D4, D5);
static LIS3MDL magnetometer(&devI2c, 0x3C);
static DigitalOut shutdown_pin(PC_6);
static VL53L0X range(&devI2c, &shutdown_pin, PC_7, 0x52);

// Serial port
UnbufferedSerial serial(USBTX, USBRX);

// Functions
void print_t_rh() {
    float value1, value2;
    hum_temp.get_temperature(&value1);
    hum_temp.get_humidity(&value2);

    printf("LPS22HB: [temp] %.2f C, [press] %.2f mbar\r\n", value1, value2);
}

void print_mag() {
    int32_t axes[3];
    magnetometer.get_m_axes(axes);
    printf("LIS3MDL [mag/mgauss]:    %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
}

void print_accel() {
    int32_t axes[3];
    acc_gyro.get_x_axes(axes);
    printf("LSM6DSL [acc/mg]:        %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
}

void print_gyro() {
    int32_t axes[3];
    acc_gyro.get_g_axes(axes);
    printf("LSM6DSL [gyro/mdps]:     %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
}

void print_distance() {
    uint32_t distance;
    int status = range.get_distance(&distance);
    if (status == VL53L0X_ERROR_NONE) {
        printf("VL53L0X [mm]:            %6ld\r\n", distance);
    } else {
        printf("VL53L0X [mm]:                --\r\n");
    }
}

int main() {
    uint8_t id;

    hum_temp.init(NULL);
    press_temp.init(NULL);
    magnetometer.init(NULL);
    acc_gyro.init(NULL);
    range.init_sensor(0x52);

    hum_temp.enable();
    press_temp.enable();
    acc_gyro.enable_x();
    acc_gyro.enable_g();

    printf("\033[2J\033[20A");
    printf("\r\n--- Starting new run ---\r\n\r\n");

    hum_temp.read_id(&id);
    printf("HTS221  humidity & temperature    = 0x%X\r\n", id);

    press_temp.read_id(&id);
    printf("LPS22HB pressure & temperature    = 0x%X\r\n", id);
    magnetometer.read_id(&id);
    printf("LIS3MDL magnetometer              = 0x%X\r\n", id);
    acc_gyro.read_id(&id);
    printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

    printf("\n\r--- Reading sensor values ---\n\r");

    while (1) {
        if (serial.readable()) {
            char c;
            serial.read(&c, 1);

            switch (c) {
                case 'a':
                    print_accel();
                    break;
                case 'm':
                    print_mag();
                    break;
                case 't':
                    print_t_rh();
                    break;
                case 'g':
                    print_gyro();
                    break;
                case 'd':
                    print_distance();
                    break;
                default:
                    break;
            }
        }

        wait_us(500000);
    }
}
