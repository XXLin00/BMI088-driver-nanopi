//
//writen by XXLin
//

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

//user pin
#define PIN_CS_A 363    //CSB1
#define PIN_CS_G 203    //CSB2

#define PIN_SCK 14
#define PIN_MOSI 15
#define PIN_MISO 16

#define PIN_INT_A 199   //INT1
#define PIN_INT_G 198   //INT3



/*
Chang {  "WiringNP/wiringPi/wiringPiSPI.c"   ------>  (*spiDev1  = "/dev/spidev0.1")   }
to (*spiDev1 = "/dev/spidev1.0") and rebuild WiringNP!
*/
#define SPI_PORT 1      //Use npi-config to enable SPI1
#define READ_BIT 0x80   //


//Sensor informatin
#define BMI08X_SPI_RD_MASK                       UINT8_C(0x80)
#define BMI08X_REG_ACCEL_X_LSB                   UINT8_C(0x12)
#define BMI08X_REG_GYRO_X_LSB                    UINT8_C(0x02)
#define BMI08X_REG_TEMP_MSB                      UINT8_C(0x22)
#define BMI08X_GYRO_RANGE_2000_DPS               UINT8_C(0x00)


#define GRAVITY_EARTH  (9.80665f)


struct bmi08x_sensor_data
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;
};


int8_t bmi08a_get_data(struct bmi08x_sensor_data *accel);
int8_t bmi08g_get_data(struct bmi08x_sensor_data *gyro);



/*! variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;

/*! variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;



//改cs
static inline void cs_select(){
    asm volatile("nop \n nop \n nop");
    pinMode(PIN_CS,OUTPUT);
    digitalWrite(PIN_CS,0); //active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(){
    asm volatile("nop \n nop \n nop");
    pinMode(PIN_CS,OUTPUT);
    digitalWrite(PIN_CS,1); //active high
    asm volatile("nop \n nop \n nop");
}


//
static void IMU088_reset(){
    unsigned char buf[]={0x7E,0xb6};
    cs_select();
    wiringPiSPIDataRW(SPI_PORT,buf,2);
    cs_deselect();
    usleep(50);
}


//初始化
void register_init()
{

}

//加对于int的判断(4.7New Data Interrupt)



//重写
static void read_register(uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    unsigned char data[2];
    data[0] = reg_addr;
    data[1] = *reg_data;
    cs_select();
    wiringPiSPIDataRW(SPI_PORT,data,len);
    usleep(10);
    cs_deselect();
    usleep(10);

}






int8_t bmi08a_get_data(struct bmi08x_sensor_data *accel)
{

    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Read accel sensor data */
    read_register(BMI08X_REG_ACCEL_X_LSB, data, 6);


    lsb = data[0];
    msb = data[1];
    msblsb = (msb << 8) | lsb;
    accel->x = ((int16_t) msblsb); /* Data in X axis */

    lsb = data[2];
    msb = data[3];
    msblsb = (msb << 8) | lsb;
    accel->y = ((int16_t) msblsb); /* Data in Y axis */

    lsb = data[4];
    msb = data[5];
    msblsb = (msb << 8) | lsb;
    accel->z = ((int16_t) msblsb); /* Data in Z axis */

    return 0;
}


int8_t bmi08g_get_data(struct bmi08x_sensor_data *gyro)
{

    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;


    /* read gyro sensor data */
    read_register(BMI08X_REG_GYRO_X_LSB, data, 6);


    lsb = data[0];
    msb = data[1];
    msblsb = (msb << 8) | lsb;
    gyro->x = (int16_t)msblsb; /* Data in X axis */

    lsb = data[2];
    msb = data[3];
    msblsb = (msb << 8) | lsb;
    gyro->y = (int16_t)msblsb; /* Data in Y axis */

    lsb = data[4];
    msb = data[5];
    msblsb = (msb << 8) | lsb;
    gyro->z = (int16_t)msblsb; /* Data in Z axis */

    return 0;
}


static int8_t bmi08a_get_sensor_temperature(int32_t *sensor_temp)
{

    uint8_t data[2] = {0};
    uint16_t msb, lsb;
    uint16_t msblsb;
    int16_t temp;

    /* Read sensor temperature */
    read_register(BMI08X_REG_TEMP_MSB, data, 2);


    msb = (data[0] << 3); /* MSB data */
    lsb = (data[1] >> 5); /* LSB data */
    msblsb = (uint16_t) (msb + lsb);

    if (msblsb > 1023)
    {
        /* Updating the msblsb */
        temp = (int16_t) (msblsb - 2048);
    }
    else
    {
        temp = (int16_t) msblsb;
    }

    /* sensor temperature */
    *sensor_temp = (temp * 125) + 23000;

    return 0;
}


/*!
 * This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float gravity;

    float half_scale = ((1 << bit_width) / 2.0f);

    gravity = (float)((GRAVITY_EARTH * val * g_range) / half_scale);

    return gravity;
}

/*!
 * This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI08X_GYRO_RANGE_2000_DPS)) * (val);
}




int main(){


    uint8_t times_to_read = 0;
    float x = 0.0, y = 0.0, z = 0.0;

    wiringPiSetup();
    printf("hello, IMU088 reading raw data from register via SPI ....\n");

    //this example will use spi at 0.5MHz.
    wiringPiSPISetup(SPI_PORT,500*1000);

    //Chip select is active-low, so we'll initialise it to a driven-high state
    pinMode(PIN_CS,OUTPUT);
    digitalWrite(PIN_CS,1);

    IMU088_reset();




    while (times_to_read < 10)
    {
        printf("\nACCEL DATA\n");
        printf("Accel data in LSB units and Gravity data in m/s^2\n");
        printf("Accel data range : 24G for BMI088\n\n");

        bmi08a_get_data(&bmi08x_accel);
        printf("ACCEL[%d]  Acc_Raw_X : %d  Acc_Raw_Y : %d   Acc_Raw_Z : %d  ",
               times_to_read,
               bmi08x_accel.x,
               bmi08x_accel.y,
               bmi08x_accel.z);

        /* Converting lsb to meter per second squared for 16 bit accelerometer at 24G range. */
        x = lsb_to_mps2(bmi08x_accel.x, 24, 16);
        y = lsb_to_mps2(bmi08x_accel.y, 24, 16);
        z = lsb_to_mps2(bmi08x_accel.z, 24, 16);

        /* Print the data in m/s2. */
        printf("\t  Acc_ms2_X = %4.2f,  Acc_ms2_Y = %4.2f,  Acc_ms2_Z = %4.2f\n", x, y, z);

        times_to_read = times_to_read + 1;
        usleep(100);
    }

    times_to_read = 0;

    while (times_to_read < 10)
    {
        printf("\n\nGYRO DATA\n");
        printf("Gyro data in LSB units and degrees per second\n");
        printf("Gyro data range : 250 dps for BMI088\n\n");

        bmi08g_get_data(&bmi08x_gyro);

        printf("GYRO:[%d]  Gyr_Raw_X : %d   Gyr_Raw_Y : %d   Gyr_Raw_Z : %d   ",
               times_to_read,
               bmi08x_gyro.x,
               bmi08x_gyro.y,
               bmi08x_gyro.z);

        /* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
        x = lsb_to_dps(bmi08x_gyro.x, 250, 16);
        y = lsb_to_dps(bmi08x_gyro.y, 250, 16);
        z = lsb_to_dps(bmi08x_gyro.z, 250, 16);

        /* Print the data in dps. */
        printf("\t  Gyr_DPS_X = %4.2f  , Gyr_DPS_Y = %4.2f  , Gyr_DPS_Z = %4.2f\n", x, y, z);

        times_to_read = times_to_read + 1;

    }

    return 0;
}
