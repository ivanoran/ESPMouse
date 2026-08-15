#include <SPI.h>

//=========================================================
// Pins
//=========================================================

#define RESET_PIN 8
#define CS_PIN 9
#define MISO_PIN 10
#define MOSI_PIN 11
#define SCLK_PIN 12
#define MOTION_PIN 13

//=========================================================
// Register Addresses
//=========================================================

#define PRODUCT_ID          0x00        //R
#define REVISION_ID         0x01        //R
#define MOTION              0x02        //R/W
#define DELTA_X_L           0x03        //R
#define DELTA_X_H           0x04        //R
#define DELTA_Y_L           0x05        //R
#define DELTA_Y_H           0x06        //R
#define SQUAL               0x07        //R
#define RAWDATA_SUM         0x08        //R
#define MAX_RAWDATA         0x09        //R
#define MIN_RAWDATA         0x0A        //R
#define SHUTTER_L           0x0B        //R
#define SHUTTER_H           0x0C        //R

#define OBSERVATION         0x15        //R/W
#define MOTION_BURST        0x16        //R/W

#define POWER_UP_RESET      0x3A        //W
#define SHUTDOWN            0x3B        //W

#define PERFOMANCE          0x40        //R/W

#define SET_RESOLUTION      0x47        //W
#define RESOLUTION_X_L      0x48        //R/W
#define RESOLUTION_X_H      0x49        //R/W
#define RESOLUTION_Y_L      0x4A        //R/W
#define RESOLUTION_Y_H      0x4B        //R/W

#define ANGLE_SNAP          0x56        //R/W

#define RAWDATA_OUTPUT      0x58        //R
#define RAWDATA_STATUS      0x59        //R
#define RIPPLE_CONTROL      0x5A        //R/W
#define AXIS_CONTROL        0x5B        //R/W
#define MOTION_CTRL         0x5C        //R/W

#define INV_PRODUCT_ID      0x5F        //R 

#define RUN_DOWNSHIFT       0x77        //R/W
#define REST1_PERIOD        0x78        //R/W
#define REST1_DOWNSHIFT     0x79        //R/W
#define REST2_PERIOD        0x7A        //R/W
#define REST2_DOWNSHIFT     0x7B        //R/W
#define REST3_PERIOD        0x7C        //R/W
#define RUN_DOWNSHIFT_MULT  0x7D        //R/W
#define REST_DOWNSHIFT_MULT 0x7E        //R/W

#define ANGLE_TUNE1         0x0577      //R/W
#define ANGLE_TUNE2         0x0578      //R/W
#define LIFT_CONFIG         0x0C4E      //R/W

struct motion_read_data
{
    uint8_t motion;
    uint8_t observation;
    uint8_t delta_x_l;
    uint8_t delta_x_h;
    int16_t delta_x;
    uint8_t delta_y_l;
    uint8_t delta_y_h;
    int16_t delta_y;
    uint8_t squal;
    uint8_t rawdata_sum;
    uint8_t max_rawdata;
    uint8_t min_rawdata;
    uint8_t shutter_upper;
    uint8_t shutter_lower;
};

class PAW3395
{
    public:

    PAW3395(){};

    void init()
    {
        pinMode(MOTION_PIN, INPUT);

        pinMode(RESET_PIN, OUTPUT);
        digitalWrite(RESET_PIN, 1);

        pinMode(CS_PIN, OUTPUT);
        digitalWrite(CS_PIN, 1);

        _SPI = new SPIClass(HSPI);
        _SPI->begin(SCLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

        _reset();
        _power_up_init();
    };

    bool moving()
    {
        return !digitalRead(MOTION_PIN);
    };

    void img(uint8_t * data)
    {
        _write_register(0x7F, 0x00);
        _write_register(0x40, 0x80);
        while (1)
        {
            uint8_t val = _read_register(MOTION);
            if ((val & 3) == 0) break;
        };
        _write_register(0x50, 0x01);
        _write_register(0x55, 0x04);
        _write_register(0x58, 0xFF);
        while (1)
        {
            uint8_t val = _read_register(0x59);
            if ((val >> 6) == 3) break;
        };
        _write_register(0x55, 0x00);
        uint8_t frd = _read_register(0x58);
        while (1)
        {
            uint8_t val = _read_register(0x59);
            if ((val >> 7) == 1) break;
        };

        
        for (uint16_t i = 0; i < 1296; i++)
        {
            data[i] = _read_register(RAWDATA_OUTPUT);
        }

        _write_register(0x40, 0x00);
        _write_register(0x50, 0x00);
        _write_register(0x55, 0x00);
    }

    motion_read_data read_motion()
    {
        if (_burst_enabled) return read_motion_burst();
        else
        {
            motion_read_data mrd;
            mrd.motion = _read_register(MOTION);
            mrd.observation = _read_register(OBSERVATION);
            mrd.delta_x_l = _read_register(DELTA_X_L);
            mrd.delta_x_h = _read_register(DELTA_X_H);
            mrd.delta_x = (int16_t)((mrd.delta_x_h << 8) | mrd.delta_x_l);
            mrd.delta_y_l = _read_register(DELTA_Y_L);
            mrd.delta_y_h = _read_register(DELTA_Y_H);
            mrd.delta_y = (int16_t)((mrd.delta_y_h << 8) | mrd.delta_y_l);
            mrd.squal = _read_register(SQUAL);
            mrd.rawdata_sum = _read_register(RAWDATA_SUM);
            mrd.max_rawdata = _read_register(MAX_RAWDATA);
            mrd.min_rawdata = _read_register(MIN_RAWDATA);
            mrd.shutter_upper = _read_register(SHUTTER_H);
            mrd.shutter_lower = _read_register(SHUTTER_L);
            return mrd;
        };
    };

    motion_read_data read_motion_burst()
    {
        motion_read_data mrd;
        _read_register_burst();
        mrd.motion = _burst_data[0];
        mrd.observation = _burst_data[1];
        mrd.delta_x_l = _burst_data[2];
        mrd.delta_x_h = _burst_data[3];
        mrd.delta_x = (int16_t)((mrd.delta_x_h << 8) | mrd.delta_x_l);
        mrd.delta_y_l = _burst_data[4];
        mrd.delta_y_h = _burst_data[5];
        mrd.delta_y = (int16_t)((mrd.delta_y_h << 8) | mrd.delta_y_l);
        mrd.squal = _burst_data[6];
        mrd.rawdata_sum = _burst_data[7];
        mrd.max_rawdata = _burst_data[8];
        mrd.min_rawdata = _burst_data[9];
        mrd.shutter_upper = _burst_data[10];
        mrd.shutter_lower = _burst_data[11];
        return mrd;
    };

    bool read_ripple_control()
    {
        uint8_t val = _read_register(RIPPLE_CONTROL);
        return (val >> 7) & 0x01;
    }

    bool read_angle_snap()
    {
        uint8_t val = _read_register(ANGLE_SNAP);
        return (val >> 7) & 0x01;
    }

    bool read_axis_control_swapxy()
    {
        return _read_register(AXIS_CONTROL) >> 7 & 0x01;
    };

    bool read_axis_control_invertx()
    {
        return _read_register(AXIS_CONTROL) >> 6 & 0x01;
    };

    bool read_axis_control_inverty()
    {
        return _read_register(AXIS_CONTROL) >> 5 & 0x01;
    };

    bool read_lift_config()
    {
        uint8_t val = _read_register16(LIFT_CONFIG);
        return val >> 1 & 0x01;
    }
  
    uint8_t read_angle_tune_enabled()
    {
        uint8_t val = _read_register16(ANGLE_TUNE2);
        return val;
        return (val >> 7) & 0x01;
    };

    int8_t read_angle_tune()
    {
        return (int8_t)_read_register16(ANGLE_TUNE1);
    };

    uint16_t read_resolution()
    {
        uint8_t resolution_l = _read_register(RESOLUTION_X_L);
        uint8_t resolution_h = _read_register(RESOLUTION_X_H);
        return ((uint16_t)resolution_h << 8) | resolution_l;
    };

    uint8_t read_product_id() {return _read_register(PRODUCT_ID);};
    uint8_t read_inv_product_id() {return _read_register(INV_PRODUCT_ID);};
    uint8_t read_revision_id() {return _read_register(REVISION_ID);};

    void set_ripple_control(bool enable)
    {
        uint8_t old = 0x00010000;
        bitWrite(old, 7, enable);
        _write_register(RIPPLE_CONTROL, old);
    }

    void set_angle_snap(bool enable)
    {
        uint8_t old = 0x00001101;
        bitWrite(old, 7, enable);
        _write_register(ANGLE_SNAP, old);
    }

    void set_axis_control_swapxy(bool enable)
    {
        uint8_t old = _read_register(AXIS_CONTROL);
        bitWrite(old, 7, enable);
        _write_register(AXIS_CONTROL, old);
    }

    void set_axis_control_invertx(bool enable)
    {
        uint8_t old = _read_register(AXIS_CONTROL);
        bitWrite(old, 6, enable);
        _write_register(AXIS_CONTROL, old);
    }

    void set_axis_control_inverty(bool enable)
    {
        uint8_t old = _read_register(AXIS_CONTROL);
        bitWrite(old, 5, enable);
        _write_register(AXIS_CONTROL, old);
    }

    void set_lift_config(bool high)
    {
        uint8_t old = 0x00001000;
        uint8_t newval = 0;
        if (high) newval = 2;
        _write_register16(LIFT_CONFIG, old | newval);
    }

    void set_angle_tune_enabled(bool enable) // angle: -128 ~ 127
    {
        _write_register16(ANGLE_TUNE2, enable << 7);
    };
    
    void set_angle_tune(int8_t angle) // angle: -128 ~ 127
    {
        _write_register16(ANGLE_TUNE1, (uint8_t)angle);
    };

    void set_resolution(uint16_t resolution)
    {
        if (resolution < 0) resolution = 0;
        if (resolution > 520) resolution = 520;

        uint8_t resolution_l = resolution & 0xFF;
        uint8_t resolution_h = resolution >> 8;

        _write_register(RESOLUTION_X_L, resolution_l);
        _write_register(RESOLUTION_X_H, resolution_h);
        _write_register(RESOLUTION_Y_L, resolution_l);
        _write_register(RESOLUTION_Y_H, resolution_h);
        _write_register(SET_RESOLUTION, 0x01);

        //It is recommended to set bit-7 in RIPPLE_CONTROL register to enable the ripple control when select 9000 CPI and above.
        if (resolution >= 180) set_ripple_control(true);
        else set_ripple_control(false);    
    }

    void set_mode(uint8_t mode)
    {
        _write_register(0x7F, 0x05);
        _write_register(0x51, (mode == 2) ? 0x28 : 0x40);
        _write_register(0x53, (mode == 2) ? 0x30 : 0x40);
        _write_register(0x61, (mode == 0 || mode == 3) ? 0x31 : 0x3B);
        _write_register(0x6E, (mode == 0 || mode == 3) ? 0x0F : 0x1F);
        _write_register(0x7F, 0x07);
        _write_register(0x42, (mode == 3) ? 0x2F : 0x32);
        _write_register(0x43, 0x00);
        _write_register(0x7F, 0x0D);
        _write_register(0x51, (mode == 3) ? 0x12 : 0x00);
        _write_register(0x52, (mode == 3) ? 0xDB : 0x49);
        _write_register(0x53, (mode == 3) ? 0x12 : 0x00);
        _write_register(0x54, (mode == 3) ? 0xDC : 0x5B);
        _write_register(0x55, (mode == 3) ? 0x12 : 0x00);
        _write_register(0x56, (mode == 3) ? 0xEA : 0x64);
        _write_register(0x57, (mode == 3) ? 0x15 : 0x02);
        _write_register(0x58, (mode == 3) ? 0x2D : 0xA5);
        _write_register(0x7F, 0x05);
        _write_register(0x54, (mode == 2) ? 0x52 : (mode == 3) ? 0x55 : 0x54);
        if (mode != 3)
        {
            _write_register(0x78, (mode == 2) ? 0x0A : 0x01);
            _write_register(0x79, (mode == 2) ? 0x0F : 0x9C);
        }

        uint8_t tmp = _read_register(0x40);
        if (mode != 3) _write_register(0x40, tmp & mode);
        else _write_register(0x40, tmp & 0x83);     
    }

    void set(const char* name, int32_t value)
    {
        if (strcmp(name, "spi_speed") == 0) set_spi_speed((uint8_t)value);
        if (strcmp(name, "resolution") == 0) set_resolution((uint16_t)value);
        if (strcmp(name, "ripple_control") == 0) set_ripple_control((bool)value);
        if (strcmp(name, "angle_snap") == 0) set_angle_snap((bool)value);
        if (strcmp(name, "swap_xy") == 0) set_axis_control_swapxy((bool)value);
        if (strcmp(name, "invert_x") == 0) set_axis_control_invertx((bool)value);
        if (strcmp(name, "invert_y") == 0) set_axis_control_inverty((bool)value);
        if (strcmp(name, "lift_config") == 0) set_lift_config((bool)value);
        if (strcmp(name, "angle_tune_ena") == 0) set_angle_tune_enabled((bool)value);
        if (strcmp(name, "angle_tune_val") == 0) set_angle_tune((int8_t)value);
        if (strcmp(name, "read_burst_ena") == 0) set_burst_enabled((bool)value);
    }

    int32_t get(const char* name)
    {
        if (strcmp(name, "spi_speed") == 0) return (int32_t)_spi_speed / 1000000;
        if (strcmp(name, "resolution") == 0) return (int32_t)read_resolution();
        if (strcmp(name, "ripple_control") == 0) return (int32_t)read_ripple_control();
        if (strcmp(name, "angle_snap") == 0) return (int32_t)read_angle_snap();
        if (strcmp(name, "swap_xy") == 0) return (int32_t)read_axis_control_swapxy();
        if (strcmp(name, "invert_x") == 0) return (int32_t)read_axis_control_invertx();
        if (strcmp(name, "invert_y") == 0) return (int32_t)read_axis_control_inverty();
        if (strcmp(name, "lift_config") == 0) return (int32_t)read_lift_config();
        if (strcmp(name, "angle_tune_ena") == 0) return (int32_t)read_angle_tune_enabled();
        if (strcmp(name, "angle_tune_val") == 0) return (int32_t)read_angle_tune();
        if (strcmp(name, "read_burst_ena") == 0) return (int32_t)_burst_enabled;
    }

    void set_spi_speed(uint8_t speed)
    {
        _spi_speed = speed * 1000000;
    }    
    
    void set_burst_enabled(bool value)
    {
        _burst_enabled = value;
    }

    private:

    SPIClass *_SPI = NULL;
    uint32_t _spi_speed = 1000000;

    bool _burst_enabled = false;
    uint8_t _burst_data[12];

    void _reset()
    {
        digitalWrite(RESET_PIN, 0);
        delay(50);
        digitalWrite(RESET_PIN, 1);
        delay(50);
    };

    void _power_up_init()
    {
        _write_register(0x7F, 0x07);
        _write_register(0x40, 0x41);
        _write_register(0x7F, 0x00);
        _write_register(0x40, 0x80);
        _write_register(0x7F, 0x0E);
        _write_register(0x55, 0x0D);
        _write_register(0x56, 0x1B);
        _write_register(0x57, 0xE8);
        _write_register(0x58, 0xD5);
        _write_register(0x7F, 0x14);
        _write_register(0x42, 0xBC);
        _write_register(0x43, 0x74);
        _write_register(0x4B, 0x20);
        _write_register(0x4D, 0x00);
        _write_register(0x53, 0x0E);
        _write_register(0x7F, 0x05);
        _write_register(0x44, 0x04);
        _write_register(0x4D, 0x06);
        _write_register(0x51, 0x40);
        _write_register(0x53, 0x40);
        _write_register(0x55, 0xCA);
        _write_register(0x5A, 0xE8);
        _write_register(0x5B, 0xEA);
        _write_register(0x61, 0x31);
        _write_register(0x62, 0x64);
        _write_register(0x6D, 0xB8);
        _write_register(0x6E, 0x0F);
        _write_register(0x70, 0x02);
        _write_register(0x4A, 0x2A);
        _write_register(0x60, 0x26);
        _write_register(0x7F, 0x06);
        _write_register(0x6D, 0x70);
        _write_register(0x6E, 0x60);
        _write_register(0x6F, 0x04);
        _write_register(0x53, 0x02);
        _write_register(0x55, 0x11);
        _write_register(0x7A, 0x01);
        _write_register(0x7D, 0x51);
        _write_register(0x7F, 0x07);
        _write_register(0x41, 0x10);
        _write_register(0x42, 0x32);
        _write_register(0x43, 0x00);
        _write_register(0x7F, 0x08);
        _write_register(0x71, 0x4F);
        _write_register(0x7F, 0x09);
        _write_register(0x62, 0x1F);
        _write_register(0x63, 0x1F);
        _write_register(0x65, 0x03);
        _write_register(0x66, 0x03);
        _write_register(0x67, 0x1F);
        _write_register(0x68, 0x1F);
        _write_register(0x69, 0x03);
        _write_register(0x6A, 0x03);
        _write_register(0x6C, 0x1F);
        _write_register(0x6D, 0x1F);
        _write_register(0x51, 0x04);
        _write_register(0x53, 0x20);
        _write_register(0x54, 0x20);
        _write_register(0x71, 0x0C);
        _write_register(0x72, 0x07);
        _write_register(0x73, 0x07);
        _write_register(0x7F, 0x0A);
        _write_register(0x4A, 0x14);
        _write_register(0x4C, 0x14);
        _write_register(0x55, 0x19);
        _write_register(0x7F, 0x14);
        _write_register(0x4B, 0x30);
        _write_register(0x4C, 0x03);
        _write_register(0x61, 0x0B);
        _write_register(0x62, 0x0A);
        _write_register(0x63, 0x02);
        _write_register(0x7F, 0x15);
        _write_register(0x4C, 0x02);
        _write_register(0x56, 0x02);
        _write_register(0x41, 0x91);
        _write_register(0x4D, 0x0A);
        _write_register(0x7F, 0x0C);
        _write_register(0x4A, 0x10);
        _write_register(0x4B, 0x0C);
        _write_register(0x4C, 0x40);
        _write_register(0x41, 0x25);
        _write_register(0x55, 0x18);
        _write_register(0x56, 0x14);
        _write_register(0x49, 0x0A);
        _write_register(0x42, 0x00);
        _write_register(0x43, 0x2D);
        _write_register(0x44, 0x0C);
        _write_register(0x54, 0x1A);
        _write_register(0x5A, 0x0D);
        _write_register(0x5F, 0x1E);
        _write_register(0x5B, 0x05);
        _write_register(0x5E, 0x0F);
        _write_register(0x7F, 0x0D);
        _write_register(0x48, 0xDD);
        _write_register(0x4F, 0x03);
        _write_register(0x52, 0x49);
        _write_register(0x51, 0x00);
        _write_register(0x54, 0x5B);
        _write_register(0x53, 0x00);
        _write_register(0x56, 0x64);
        _write_register(0x55, 0x00);
        _write_register(0x58, 0xA5);
        _write_register(0x57, 0x02);
        _write_register(0x5A, 0x29);
        _write_register(0x5B, 0x47);
        _write_register(0x5C, 0x81);
        _write_register(0x5D, 0x40);
        _write_register(0x71, 0xDC);
        _write_register(0x70, 0x07);
        _write_register(0x73, 0x00);
        _write_register(0x72, 0x08);
        _write_register(0x75, 0xDC);
        _write_register(0x74, 0x07);
        _write_register(0x77, 0x00);
        _write_register(0x76, 0x08);
        _write_register(0x7F, 0x10);
        _write_register(0x4C, 0xD0);
        _write_register(0x7F, 0x00);
        _write_register(0x4F, 0x63);
        _write_register(0x4E, 0x00);
        _write_register(0x52, 0x63);
        _write_register(0x51, 0x00);
        _write_register(0x54, 0x54);
        _write_register(0x5A, 0x10);
        _write_register(0x77, 0x4F);
        _write_register(0x47, 0x01);
        _write_register(0x5B, 0x40);
        _write_register(0x64, 0x60);
        _write_register(0x65, 0x06);
        _write_register(0x66, 0x13);
        _write_register(0x67, 0x0F);
        _write_register(0x78, 0x01);
        _write_register(0x79, 0x9C);
        _write_register(0x40, 0x00);
        _write_register(0x55, 0x02);
        _write_register(0x23, 0x70);
        _write_register(0x22, 0x01);
        delay(1);

        bool step139 = false;
        for (uint8_t i = 0; i < 60; i++)
        {
            uint8_t val = _read_register(0x6C);
            if (val == 0x80)
            {
                step139 = true;
                break;
            }
            delay(1);
        }
        if (!step139)
        {
            _write_register(0x7F, 0x14);
            _write_register(0x6C, 0x00);
            _write_register(0x7F, 0x00);
        }

        _write_register(0x22, 0x00);
        _write_register(0x55, 0x00);
        _write_register(0x7F, 0x07);
        _write_register(0x40, 0x40);
        _write_register(0x7F, 0x00);

        _read_register(MOTION);  
        _read_register(DELTA_X_L);
        _read_register(DELTA_X_H);
        _read_register(DELTA_Y_L);
        _read_register(DELTA_Y_H);
    };

    void _write_register(uint8_t address, uint8_t data)
    {
        SPISettings _SPISettings(_spi_speed, MSBFIRST, SPI_MODE0);

        _SPI->beginTransaction(_SPISettings);
        digitalWrite(CS_PIN, LOW);
        _SPI->write16((address | 0x80) << 8 | data);
        ets_delay_us(1);
        digitalWrite(CS_PIN, HIGH);
        _SPI->endTransaction();
    };

    void _write_register16(uint16_t address, uint8_t data)
    {
        _write_register(0x7F, address >> 8);
        _write_register(address & 0xFF, data);
        _write_register(0x7F, 0x00);
    };

    uint8_t _read_register16(uint16_t address)
    {
        _write_register(0x7F, address >> 8);
        uint8_t data = _read_register(address & 0xFF);
        _write_register(0x7F, 0x00);
        return data;
    };

    uint8_t _read_register(uint8_t address)
    {
        SPISettings _SPISettings(_spi_speed, MSBFIRST, SPI_MODE0);

        _SPI->beginTransaction(_SPISettings);
        digitalWrite(CS_PIN, LOW);
        _SPI->write(address);
        ets_delay_us(2);
        uint8_t data = _SPI->transfer(0x00);
        digitalWrite(CS_PIN, HIGH);
        _SPI->endTransaction();
        ets_delay_us(2);
        return data;
    };

    void _read_register_burst()
    {
        SPISettings _SPISettings(_spi_speed, MSBFIRST, SPI_MODE0);

        _SPI->beginTransaction(_SPISettings);
        digitalWrite(CS_PIN, LOW);
        _SPI->write(MOTION_BURST);
        ets_delay_us(2);

        _SPI->transferBytes(NULL, _burst_data, 12);
        digitalWrite(CS_PIN, HIGH);
        _SPI->endTransaction();
        ets_delay_us(1);
    };
};