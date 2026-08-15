#include <Preferences.h>

struct setting_line {
    const char* name;       // текстовый ключ
    int32_t     value;      // текущее значение
    int32_t     min;        // минимально допустимое
    int32_t     max;        // максимально допустимое
};

class Settings
{
    public:

        static const uint8_t data_count = 11;
        const setting_line data[data_count] = 
        {
        //  {name               value   min     max}
            {"ripple_control",  0,      0,      1},
            {"angle_snap",      0,      0,      1},
            {"swap_xy",         0,      0,      1},
            {"invert_x",        0,      0,      1},
            {"invert_y",        0,      0,      1},
            {"lift_config",     0,      0,      1},
            {"angle_tune_ena",  0,      0,      1},
            {"angle_tune_val",  0,      -128,   127},
            {"read_burst_ena",  0,      0,      1},
            {"resolution",      0,      0,      300},
            {"spi_speed",       1,      1,      39},
        };
        

        Settings()
        {
            _pref.begin("config", false);
            for (uint8_t i = 0; i < data_count; i++) 
            {
                if (!_pref.isKey(data[i].name)) // exist
                {
                    _pref.putInt(data[i].name, data[i].value);
                }
            }
            _pref.end();
        };

        int32_t get(const char* name)
        {
            int32_t value = 0;

            _pref.begin("config", true);
            for (uint8_t i = 0; i < data_count; i++)
            {
                if (strcmp(data[i].name, name) == 0)
                {
                    value = _pref.getInt(name, data[i].value);
                }
            }
            _pref.end();

            return value;
        }

        void set(const char* name, int32_t value)
        {
            for (uint8_t i = 0; i < data_count; i++)
            {
                if (strcmp(data[i].name, name) == 0)
                {
                    if (value >= data[i].min && value <= data[i].max) 
                    {
                        _pref.begin("config", false);
                        _pref.putInt(name, value);
                        _pref.end();
                    }
                }
            }
        }

    private:
        Preferences _pref;
};

