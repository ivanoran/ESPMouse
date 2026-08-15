# ESPMouse

BLE HID Mouse for ESP32-C3 with PAW3395 sensor.

## Features

- BLE HID (NimBLE) connection
- PAW3395 sensor motion tracking (16-bit delta_x/delta_y)
- 5 buttons support (left, right, middle, wheel up, wheel down)
- Configurable settings via PC application
- Settings persistence in NVS

## Settings

The following settings are available:
- ripple_control (0-1)
- angle_snap (0-1)
- swap_xy (0-1)
- invert_x (0-1)
- invert_y (0-1)
- lift_config (0-1)
- angle_tune_ena (0-1)
- angle_tune_val (-128 to 127)
- read_burst_ena (0-1)
- resolution (0-300)
- spi_speed (1-39)

## Building

```bash
cd main
idf.py build
```

## PC Application

See `pc_app/mouse_config.py` for the Python configuration application.