# EDFA Controller - Python RS232 Interface

Python class for controlling FiberLabs Desktop Optical Fibre Amplifier via RS232 protocol.

Based on **FiberLabs Programming Manual v4.0** and **Operations Guide v5.0**.

## Features

- ✅ Complete RS232 command implementation
- ✅ Support for both ACC and ALC driving modes
- ✅ Real-time monitoring of optical levels, currents, and temperatures
- ✅ Comprehensive alarm configuration
- ✅ Context manager support for clean connection handling
- ✅ Type hints for better IDE support
- ✅ Detailed error handling and validation

## Requirements

- Python 3.7+
- pyserial library

## Installation

### 1. Install Python dependencies

```bash
pip install pyserial
```

### 2. Download the controller files

- `edfa_controller.py` - Main controller class
- `edfa_examples.py` - Example usage scripts

### 3. Configure your serial port

**Linux/macOS:**
```python
PORT = "/dev/ttyUSB0"  # or /dev/ttyS0, /dev/cu.usbserial-XXX, etc.
```

**Windows:**
```python
PORT = "COM3"  # or COM4, COM5, etc.
```

Check device manager or use:
```bash
# Linux
ls /dev/tty*

# macOS
ls /dev/cu.*

# Windows
mode
```

## Quick Start

### Basic Usage

```python
from edfa_controller import EDFAController, DrivingMode

# Connect to EDFA
with EDFAController("/dev/ttyUSB0", baudrate=9600) as edfa:
    # Get device info
    print(edfa.get_identification())
    
    # Monitor parameters
    output_levels = edfa.get_output_level()
    print(f"Output levels: {output_levels} dBm")
    
    # Set to ALC mode
    edfa.set_driving_mode(1, DrivingMode.ALC)
    edfa.set_alc_output_level(1, 13.0)  # 13 dBm
    
    # Activate output
    edfa.set_output_active(True)
```

### Manual Connection Management

```python
from edfa_controller import EDFAController

edfa = EDFAController("/dev/ttyUSB0", baudrate=9600)

# Connect
if edfa.connect():
    print("Connected!")
    
    # Your operations here
    case_temp = edfa.get_case_temperature()
    print(f"Case temperature: {case_temp}°C")
    
    # Disconnect when done
    edfa.disconnect()
```

## Command Reference

### Monitoring Commands

| Method | Description | Returns |
|--------|-------------|---------|
| `get_output_level()` | Get optical output levels | List[Optional[float]] in dBm |
| `get_input_level()` | Get optical input levels | List[Optional[float]] in dBm |
| `get_return_level()` | Get optical return levels | List[Optional[float]] in dBm |
| `get_case_temperature()` | Get case temperature | float in °C |
| `get_ld_current(channel)` | Get pump-LD current | float in mA |
| `get_ld_temperature(channel)` | Get pump-LD temperature | float in °C |
| `get_tec_current(channel)` | Get TEC current | float in mA |

### Setting Commands

| Method | Description | Arguments |
|--------|-------------|-----------|
| `set_output_active(active)` | Turn output ON/OFF | bool |
| `set_driving_mode(ch, mode)` | Set driving mode | channel, DrivingMode |
| `set_acc_current(ch, current)` | Set ACC current | channel, float (mA) |
| `set_alc_output_level(ch, level)` | Set ALC output level | channel, float (dBm) |
| `save_settings()` | Save settings to EEPROM | None |

### Alarm Commands

| Method | Description |
|--------|-------------|
| `get_alarm_status()` | Get current alarm states |
| `set_output_alarm(...)` | Configure output level alarm |
| `set_input_alarm(...)` | Configure input level alarm |
| `set_return_loss_alarm(...)` | Configure return loss alarm |
| `set_case_temp_alarm(...)` | Configure case temperature alarm |
| `set_ld_current_alarm(...)` | Configure LD current alarm |
| `set_ld_temp_alarm(...)` | Configure LD temperature alarm |
| `set_input_interlock(enabled)` | Enable/disable input interlock |

## Driving Modes

### ACC Mode (Auto Current Control)

Maintains constant pump-LD current.

**Use case:** Fine adjustment of output levels, optimisation

```python
edfa.set_driving_mode(1, DrivingMode.ACC)
edfa.set_acc_current(1, 400.0)  # 400 mA
edfa.set_output_active(True)
```

**Characteristics:**
- ✅ Easy adjustment
- ⚠️ Output may vary with input signal, temperature
- Best for: Lab testing, optimisation

### ALC Mode (Auto Light Control)

Maintains constant optical output level.

**Use case:** System operation, stable output required

```python
edfa.set_driving_mode(1, DrivingMode.ALC)
edfa.set_alc_output_level(1, 13.0)  # 13 dBm
edfa.set_output_active(True)
```

**Characteristics:**
- ✅ Constant output despite input variations
- ✅ Compensates for temperature drift
- Best for: Production systems, long-term operation

## Multi-Channel Operation

For amplifiers with multiple pump-LDs:

### Type A: All LDs work in conjunction on ALC

```python
# Set any LD to ALC - all others automatically follow
edfa.set_driving_mode(1, DrivingMode.ALC)
edfa.set_alc_output_level(1, 13.0)
edfa.set_output_active(True)
# All LDs work together to maintain output level
```

### Type B: Main LD + Sub-LDs (ACC only)

```python
# 1. Set main LD to ALC
edfa.set_driving_mode(1, DrivingMode.ALC)
edfa.set_alc_output_level(1, 13.0)

# 2. Activate output
edfa.set_output_active(True)

# 3. Adjust sub-LD currents to support main LD
# Keep main LD current at ~90% of limit
edfa.set_acc_current(2, 300.0)  # Sub-LD 2
edfa.set_acc_current(3, 300.0)  # Sub-LD 3
edfa.set_acc_current(4, 300.0)  # Sub-LD 4

# 4. Check main LD current has margin
main_ld_current = edfa.get_ld_current(1)
print(f"Main LD current: {main_ld_current} mA")
```

## Alarm Configuration

### Output Level Alarm

Triggers when output drops below threshold:

```python
edfa.set_output_alarm(
    path=1,
    threshold_dbm=10.0,      # Alert if < 10 dBm
    enabled=True,
    hysteresis_db=0.5        # Recover at 10.5 dBm
)
```

### LD Current Alarm

Triggers when LD current exceeds threshold:

```python
edfa.set_ld_current_alarm(
    channel=1,
    threshold_ma=500.0,      # Alert if > 500 mA
    enabled=True,
    hysteresis_ma=5.0        # Recover at 495 mA
)
```

### Input Interlock

Automatically shuts down output if input signal lost:

```python
# Configure input alarm
edfa.set_input_alarm(
    path=1,
    threshold_dbm=-10.0,
    enabled=True,
    hysteresis_db=0.5
)

# Enable interlock
edfa.set_input_interlock(enabled=True)
```

⚠️ **Safety Warning:** Output turns OFF automatically when input alarm triggers.

## Error Handling

The controller raises exceptions for communication errors:

```python
try:
    edfa.set_driving_mode(1, DrivingMode.ALC)
except ConnectionError:
    print("Serial connection lost")
except ValueError as e:
    # Device returned error (e.g., !!BUFOVFL, ??CMD)
    print(f"Device error: {e}")
```

### Common Error Messages

| Error | Description | Solution |
|-------|-------------|----------|
| `!!BUFOVFL` | Receiving buffer overflow | Check delimiter setting |
| `!!NORPLY` | Internal device reply error | Check device status |
| `??CMD` | Command error | Check command syntax |
| `??ARG` | Argument error | Check parameter values |
| `??NODTCT` | Invalid channel | Channel not installed |
| `??MODE_LOCKED` | Mode switch error | ACC ONLY channel |

## Examples

See `edfa_examples.py` for comprehensive examples:

```bash
python edfa_examples.py
```

### Example 1: Basic Monitoring

```python
with EDFAController(PORT) as edfa:
    print(f"Device: {edfa.get_identification()}")
    print(f"Output: {edfa.get_output_level()[0]:.2f} dBm")
    print(f"Case temp: {edfa.get_case_temperature():.1f} °C")
    print(f"Alarms: {edfa.get_alarm_status()}")
```

### Example 2: Continuous Monitoring

```python
with EDFAController(PORT) as edfa:
    edfa.set_output_active(True)
    
    for i in range(10):
        output = edfa.get_output_level()[0]
        current = edfa.get_ld_current(1)
        print(f"{i}s: {output:.2f} dBm, {current:.1f} mA")
        time.sleep(1)
    
    edfa.set_output_active(False)
```

### Example 3: Alarm Monitoring

```python
with EDFAController(PORT) as edfa:
    while True:
        alarms = edfa.get_alarm_status()
        
        if any(alarms.values()):
            print("⚠️ ALARM DETECTED:")
            for name, active in alarms.items():
                if active:
                    print(f"  - {name}")
        
        time.sleep(1)
```

## Important Notes

### EEPROM Write Cycles

⚠️ The EEPROM has limited write cycles (~10⁷). Avoid frequent use of:
```python
edfa.save_settings()  # Use sparingly!
```

Temporary settings are fine:
```python
edfa.set_alc_output_level(1, 13.0)  # OK to call frequently
```

### Auto Recovery (Not Recommended)

⚠️ **Warning:** Auto recovery from interlock may be dangerous depending on laser class.

```python
# Use with caution and proper safety procedures
edfa.set_auto_recovery(start=True, manual=False)
```

### Thread Safety

The controller is **not** thread-safe. Use separate instances or add locking:

```python
import threading

lock = threading.Lock()

def safe_operation():
    with lock:
        output = edfa.get_output_level()
```

## Communication Settings

Configure in device menu or via front panel:

| Parameter | Options | Default |
|-----------|---------|---------|
| Baud rate | 9600, 19200, 38400, 57600 | 9600 |
| Delimiter | CR, LF | CR |
| Data bits | 8 | 8 |
| Parity | None | None |
| Stop bits | 1 | 1 |

Change in Python:
```python
edfa = EDFAController(
    port="/dev/ttyUSB0",
    baudrate=19200,
    delimiter='LF'  # or 'CR'
)
```

## Troubleshooting

### Cannot Connect

```python
# Linux: Check permissions
sudo chmod 666 /dev/ttyUSB0

# Or add user to dialout group
sudo usermod -a -G dialout $USER
```

### No Response

- Check baud rate matches device setting
- Check delimiter (CR vs LF)
- Verify device is powered on
- Try different USB cable/port

### Unexpected Values

- Check connector cleaning
- Verify input signal present
- Check alarm status
- Review driving mode settings

## Safety Considerations

⚠️ **Laser Safety:**
- Class 3B/4 laser radiation
- Never look into fibre end
- Always cap unused outputs
- Follow local laser safety regulations

⚠️ **Interlock:**
- Emergency remote interlock available on rear panel
- Short circuit = normal operation
- Open circuit = output OFF

⚠️ **Temperature:**
- Do not block ventilation
- Monitor case temperature
- Check alarm thresholds

## References

- FiberLabs Programming Manual v4.0
- FiberLabs Operations Guide v5.0
- IEC 60825-1:2001 (Laser Safety)

## Licence

This code is provided as-is for controlling FiberLabs EDFA equipment.

## Support

For device support, contact FiberLabs:
- Email: info@fiberlabs.co.jp
- Web: http://www.fiberlabs.co.jp
- Tel: +81-49-278-7829

For code issues, refer to the documentation or examples.
