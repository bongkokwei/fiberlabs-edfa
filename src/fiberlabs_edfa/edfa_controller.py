#!/usr/bin/env python3
"""
EDFA Controller Class
Control Desktop Optical Fibre Amplifier via RS232 interface
Based on FiberLabs Programming Manual v4.0
"""

import serial
import time
from typing import Optional, Dict, List, Tuple, Union
from enum import Enum


class DrivingMode(Enum):
    """Pump-LD driving modes"""

    ALC = 0  # Auto Light Control (constant output level)
    ACC = 1  # Auto Current Control (constant pump-LD current)


class EDFAController:
    """
    Controller class for FiberLabs Desktop Optical Fibre Amplifier

    Supports RS232 communication with commands for:
    - Monitoring optical levels, temperatures, and currents
    - Setting driving modes and parameters
    - Configuring alarms and interlocks
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        delimiter: str = "\r",
        timeout: float = 1.0,
    ):
        """
        Initialise EDFA controller

        Args:
            port: Serial port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
            baudrate: Communication speed (9600, 19200, 38400, 57600)
            delimiter: Message terminator ('CR' or 'LF')
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.delimiter = "\r" if delimiter.upper() == "CR" else "\n"
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None

    def connect(self) -> bool:
        """
        Open serial connection

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            time.sleep(0.1)  # Allow connection to stabilise
            print("Connected to EDFA")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

    def _send_command(self, command: str) -> str:
        """
        Send command and receive response

        Args:
            command: Command string without delimiter

        Returns:
            Response string from device
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            raise ConnectionError("Serial connection not open")

        # Clear input buffer
        self.serial_conn.reset_input_buffer()

        # Send command with delimiter
        full_command = command + self.delimiter
        self.serial_conn.write(full_command.encode("ascii"))

        # Read response until delimiter
        response = self.serial_conn.read_until(self.delimiter.encode("ascii"))
        response = response.decode("ascii").strip()

        # Check for error messages
        if response.startswith("!!") or response.startswith("??"):
            raise ValueError(f"Device error: {response}")

        return response

    # ========== MONITOR COMMANDS ==========

    def get_output_level(self) -> List[Optional[float]]:
        """
        Get optical output level for all channels

        Returns:
            List of output levels in dBm (None for unavailable channels)
        """
        response = self._send_command("MONOUT")
        levels = []
        for val in response.split(","):
            val = val.strip()
            if val == "N/A":
                levels.append(None)
            else:
                levels.append(float(val))
        return levels

    def get_input_level(self) -> List[Optional[float]]:
        """
        Get optical input level for all channels

        Returns:
            List of input levels in dBm (None for unavailable channels)
        """
        response = self._send_command("MONIN")
        levels = []
        for val in response.split(","):
            val = val.strip()
            if val == "N/A":
                levels.append(None)
            else:
                levels.append(float(val))
        return levels

    def get_return_level(self) -> List[Optional[float]]:
        """
        Get optical return level for all channels

        Returns:
            List of return levels in dBm (None for unavailable channels)
        """
        response = self._send_command("MONRET")
        levels = []
        for val in response.split(","):
            val = val.strip()
            if val == "N/A":
                levels.append(None)
            else:
                levels.append(float(val))
        return levels

    def get_case_temperature(self) -> float:
        """
        Get case temperature

        Returns:
            Case temperature in degrees Celsius
        """
        response = self._send_command("MONCTMP")
        return float(response)

    def get_ld_current(
        self, channel: Optional[int] = None
    ) -> Union[float, List[float]]:
        """
        Get pump-LD forward current

        Args:
            channel: LD channel (1-4), or None for all channels

        Returns:
            Current in mA (single value if channel specified, list otherwise)
        """
        if channel is not None:
            response = self._send_command(f"MONLDC,{channel}")
            return float(response)
        else:
            response = self._send_command("MONLDC")
            return [float(val.strip()) for val in response.split(",")]

    def get_ld_temperature(
        self, channel: Optional[int] = None
    ) -> Union[float, List[float]]:
        """
        Get pump-LD temperature

        Args:
            channel: LD channel (1-4), or None for all channels

        Returns:
            Temperature in degrees Celsius
        """
        if channel is not None:
            response = self._send_command(f"MONLDT,{channel}")
            return float(response)
        else:
            response = self._send_command("MONLDT")
            return [float(val.strip()) for val in response.split(",")]

    def get_tec_current(
        self, channel: Optional[int] = None
    ) -> Union[float, List[float]]:
        """
        Get pump-LD TEC (Thermo Electric Cooler) current

        Args:
            channel: LD channel (1-4), or None for all channels

        Returns:
            TEC current in mA
        """
        if channel is not None:
            response = self._send_command(f"MONTEC,{channel}")
            return float(response)
        else:
            response = self._send_command("MONTEC")
            return [float(val.strip()) for val in response.split(",")]

    # ========== SETTING COMMANDS ==========

    def set_output_active(self, active: bool):
        """
        Switch pump-LD output ON/OFF

        Args:
            active: True to turn ON, False to turn OFF
        """
        state = 1 if active else 0
        response = self._send_command(f"ACTIVE,{state}")
        # Response echoes the command
        return response

    def set_auto_recovery(self, start: bool, manual: bool = False):
        """
        Configure auto recovery operation (NOT RECOMMENDED - see manual)

        Args:
            start: True to start operation, False to stop
            manual: True for manual recovery, False for auto recovery

        Note:
            Auto recovery from interlock may be dangerous depending on laser class
        """
        start_val = 1 if start else 0
        manual_val = 1 if manual else 0
        response = self._send_command(f"PRMACTV,{start_val},{manual_val}")
        return response

    def get_auto_recovery_status(self) -> Tuple[bool, bool]:
        """
        Get current auto recovery status

        Returns:
            Tuple of (operating, manual_mode)
        """
        response = self._send_command("PRMACTV")
        # Response format: "PRMACTV, start, manual"
        parts = response.split(",")
        start = int(parts[1].strip()) == 1
        manual = int(parts[2].strip()) == 1
        return start, manual

    def set_driving_mode(self, channel: int, mode: DrivingMode):
        """
        Set pump-LD driving mode

        Args:
            channel: LD channel (1-4)
            mode: DrivingMode.ALC or DrivingMode.ACC

        Note:
            Switching mode will turn OFF optical output
        """
        response = self._send_command(f"SETMOD,{channel},{mode.value}")
        return response

    def get_driving_mode(self, channel: int) -> DrivingMode:
        """
        Get current driving mode

        Args:
            channel: LD channel (1-4)

        Returns:
            Current driving mode
        """
        response = self._send_command(f"SETMOD,{channel}")
        # Response format: "SETMOD, channel, mode"
        parts = response.split(",")
        mode_val = int(parts[2].strip())
        return DrivingMode(mode_val)

    def set_acc_current(self, channel: int, current_ma: float):
        """
        Set pump-LD current for ACC mode (temporary setting)

        Args:
            channel: LD channel (1-4)
            current_ma: Target current in mA

        Note:
            Use save_settings() to make permanent
        """
        response = self._send_command(f"SETACC,{channel},{int(current_ma)}")
        return response

    def get_acc_current(self, channel: int) -> float:
        """
        Get current ACC mode setting

        Args:
            channel: LD channel (1-4)

        Returns:
            Current setting in mA
        """
        response = self._send_command(f"SETACC,{channel}")
        # Response format: "SETACC, channel, current"
        parts = response.split(",")
        return float(parts[2].strip())

    def set_alc_output_level(self, channel: int, level_dbm: float):
        """
        Set optical output level for ALC mode (temporary setting)

        Args:
            channel: LD channel (1-4)
            level_dbm: Target output level in dBm

        Note:
            Use save_settings() to make permanent
        """
        response = self._send_command(f"SETALC,{channel},{level_dbm}")
        return response

    def get_alc_output_level(self, channel: int) -> float:
        """
        Get current ALC mode setting

        Args:
            channel: LD channel (1-4)

        Returns:
            Current setting in dBm
        """
        response = self._send_command(f"SETALC,{channel}")
        # Response format: "SETALC, channel, level"
        parts = response.split(",")
        return float(parts[2].strip())

    def save_settings(self):
        """
        Save temporary settings to EEPROM

        Warning:
            EEPROM has limited rewrite cycles (~10^7)
            Avoid extremely frequent use
        """
        response = self._send_command("SAVEREF")
        return response

    # ========== ALARM COMMANDS ==========

    def get_alarm_status(self) -> Dict[str, bool]:
        """
        Get current alarm status

        Returns:
            Dictionary with alarm states: 'OUT', 'LDC', 'IN', 'TEMP'
        """
        response = self._send_command("ALMSTAT")
        # Response format: "ALMSTAT,hex_value"
        hex_val = int(response.split(",")[1].strip(), 16)

        return {
            "OUT": bool(hex_val & 0x01),  # bit 0
            "LDC": bool(hex_val & 0x02),  # bit 1
            "IN": bool(hex_val & 0x04),  # bit 2
            "TEMP": bool(hex_val & 0x08),  # bit 3
        }

    def set_output_alarm(
        self, path: int, threshold_dbm: float, enabled: bool, hysteresis_db: float = 0.5
    ):
        """
        Configure optical output level alarm

        Args:
            path: Optical path (1-4)
            threshold_dbm: Alarm threshold in dBm
            enabled: Enable/disable alarm detection
            hysteresis_db: Hysteresis in dB for recovery
        """
        enable_val = 1 if enabled else 0
        response = self._send_command(
            f"ALMOUT,{path},{threshold_dbm},{enable_val},{hysteresis_db}"
        )
        return response

    def get_output_alarm(self, path: int) -> Dict[str, Union[float, bool]]:
        """
        Get output alarm configuration

        Args:
            path: Optical path (1-4)

        Returns:
            Dictionary with threshold, enabled, and hysteresis
        """
        response = self._send_command(f"ALMOUT,{path}")
        parts = response.split(",")
        return {
            "threshold_dbm": float(parts[2].strip()),
            "enabled": int(parts[3].strip()) == 1,
            "hysteresis_db": float(parts[4].strip()),
        }

    def set_input_alarm(
        self, path: int, threshold_dbm: float, enabled: bool, hysteresis_db: float = 0.5
    ):
        """
        Configure optical input level alarm

        Args:
            path: Optical path (1-4)
            threshold_dbm: Alarm threshold in dBm
            enabled: Enable/disable alarm detection
            hysteresis_db: Hysteresis in dB for recovery
        """
        enable_val = 1 if enabled else 0
        response = self._send_command(
            f"ALMIN,{path},{threshold_dbm},{enable_val},{hysteresis_db}"
        )
        return response

    def get_input_alarm(self, path: int) -> Dict[str, Union[float, bool]]:
        """Get input alarm configuration"""
        response = self._send_command(f"ALMIN,{path}")
        parts = response.split(",")
        return {
            "threshold_dbm": float(parts[2].strip()),
            "enabled": int(parts[3].strip()) == 1,
            "hysteresis_db": float(parts[4].strip()),
        }

    def set_return_loss_alarm(
        self, path: int, threshold_db: float, enabled: bool, hysteresis_db: float = 0.5
    ):
        """
        Configure return loss alarm

        Args:
            path: Optical path (1-4)
            threshold_db: Alarm threshold in dB (return loss)
            enabled: Enable/disable alarm detection
            hysteresis_db: Hysteresis in dB for recovery
        """
        enable_val = 1 if enabled else 0
        response = self._send_command(
            f"ALMRET,{path},{threshold_db},{enable_val},{hysteresis_db}"
        )
        return response

    def get_return_loss_alarm(self, path: int) -> Dict[str, Union[float, bool]]:
        """Get return loss alarm configuration"""
        response = self._send_command(f"ALMRET,{path}")
        parts = response.split(",")
        return {
            "threshold_db": float(parts[2].strip()),
            "enabled": int(parts[3].strip()) == 1,
            "hysteresis_db": float(parts[4].strip()),
        }

    def set_input_interlock(self, enabled: bool):
        """
        Enable/disable interlock on input level / return loss alarm

        Args:
            enabled: True to enable interlock, False to disable
        """
        enable_val = 1 if enabled else 0
        response = self._send_command(f"SETIL,{enable_val}")
        return response

    def get_input_interlock(self) -> bool:
        """Get input interlock status"""
        response = self._send_command("SETIL")
        parts = response.split(",")
        return int(parts[1].strip()) == 1

    def set_case_temp_alarm(
        self, threshold_degc: float, enabled: bool, hysteresis_degc: float = 0.5
    ):
        """
        Configure case temperature alarm

        Args:
            threshold_degc: Alarm threshold in degrees Celsius
            enabled: Enable/disable alarm detection
            hysteresis_degc: Hysteresis in degrees Celsius
        """
        enable_val = 1 if enabled else 0
        response = self._send_command(
            f"ALMCTMP,{threshold_degc},{enable_val},{hysteresis_degc}"
        )
        return response

    def get_case_temp_alarm(self) -> Dict[str, Union[float, bool]]:
        """Get case temperature alarm configuration"""
        response = self._send_command("ALMCTMP")
        parts = response.split(",")
        return {
            "threshold_degc": float(parts[1].strip()),
            "enabled": int(parts[2].strip()) == 1,
            "hysteresis_degc": float(parts[3].strip()),
        }

    def set_ld_current_alarm(
        self,
        channel: int,
        threshold_ma: float,
        enabled: bool,
        hysteresis_ma: float = 5.0,
    ):
        """
        Configure pump-LD current alarm

        Args:
            channel: LD channel (1-4)
            threshold_ma: Alarm threshold in mA
            enabled: Enable/disable alarm detection
            hysteresis_ma: Hysteresis in mA
        """
        enable_val = 1 if enabled else 0
        response = self._send_command(
            f"ALMLDC,{channel},{threshold_ma},{enable_val},{hysteresis_ma}"
        )
        return response

    def get_ld_current_alarm(self, channel: int) -> Dict[str, Union[float, bool]]:
        """Get LD current alarm configuration"""
        response = self._send_command(f"ALMLDC,{channel}")
        parts = response.split(",")
        return {
            "threshold_ma": float(parts[2].strip()),
            "enabled": int(parts[3].strip()) == 1,
            "hysteresis_ma": float(parts[4].strip()),
        }

    def set_ld_temp_alarm(
        self,
        channel: int,
        threshold_degc: float,
        enabled: bool,
        hysteresis_degc: float = 0.5,
    ):
        """
        Configure pump-LD temperature alarm

        Args:
            channel: LD channel (1-4)
            threshold_degc: Alarm threshold in degrees Celsius
            enabled: Enable/disable alarm detection
            hysteresis_degc: Hysteresis in degrees Celsius
        """
        enable_val = 1 if enabled else 0
        response = self._send_command(
            f"ALMLDT,{channel},{threshold_degc},{enable_val},{hysteresis_degc}"
        )
        return response

    def get_ld_temp_alarm(self, channel: int) -> Dict[str, Union[float, bool]]:
        """Get LD temperature alarm configuration"""
        response = self._send_command(f"ALMLDT,{channel}")
        parts = response.split(",")
        return {
            "threshold_degc": float(parts[2].strip()),
            "enabled": int(parts[3].strip()) == 1,
            "hysteresis_degc": float(parts[4].strip()),
        }

    # ========== IDENTIFICATION ==========

    def get_identification(self) -> str:
        """
        Get instrument identification information

        Returns:
            Identification string
        """
        return self._send_command("*IDN?")

    # ========== CONTEXT MANAGER SUPPORT ==========

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


# ========== EXAMPLE USAGE ==========

if __name__ == "__main__":
    # Example usage
    PORT = "COM6"  # Change to your port (e.g., "COM3" on Windows)

    # Using context manager for automatic connection handling
    with EDFAController(PORT, baudrate=57600, delimiter="CR") as edfa:
        print("Connected to EDFA")

        # Get device information
        print(f"Device: {edfa.get_identification()}")

        # Monitor parameters
        print(f"\nOutput levels: {edfa.get_output_level()} dBm")
        print(f"Input levels: {edfa.get_input_level()} dBm")
        print(f"Case temperature: {edfa.get_case_temperature():.1f} °C")
        print(f"LD1 current: {edfa.get_ld_current(1):.1f} mA")

        # Check alarm status
        alarms = edfa.get_alarm_status()
        print(f"\nAlarm status: {alarms}")

        # Set driving mode to ALC for channel 1
        edfa.set_driving_mode(1, DrivingMode.ALC)
        print(f"Channel 1 mode: {edfa.get_driving_mode(1)}")

        # Set output level (temporary)
        edfa.set_alc_output_level(1, 10.0)
        print(f"ALC setting: {edfa.get_alc_output_level(1):.1f} dBm")

        # Turn on output
        edfa.set_output_active(True)
        print("Output activated")

        # Wait and monitor
        time.sleep(2)
        print(f"Output level after activation: {edfa.get_output_level()[0]:.2f} dBm")

        # Save settings if needed
        # edfa.save_settings()  # Be cautious with this

        # Turn off output
        edfa.set_output_active(False)
        print("Output deactivated")
