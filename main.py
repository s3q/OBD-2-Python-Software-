"""
ELM327 WiFi OBD-II Professional Scanner Tool
A comprehensive command-line interface for ELM327 OBD-II scanners
Author: Advanced OBD Tools
Version: 1.0.0

https://www.dashlogic.com/docs/technical/obdii_pids
"""

import socket
import time
import threading
import json
import csv
import logging
import re
import sys
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import time
from collections import deque
import pids

# Optional matplotlib import for graphing
try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available. Graphing features disabled.")


with open("dtc.json", "r") as f:
    dtc_dict = json.load(f)


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('elm327_scanner.log'),
        logging.StreamHandler()
    ]
)

class OBDProtocol(Enum):
    """OBD Protocol definitions"""
    AUTO = "0"
    SAE_J1850_PWM = "1"
    SAE_J1850_VPW = "2"
    ISO_9141_2 = "3"
    ISO_14230_4_KWP = "4"
    ISO_14230_4_KWP_FAST = "5"
    ISO_15765_4_CAN_11_500 = "6"
    ISO_15765_4_CAN_29_500 = "7"
    ISO_15765_4_CAN_11_250 = "8"
    ISO_15765_4_CAN_29_250 = "9"
    SAE_J1939_CAN = "A"

@dataclass
class SensorReading:
    """Data structure for sensor readings"""
    pid: str
    name: str
    signals: [{
       'value': str,
       'path': str,
       'name': str,
       'unit': str,
    }] # type: ignore

    timestamp: datetime
    raw_data: str

@dataclass
class DTCInfo:
    """Data structure for Diagnostic Trouble Codes"""
    code: str
    description: str
    status: str

class PIDDatabase:
    """Complete PID database with formulas and descriptions"""
    # (PIDs 0114-011B)
    PIDS = pids.PID_DICT
    # {
    #     # Mode 01 PIDs
    #     "0100": {"name": "PIDs supported [01-20]", "unit": "bit", "formula": lambda x: x},
    #     "0101": {"name": "Monitor status since DTCs cleared", "unit": "bit", "formula": lambda x: x},
    #     "0102": {"name": "Freeze DTC", "unit": "code", "formula": lambda x: x},
    #     "0103": {"name": "Fuel system status", "unit": "status", "formula": lambda x: x},
    #     "0104": {"name": "Calculated engine load", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "0105": {"name": "Engine coolant temperature", "unit": "°C", "formula": lambda x: x[0] - 40},
    #     "0106": {"name": "Short term fuel trim—Bank 1", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "0107": {"name": "Long term fuel trim—Bank 1", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "0108": {"name": "Short term fuel trim—Bank 2", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "0109": {"name": "Long term fuel trim—Bank 2", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "010A": {"name": "Fuel rail pressure", "unit": "kPa", "formula": lambda x: x[0] * 3},
    #     "010B": {"name": "Intake manifold absolute pressure", "unit": "kPa", "formula": lambda x: x[0]},
    #     "010C": {"name": "Engine RPM", "unit": "rpm", "formula": lambda x: (x[0] * 256 + x[1]) / 4},
    #     "010D": {"name": "Vehicle speed", "unit": "km/h", "formula": lambda x: x[0]},
    #     "010E": {"name": "Timing advance", "unit": "°", "formula": lambda x: (x[0] - 128) / 2},
    #     "010F": {"name": "Intake air temperature", "unit": "°C", "formula": lambda x: x[0] - 40},
    #     "0110": {"name": "MAF air flow rate", "unit": "g/s", "formula": lambda x: (x[0] * 256 + x[1]) / 100},
    #     "0111": {"name": "Throttle position", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "0112": {"name": "Commanded secondary air status", "unit": "status", "formula": lambda x: x},
    #     "0113": {"name": "Oxygen sensors present", "unit": "bit", "formula": lambda x: x},
    #     "0114": {"name": "Oxygen sensor (narrowband) bank 1 sensor 1", "unit": "V", "formula": lambda x: x[0] / 200},
    #     "0115": {"name": "Oxygen sensor (narrowband) bank 1 sensor 2", "unit": "V", "formula": lambda x: x[0] / 200},
    #     "0116": {"name": "Oxygen sensor (narrowband) bank 1 sensor 3", "unit": "V", "formula": lambda x: x[0] / 200},
    #     "0117": {"name": "Oxygen sensor (narrowband) bank 1 sensor 4", "unit": "V", "formula": lambda x: x[0] / 200},
    #     "0118": {"name": "Oxygen sensor (narrowband) bank 2 sensor 1", "unit": "V", "formula": lambda x: x[0] / 200},
    #     "0119": {"name": "Oxygen sensor (narrowband) bank 2 sensor 2", "unit": "V", "formula": lambda x: x[0] / 200},
    #     "011A": {"name": "Oxygen sensor (narrowband) bank 2 sensor 3", "unit": "V", "formula": lambda x: x[0] / 200},
    #     "011B": {"name": "Oxygen sensor (narrowband) bank 2 sensor 4", "unit": "V", "formula": lambda x: x[0] / 200},
    #     "011C": {"name": "OBD standards", "unit": "code", "formula": lambda x: x},
    #     "011D": {"name": "O2 sensors present (4 banks)", "unit": "bit", "formula": lambda x: x},
    #     "011E": {"name": "Auxiliary input status", "unit": "bit", "formula": lambda x: x},
    #     "011F": {"name": "Run time since engine start", "unit": "s", "formula": lambda x: x[0] * 256 + x[1]},
    #     "0120": {"name": "PIDs supported [21-40]", "unit": "bit", "formula": lambda x: x},
    #     "0121": {"name": "Distance traveled with malfunction indicator lamp (MIL) on", "unit": "km", "formula": lambda x: x[0] * 256 + x[1]},
    #     "0122": {"name": "Fuel Rail Pressure", "unit": "kPa", "formula": lambda x: ((x[0] * 256 + x[1]) * 0.079)},
    #     "0123": {"name": "Fuel Rail Gauge Pressure", "unit": "kPa", "formula": lambda x: ((x[0] * 256 + x[1]) * 10)},
    #     "0124": {"name": "Oxygen sensor (wideband) bank 1 sensor 1", "unit": "ratio", "formula": lambda x: ((x[0] * 256 + x[1]) / 32768)},
    #     "0125": {"name": "Oxygen sensor (wideband) bank 1 sensor 2", "unit": "ratio", "formula": lambda x: ((x[0] * 256 + x[1]) / 32768)},
    #     "0126": {"name": "Oxygen sensor (wideband) bank 1 sensor 3", "unit": "ratio", "formula": lambda x: ((x[0] * 256 + x[1]) / 32768)},
    #     "0127": {"name": "Oxygen sensor (wideband) bank 1 sensor 4", "unit": "ratio", "formula": lambda x: ((x[0] * 256 + x[1]) / 32768)},
    #     "0128": {"name": "Oxygen sensor (wideband) bank 2 sensor 1", "unit": "ratio", "formula": lambda x: ((x[0] * 256 + x[1]) / 32768)},
    #     "0129": {"name": "Oxygen sensor (wideband) bank 2 sensor 2", "unit": "ratio", "formula": lambda x: ((x[0] * 256 + x[1]) / 32768)},
    #     "012A": {"name": "Oxygen sensor (wideband) bank 2 sensor 3", "unit": "ratio", "formula": lambda x: ((x[0] * 256 + x[1]) / 32768)},
    #     "012B": {"name": "Oxygen sensor (wideband) bank 2 sensor 4", "unit": "ratio", "formula": lambda x: ((x[0] * 256 + x[1]) / 32768)},
    #     "012C": {"name": "Commanded EGR", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "012D": {"name": "EGR Error", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "012E": {"name": "Commanded evaporative purge", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "012F": {"name": "Fuel Tank Level Input", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "0130": {"name": "Warm-ups since codes cleared", "unit": "count", "formula": lambda x: x[0]},
    #     "0131": {"name": "Distance traveled since codes cleared", "unit": "km", "formula": lambda x: x[0] * 256 + x[1]},
    #     "0132": {"name": "Evap. System Vapor Pressure", "unit": "Pa", "formula": lambda x: ((x[0] * 256 + x[1]) / 4)},
    #     "0133": {"name": "Absolut load value", "unit": "%", "formula": lambda x: (x[0] * 256 + x[1]) * 100 / 255},
    #     "0134": {"name": "Command equivalence ratio", "unit": "ratio", "formula": lambda x: (x[0] * 256 + x[1]) / 32768},
    #     "0135": {"name": "Relative throttle position", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "0136": {"name": "Ambient air temperature", "unit": "°C", "formula": lambda x: x[0] - 40},
    #     "0137": {"name": "Absolute throttle position B", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "0138": {"name": "Absolute throttle position C", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "0139": {"name": "Accelerator pedal position D", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "013A": {"name": "Accelerator pedal position E", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "013B": {"name": "Accelerator pedal position F", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "013C": {"name": "Commanded throttle actuator", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "013D": {"name": "Time run with MIL on", "unit": "min", "formula": lambda x: x[0] * 256 + x[1]},
    #     "013E": {"name": "Time since trouble codes cleared", "unit": "min", "formula": lambda x: x[0] * 256 + x[1]},
    #     "013F": {"name": "Maximum value for equivalence ratio, oxygen sensor voltage, oxygen sensor current, and intake manifold absolute pressure", "unit": "mixed", "formula": lambda x: x},
    #     "0140": {"name": "PIDs supported [41-60]", "unit": "bit", "formula": lambda x: x},
    #     "0141": {"name": "Maximum value for air flow rate from mass air flow sensor", "unit": "g/s", "formula": lambda x: x[0] * 10},
    #     "0142": {"name": "Fuel Type", "unit": "code", "formula": lambda x: x[0]},
    #     "0143": {"name": "Ethanol fuel %", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "0144": {"name": "Absolute Evap system Vapor Pressure", "unit": "kPa", "formula": lambda x: (x[0] * 256 + x[1]) / 200},
    #     "0145": {"name": "Evap system vapor pressure", "unit": "Pa", "formula": lambda x: x[0] * 256 + x[1] - 32767},
    #     "0146": {"name": "Short term secondary oxygen sensor trim, A: bank 1, B: bank 3", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "0147": {"name": "Long term secondary oxygen sensor trim, A: bank 1, B: bank 3", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "0148": {"name": "Short term secondary oxygen sensor trim, A: bank 2, B: bank 4", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "0149": {"name": "Long term secondary oxygen sensor trim, A: bank 2, B: bank 4", "unit": "%", "formula": lambda x: (x[0] - 128) * 100 / 128},
    #     "014A": {"name": "Fuel rail absolute pressure", "unit": "kPa", "formula": lambda x: (x[0] * 256 + x[1]) * 10},
    #     "014B": {"name": "Relative accelerator pedal position", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "014C": {"name": "Hybrid battery pack remaining life", "unit": "%", "formula": lambda x: x[0] * 100 / 255},
    #     "014D": {"name": "Engine oil temperature", "unit": "°C", "formula": lambda x: x[0] - 40},
    #     "014E": {"name": "Fuel injection timing", "unit": "°", "formula": lambda x: (x[0] * 256 + x[1] - 26880) / 128},
    #     "014F": {"name": "Engine fuel rate", "unit": "L/h", "formula": lambda x: (x[0] * 256 + x[1]) / 20},
    #     "0150": {"name": "Emission requirements", "unit": "bit", "formula": lambda x: x},
    #     "0151": {"name": "PIDs supported [51-70]", "unit": "bit", "formula": lambda x: x},
    #     "0152": {"name": "Driver's demand engine - percent torque", "unit": "%", "formula": lambda x: x[0] - 125},
    #     "0153": {"name": "Actual engine - percent torque", "unit": "%", "formula": lambda x: x[0] - 125},
    #     "0154": {"name": "Engine reference torque", "unit": "Nm", "formula": lambda x: x[0] * 256 + x[1]},
    # }
    
    @classmethod
    def get_pid_info(cls, pid: str) -> Dict[str, Any]:
        """Get PID information"""
        return cls.PIDS.get(pid.upper(), {
            "name": f"Unknown PID {pid}",
            "unit": "unknown",
            "formula": lambda x: x
        })

class DTC_Database:
    """Diagnostic Trouble Code database"""
    
    DTC_PREFIXES = {
        'P0': 'Powertrain - Generic',
        'P1': 'Powertrain - Manufacturer',
        'P2': 'Powertrain - Generic',
        'P3': 'Powertrain - Reserved',
        'B0': 'Body - Generic',
        'B1': 'Body - Manufacturer',
        'B2': 'Body - Generic',
        'B3': 'Body - Reserved',
        'C0': 'Chassis - Generic',
        'C1': 'Chassis - Manufacturer',
        'C2': 'Chassis - Generic',
        'C3': 'Chassis - Reserved',
        'U0': 'Network - Generic',
        'U1': 'Network - Manufacturer',
        'U2': 'Network - Generic',
        'U3': 'Network - Reserved',
    }
    
    COMMON_DTCS = dtc_dict
    # {
    #     'P0100': 'Mass or Volume Air Flow Circuit Malfunction',
    #     'P0101': 'Mass or Volume Air Flow Circuit Range/Performance Problem',
    #     'P0102': 'Mass or Volume Air Flow Circuit Low Input',
    #     'P0103': 'Mass or Volume Air Flow Circuit High Input',
    #     'P0104': 'Mass or Volume Air Flow Circuit Intermittent',
    #     'P0105': 'Manifold Absolute Pressure/Barometric Pressure Circuit Malfunction',
    #     'P0106': 'Manifold Absolute Pressure/Barometric Pressure Circuit Range/Performance Problem',
    #     'P0107': 'Manifold Absolute Pressure/Barometric Pressure Circuit Low Input',
    #     'P0108': 'Manifold Absolute Pressure/Barometric Pressure Circuit High Input',
    #     'P0109': 'Manifold Absolute Pressure/Barometric Pressure Circuit Intermittent',
    #     'P0110': 'Intake Air Temperature Circuit Malfunction',
    #     'P0111': 'Intake Air Temperature Circuit Range/Performance Problem',
    #     'P0112': 'Intake Air Temperature Circuit Low Input',
    #     'P0113': 'Intake Air Temperature Circuit High Input',
    #     'P0114': 'Intake Air Temperature Circuit Intermittent',
    #     'P0115': 'Engine Coolant Temperature Circuit Malfunction',
    #     'P0116': 'Engine Coolant Temperature Circuit Range/Performance Problem',
    #     'P0117': 'Engine Coolant Temperature Circuit Low Input',
    #     'P0118': 'Engine Coolant Temperature Circuit High Input',
    #     'P0119': 'Engine Coolant Temperature Circuit Intermittent',
    #     'P0120': 'Throttle/Pedal Position Sensor/Switch "A" Circuit Malfunction',
    #     'P0121': 'Throttle/Pedal Position Sensor/Switch "A" Circuit Range/Performance Problem',
    #     'P0122': 'Throttle/Pedal Position Sensor/Switch "A" Circuit Low Input',
    #     'P0123': 'Throttle/Pedal Position Sensor/Switch "A" Circuit High Input',
    #     'P0124': 'Throttle/Pedal Position Sensor/Switch "A" Circuit Intermittent',
    #     'P0125': 'Insufficient Coolant Temperature for Closed Loop Fuel Control',
    #     'P0126': 'Insufficient Coolant Temperature for Stable Operation',
    #     'P0127': 'Intake Air Temperature Too High',
    #     'P0128': 'Coolant Thermostat (Coolant Temperature Below Thermostat Regulating Temperature)',
    #     'P0129': 'Barometric Pressure Too Low',
    #     'P0130': 'O2 Sensor Circuit Malfunction (Bank 1 Sensor 1)',
    #     'P0131': 'O2 Sensor Circuit Low Voltage (Bank 1 Sensor 1)',
    #     'P0132': 'O2 Sensor Circuit High Voltage (Bank 1 Sensor 1)',
    #     'P0133': 'O2 Sensor Circuit Slow Response (Bank 1 Sensor 1)',
    #     'P0134': 'O2 Sensor Circuit No Activity Detected (Bank 1 Sensor 1)',
    #     'P0135': 'O2 Sensor Heater Circuit Malfunction (Bank 1 Sensor 1)',
    #     'P0136': 'O2 Sensor Circuit Malfunction (Bank 1 Sensor 2)',
    #     'P0137': 'O2 Sensor Circuit Low Voltage (Bank 1 Sensor 2)',
    #     'P0138': 'O2 Sensor Circuit High Voltage (Bank 1 Sensor 2)',
    #     'P0139': 'O2 Sensor Circuit Slow Response (Bank 1 Sensor 2)',
    #     'P0140': 'O2 Sensor Circuit No Activity Detected (Bank 1 Sensor 2)',
    #     'P0141': 'O2 Sensor Heater Circuit Malfunction (Bank 1 Sensor 2)',
    #     'P0171': 'System Too Lean (Bank 1)',
    #     'P0172': 'System Too Rich (Bank 1)',
    #     'P0173': 'Fuel Trim Malfunction (Bank 2)',
    #     'P0174': 'System Too Lean (Bank 2)',
    #     'P0175': 'System Too Rich (Bank 2)',
    #     'P0300': 'Random/Multiple Cylinder Misfire Detected',
    #     'P0301': 'Cylinder 1 Misfire Detected',
    #     'P0302': 'Cylinder 2 Misfire Detected',
    #     'P0303': 'Cylinder 3 Misfire Detected',
    #     'P0304': 'Cylinder 4 Misfire Detected',
    #     'P0305': 'Cylinder 5 Misfire Detected',
    #     'P0306': 'Cylinder 6 Misfire Detected',
    #     'P0307': 'Cylinder 7 Misfire Detected',
    #     'P0308': 'Cylinder 8 Misfire Detected',
    #     'P0420': 'Catalyst System Efficiency Below Threshold (Bank 1)',
    #     'P0430': 'Catalyst System Efficiency Below Threshold (Bank 2)',
    #     'P0442': 'Evaporative Emission Control System Leak Detected (Small Leak)',
    #     'P0446': 'Evaporative Emission Control System Vent Control Circuit Malfunction',
    #     'P0455': 'Evaporative Emission Control System Leak Detected (Gross Leak)',
    #     'P0456': 'Evaporative Emission Control System Leak Detected (Very Small Leak)',
    #     'P0506': 'Idle Control System RPM Lower Than Expected',
    #     'P0507': 'Idle Control System RPM Higher Than Expected',
    # }
    @classmethod
    def get_dtc_description(cls, code: str) -> str:
        """Get DTC description"""
        if code in cls.COMMON_DTCS:
            return cls.COMMON_DTCS[code]
        
        prefix = code[:2].upper()
        if prefix in cls.DTC_PREFIXES:
            return f"{cls.DTC_PREFIXES[prefix]} - {code}"
        
        return f"Unknown DTC: {code}"

class ELM327Scanner:
    """Main ELM327 Scanner class with comprehensive functionality"""
    
    def __init__(self, host: str = "192.168.0.10", port: int = 35000):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.protocol = None
        self.vin = None
        self.ecu_info = {}
        self.streaming = False
        self.log_file = None
        self.performance_data = {}
        
        # Initialize logging
        self.logger = logging.getLogger(__name__)
        
    def connect(self) -> bool:
        """Connect to ELM327 device"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((self.host, self.port))
            self.connected = True
            
            # Initialize ELM327
            if self._initialize_elm327():
                self.logger.info(f"Connected to ELM327 at {self.host}:{self.port}")
                return True
            else:
                self.disconnect()
                return False
                
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from ELM327 device"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        self.connected = False
        self.streaming = False
        self.logger.info("Disconnected from ELM327")
    
    def _initialize_elm327(self) -> bool:
        """Initialize ELM327 with proper settings"""
        try:
            # Reset device
            if not self._send_command("ATZ"):
                return False
            time.sleep(2)
            
            # Turn off echo
            if not self._send_command("ATE0"):
                return False
            
            # Turn off line feeds
            if not self._send_command("ATL0"):
                return False
            
            # Turn off spaces
            if not self._send_command("ATS0"):
                return False
            
            # Set headers off
            if not self._send_command("ATH0"):
                return False
            
            # Adaptive timing auto
            if not self._send_command("ATAT1"):
                return False
            
            # Try automatic protocol detection
            response = self._send_command("ATSP0")
            if response:
                self.logger.info("ELM327 initialized successfully")
                return True
            
            return False
            
        except Exception as e:
            self.logger.error(f"ELM327 initialization failed: {e}")
            return False
    
    def _send_command(self, command: str, timeout: float = 5.0) -> str:
        """Send command to ELM327 and return response"""
        if not self.connected or not self.socket:
            return ""
        
        try:
            # Send command
            cmd = f"{command}\r"
            self.socket.send(cmd.encode('ascii'))
            
            # Receive response
            response = ""
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                try:
                    data = self.socket.recv(1024).decode('ascii', errors='ignore')
                    if not data:
                        break
                    
                    response += data
                    
                    # Check for prompt
                    if '>' in response:
                        break
                        
                except socket.timeout:
                    break
                    
            # Clean up response
            response = response.replace('\r', '').replace('\n', '').replace('>', '').strip()
            
            # Filter out echo and common responses
            if response.startswith(command):
                response = response.strip()
            
            return response
            
        except Exception as e:
            self.logger.error(f"Command failed: {command} - {e}")
            return ""
    
    def _parse_hex_response(self, response: str) -> List[int]:
        """Parse hex response into byte array"""
        try:
            # Remove spaces and common error responses
            clean_response = response.replace(' ', '').replace('\r', '').replace('\n', '')
            
            # Check for error responses
            error_responses = ['NODATA', 'ERROR', 'TIMEOUT', '?', 'UNABLETOCONNECT', 'BUSBUSY']
            if any(err in clean_response.upper() for err in error_responses):
                return []
            
            # Convert hex pairs to integers
            if len(clean_response) % 2 == 0:
                return [int(clean_response[i:i+2], 16) for i in range(0, len(clean_response), 2)]
            else:
                return []
                
        except Exception as e:
            self.logger.error(f"Failed to parse hex response: {response} - {e}")
            return []
    
    def get_supported_pids(self, mode: str = "01") -> List[str]:
        """Get list of supported PIDs for given mode"""
        supported_pids = []
        
        try:
            # Check PID support ranges
            pid_ranges = ["00", "20", "40", "60", "80", "A0", "C0", "E0"]
            
            for pid_range in pid_ranges:
                command = f"{mode}{pid_range}"
                response = self._send_command(command)
                
                if response and "NODATA" not in response.upper():
                    data = self._parse_hex_response(response)
                    if len(data) >= 6:  # Mode + PID + 4 bytes of data
                        # Skip mode and PID bytes
                        pid_data = data[2:6]
                        
                        # Parse supported PIDs from bitmask
                        base_pid = int(pid_range, 16)
                        for byte_idx, byte_val in enumerate(pid_data):
                            for bit_idx in range(8):
                                if byte_val & (0x80 >> bit_idx):
                                    pid_num = base_pid + byte_idx * 8 + bit_idx + 1
                                    if pid_num <= 255:
                                        supported_pids.append(f"{mode}{pid_num:02X}")
            
        except Exception as e:
            self.logger.error(f"Failed to get supported PIDs: {e}")
        
        return supported_pids
    
    def signals_values(self,  bits_list:list, data_bits: str) -> List:
        """give each signal it is value """
        signalsv = []
        dbits = bits_list
        for b in bits_list:
            value = dbits[b[1]:b[0]-1]
            print(value)
            signalsv.append(value)

        return signalsv
    
    def read_pid(self, pid: str) -> Optional[SensorReading]:
        """Read a specific PID and return sensor reading"""
        try:
            response = self._send_command(pid)
            if not response or "NODATA" in response.upper():
                return None
            
            bits = bin(int(response, 16))[2:].zfill(len(response) * 4)

            data = self._parse_hex_response(response)
            if len(data) < 3:  # At least mode + PID + 1 data byte
                return None
            
            
            # Extract data bytes (skip mode and PID)
            data_bytes = data[2:]
            dbyte = data_bytes
            # Get PID info and calculate value
            pid_info = PIDDatabase.get_pid_info(pid)
            
            signals = pid_info["signals"].keys()
            signals_list = []
            try:

                bits_list = []
                for s in signals:
                    l = pid_info["signals"][s]['bit_length']
                    i = pid_info["signals"][s].setdefault("bit_index", 0)

                    bits_list.append([l, i])
                
                
                signalsv = self.signals_values(bits_list, bits)
                print(signalsv)
#  {
#        'value': str,
#        'path': str,
#        'name': str,
#        'unit': str,
#     }
# "bit_length": 1,
# "bit_length": 16,
# "bit_length": 8,
                c = 0
                for s in signals:
                    ss = pid_info["signals"][s]
                    l = pid_info["signals"][s]['bit_length']
                    formula = ss.setdefault('formula', lambda x: x)

                    if (ss['bit_length']/8 >= 1):
                        num = int(signalsv[c], 2)   # convert to decimal
                        sss = {
                                'value': float(formula(num)),
                                'path': ss['path'],
                                'name': ss['name'],
                                'unit': ss['unit'],
                            }
                        signals_list.append(sss)

                    c += 1


                    
                    
                    
                

            except:
                sss = {
                    'value': bits,
                    'path': 'everything',
                    'name': 'everything',
                    'unit': '',
                }
                signals_list.append(sss)


            
            return SensorReading(
                pid=pid,
                name=pid_info["name"],
                signals=signals_list,
                timestamp=datetime.now(),
                raw_data=response,
            )
            
        except Exception as e:
            self.logger.error(f"Failed to read PID {pid}: {e}")
            return None
        
    
    def get_dtcs(self, mode: str = "03") -> list[DTCInfo]:
        """Read Diagnostic Trouble Codes (Mode 03)"""
        dtcs = []

        try:
            response = self._send_command(mode)
            if not response or "NODATA" in response.upper():
                return dtcs

            # Remove spaces and split into bytes
            hex_bytes = self._parse_hex_response(response)  # should return list of ints

            # Each DTC is 2 bytes
            for i in range(0, len(hex_bytes), 2):
                if i + 1 >= len(hex_bytes):
                    break  # incomplete DTC

                b1, b2 = hex_bytes[i], hex_bytes[i + 1]

                # Skip empty DTCs
                if b1 == 0 and b2 == 0:
                    continue

                dtc_code = self._decode_dtc([b1, b2])
                if dtc_code:
                    dtcs.append(DTCInfo(
                        code=dtc_code,
                        description=DTC_Database.get_dtc_description(dtc_code),
                        status="Active"
                    ))

        except Exception as e:
            self.logger.error(f"Failed to read DTCs: {e}")

        return dtcs

    
    def get_pending_dtcs(self) -> List[DTCInfo]:
        """Read Pending Diagnostic Trouble Codes"""
        dtcs = []
        
        try:
            response = self._send_command("07")
            if not response or "NODATA" in response.upper():
                return dtcs
            
            data = self._parse_hex_response(response)
            if len(data) < 2:
                return dtcs
            
            # First byte is number of DTCs
            num_dtcs = data[0]
            
            # Each DTC is 2 bytes
            for i in range(1, min(len(data), 1 + num_dtcs * 2), 2):
                if i + 1 < len(data):
                    dtc_bytes = [data[i], data[i + 1]]
                    dtc_code = self._decode_dtc(dtc_bytes)
                    
                    if dtc_code:
                        dtcs.append(DTCInfo(
                            code=dtc_code,
                            description=DTC_Database.get_dtc_description(dtc_code),
                            status="Pending"
                        ))
            
        except Exception as e:
            self.logger.error(f"Failed to read pending DTCs: {e}")
        
        return dtcs
    
    def clear_dtcs(self) -> bool:
        """Clear Diagnostic Trouble Codes"""
        try:
            response = self._send_command("04")
            return "OK" in response.upper() or response == ""
        except Exception as e:
            self.logger.error(f"Failed to clear DTCs: {e}")
            return False
    
    def _decode_dtc(self, dtc_bytes: List[int]) -> str:
        """Decode DTC bytes to standard format"""
        if len(dtc_bytes) != 2:
            return ""
        
        try:
            # First two bits determine the system
            system_bits = (dtc_bytes[0] & 0xC0) >> 6
            system_map = {0: 'P', 1: 'C', 2: 'B', 3: 'U'}
            system = system_map.get(system_bits, 'P')
            
            # Remaining 14 bits are the code
            code_value = ((dtc_bytes[0] & 0x3F) << 8) | dtc_bytes[1]
            
            return f"{system}{code_value:04X}"
            
        except Exception as e:
            self.logger.error(f"Failed to decode DTC: {dtc_bytes} - {e}")
            return ""
    
    def get_freeze_frame_data(self, dtc_index: int = 0) -> Dict[str, Any]:
        """Read freeze frame data"""
        freeze_data = {}
        
        try:
            # Mode 02 - Freeze Frame Data
            response = self._send_command(f"02{dtc_index:02X}")
            if response and "NODATA" not in response.upper():
                data = self._parse_hex_response(response)
                
                # Parse freeze frame PIDs (implementation depends on vehicle)
                if len(data) >= 3:
                    freeze_data["dtc_index"] = dtc_index
                    freeze_data["timestamp"] = datetime.now()
                    freeze_data["raw_data"] = response
                    
                    # Common freeze frame PIDs
                    freeze_pids = ["05", "0C", "0D", "0F", "11"]  # Common freeze frame PIDs
                    for pid in freeze_pids:
                        reading = self.read_pid(f"02{pid}")
                        if reading:
                            freeze_data[pid] = {
                                "name": reading.name,
                                "value": reading.value,
                                "unit": reading.unit
                            }
            
        except Exception as e:
            self.logger.error(f"Failed to read freeze frame data: {e}")
        
        return freeze_data
    
    def get_vehicle_info(self) -> Dict[str, str]:
        """Get comprehensive vehicle information"""
        info = {}
        
        try:
            # VIN (Mode 09, PID 02)
            vin = self.get_vin()
            if vin:
                info["VIN"] = vin
            
            # Calibration ID (Mode 09, PID 04)
            cal_id = self._send_command("0904")
            if cal_id and "NODATA" not in cal_id.upper():
                info["Calibration_ID"] = cal_id
            
            # CVN (Mode 09, PID 06)
            cvn = self._send_command("0906")
            if cvn and "NODATA" not in cvn.upper():
                info["CVN"] = cvn
            
            # ECU Name (Mode 09, PID 0A)
            ecu_name = self._send_command("090A")
            if ecu_name and "NODATA" not in ecu_name.upper():
                info["ECU_Name"] = ecu_name
            
            # Get protocol info
            protocol_info = self._send_command("ATDP")
            if protocol_info:
                info["Protocol"] = protocol_info
            
        except Exception as e:
            self.logger.error(f"Failed to get vehicle info: {e}")
        
        return info
    
    def get_vin(self) -> str:
        """Get Vehicle Identification Number"""
        try:
            if self.vin:
                return self.vin
            
            # Mode 09, PID 02 - VIN
            response = self._send_command("0902")
            if response and "NODATA" not in response.upper():
                data = self._parse_hex_response(response)
                
                # VIN is typically 17 ASCII characters
                if len(data) >= 19:  # Mode + PID + 17 VIN bytes
                    vin_bytes = data[2:19]  # Skip mode and PID
                    vin = ''.join(chr(b) for b in vin_bytes if 32 <= b <= 126)
                    
                    if len(vin) == 17:
                        self.vin = vin
                        return vin
            
            return ""
            
        except Exception as e:
            self.logger.error(f"Failed to get VIN: {e}")
            return ""
    
    def get_readiness_monitors(self) -> Dict[str, str]:
        """Get OBD monitor readiness status"""
        monitors = {}
        
        try:
            # Mode 01, PID 01 - Monitor status
            response = self._send_command("0101")
            if response and "NODATA" not in response.upper():
                data = self._parse_hex_response(response)
                
                if len(data) >= 6:  # Mode + PID + 4 data bytes
                    status_bytes = data[2:6]
                    
                    # Parse monitor status bits
                    monitors["MIL_Status"] = "ON" if status_bytes[0] & 0x80 else "OFF"
                    monitors["DTC_Count"] = status_bytes[0] & 0x7F
                    
                    # Continuous monitors (byte 1)
                    continuous = status_bytes[1]
                    monitors["Misfire"] = "Ready" if not (continuous & 0x01) else "Not Ready"
                    monitors["Fuel_System"] = "Ready" if not (continuous & 0x02) else "Not Ready"
                    monitors["Components"] = "Ready" if not (continuous & 0x04) else "Not Ready"
                    
                    # Non-continuous monitors (bytes 2-3)
                    non_continuous_supported = (status_bytes[2] << 8) | status_bytes[3]
                    non_continuous_ready = (status_bytes[2] << 8) | status_bytes[3]
                    
                    monitor_names = [
                        "Catalyst", "Heated_Catalyst", "Evaporative_System", "Secondary_Air",
                        "A/C_Refrigerant", "Oxygen_Sensor", "Oxygen_Sensor_Heater", "EGR_System"
                    ]
                    
                    for i, name in enumerate(monitor_names):
                        if non_continuous_supported & (1 << i):
                            monitors[name] = "Ready" if not (non_continuous_ready & (1 << (i + 8))) else "Not Ready"
            
        except Exception as e:
            self.logger.error(f"Failed to get readiness monitors: {e}")
        
        return monitors
     
    def start_live_data_stream(self, pids: List[str], interval: float = 1.0, enable_graph: bool = False):
        """Start streaming live sensor data"""
        if self.streaming:
            self.stop_live_data_stream()
        
        self.streaming = True
        
        # Initialize shared data structures for graphing
        if enable_graph and MATPLOTLIB_AVAILABLE:
            self.graph_data = {
                'pids': pids,
                'data_history': {},
                'time_history': {},
                'max_points': 50
            }
            
            for pid in pids:
                self.graph_data['data_history'][pid] = deque(maxlen=50)
                self.graph_data['time_history'][pid] = deque(maxlen=50)
        
        self.stream_thread = threading.Thread(
            target=self._stream_worker,
            args=(pids, interval, enable_graph),
            daemon=True
        )
        self.stream_thread.start()
        
        # Start graph in main thread if enabled
        if enable_graph and MATPLOTLIB_AVAILABLE:
            self._start_graph_display()
    
    def _start_graph_display(self):
        """Start graph display in main thread"""
        try:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(12, 8))
            self.lines = {}
            
            # Set up the plot
            for pid in self.graph_data['pids']:
                pid_info = PIDDatabase.get_pid_info(pid)
                label = f"{pid}: {pid_info['name']} ({pid_info['unit']})"
                line, = self.ax.plot([], [], marker='o', markersize=3, label=label, linewidth=2)
                self.lines[pid] = line
            
            self.ax.set_xlabel("Time", fontsize=12)
            self.ax.set_ylabel("Sensor Values", fontsize=12)
            self.ax.set_title("Live OBD-II Sensor Data", fontsize=14, fontweight='bold')
            self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            self.ax.grid(True, alpha=0.3)
            
            # Set up time formatting
            self.ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
            self.ax.xaxis.set_major_locator(mdates.SecondLocator(interval=10))
            
            # Limit number of ticks to prevent warnings
            self.ax.yaxis.set_major_locator(ticker.MaxNLocator(nbins=8))
            self.ax.xaxis.set_major_locator(ticker.MaxNLocator(nbins=6))
            
            plt.tight_layout()
            plt.show(block=False)
            
            # Start update timer
            self.graph_timer = threading.Timer(0.5, self._update_graph)
            self.graph_timer.start()
            
        except Exception as e:
            self.logger.error(f"Failed to start graph display: {e}")
            print(f"Graph display failed: {e}")
    
    def _update_graph(self):
        """Update graph periodically"""
        if not self.streaming:
            return
        
        try:
            # Update lines with current data
            for pid in self.graph_data['pids']:
                if len(self.graph_data['time_history'][pid]) > 0:
                    times = list(self.graph_data['time_history'][pid])
                    values = list(self.graph_data['data_history'][pid])
                    
                    self.lines[pid].set_xdata(times)
                    self.lines[pid].set_ydata(values)
            
            # Rescale axes
            self.ax.relim()
            self.ax.autoscale_view()
            
            # Format x-axis dates
            self.fig.autofmt_xdate()
            
            # Refresh display
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
            # Schedule next update
            if self.streaming:
                self.graph_timer = threading.Timer(0.5, self._update_graph)
                self.graph_timer.start()
            
        except Exception as e:
            self.logger.error(f"Graph update error: {e}")
            # Continue streaming without graph
    
    def stop_live_data_stream(self):
        """Stop streaming live sensor data"""
        self.streaming = False
        
        # Stop graph timer
        if hasattr(self, 'graph_timer'):
            self.graph_timer.cancel()
        
        # Wait for stream thread
        if hasattr(self, 'stream_thread'):
            self.stream_thread.join(timeout=2)
        
        # Close matplotlib
        if MATPLOTLIB_AVAILABLE and hasattr(self, 'fig'):
            try:
                plt.close(self.fig)
                plt.ioff()
            except:
                pass
    
    def _stream_worker(self, pids: List[str], interval: float, enable_graph: bool = False):
        """Worker thread for live data streaming"""        
        start_time = time.time()
        
        while self.streaming:
            try:
                current_time = datetime.now()
                readings = {}
                
                # Read all PIDs
                for pid in pids:
                    reading = self.read_pid(pid)
                    if reading:
                        readings[pid] = reading
                
                if readings:
                    self._log_readings(readings)
                    
                    # Console output
                    if not enable_graph or not MATPLOTLIB_AVAILABLE:
                        self._display_live_readings(readings)
                    else:
                        # Brief console output when graphing
                        elapsed = time.time() - start_time
                        print(f"\rLive Data [{elapsed:.1f}s]: ", end="")
                        for i, (pid, reading) in enumerate(readings.items()):
                            if i > 0:
                                print(" | ", end="")
                            print(f"{reading.name}: {reading.value:.1f}{reading.unit}", end="")
                        sys.stdout.flush()
                    
                    # Update graph data if enabled
                    if enable_graph and MATPLOTLIB_AVAILABLE and hasattr(self, 'graph_data'):
                        try:
                            for pid, reading in readings.items():
                                self.graph_data['data_history'][pid].append(reading.value)
                                self.graph_data['time_history'][pid].append(current_time)
                        except Exception as graph_error:
                            self.logger.error(f"Graph data update error: {graph_error}")
                
                time.sleep(max(0.1, interval - 0.1))  # Account for processing time
                
            except Exception as e:
                self.logger.error(f"Stream worker error: {e}")
                break
    
    def _display_live_readings(self, readings: Dict[str, SensorReading]):
        """Display live readings to console"""
        print("\n" + "="*60)
        print(f"Live Data - {datetime.now().strftime('%H:%M:%S')}")
        print("="*60)
        
        for pid, reading in readings.items():
            print(f"{reading.name:<35}: {reading.value:>8.2f} {reading.unit}")
        
        print("="*60)
    
    def _log_readings(self, readings: Dict[str, SensorReading]):
        """Log readings to file"""
        if not self.log_file:
            return
        
        try:
            with open(self.log_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write data row
                row = [datetime.now().isoformat()]
                for reading in readings.values():
                    row.extend([reading.pid, reading.name, reading.value, reading.unit])
                
                writer.writerow(row)
                
        except Exception as e:
            self.logger.error(f"Failed to log readings: {e}")
    
    def start_logging(self, filename: str = None):
        """Start logging sensor data to file"""
        if not filename:
            filename = f"obd_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        self.log_file = filename
        
        # Write CSV header
        try:
            with open(self.log_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Timestamp", "PID", "Name", "Value", "Unit"])
                
            self.logger.info(f"Started logging to {filename}")
            
        except Exception as e:
            self.logger.error(f"Failed to start logging: {e}")
            self.log_file = None
    
    def stop_logging(self):
        """Stop logging sensor data"""
        if self.log_file:
            self.logger.info(f"Stopped logging to {self.log_file}")
            self.log_file = None
    
    def set_protocol(self, protocol: OBDProtocol) -> bool:
        """Set OBD protocol"""
        try:
            command = f"ATSP{protocol.value}"
            response = self._send_command(command)
            
            if "OK" in response.upper():
                self.protocol = protocol
                self.logger.info(f"Protocol set to {protocol.name}")
                return True
            else:
                self.logger.error(f"Failed to set protocol: {response}")
                return False
                
        except Exception as e:
            self.logger.error(f"Protocol setting failed: {e}")
            return False
    
    def get_protocol_info(self) -> str:
        """Get current protocol information"""
        try:
            response = self._send_command("ATDP")
            return response if response else "Unknown"
        except Exception as e:
            self.logger.error(f"Failed to get protocol info: {e}")
            return "Error"
    
    def send_at_command(self, command: str) -> str:
        """Send raw AT command to ELM327"""
        try:
            if not command.startswith("AT"):
                command = "AT" + command
            
            response = self._send_command(command)
            return response
            
        except Exception as e:
            self.logger.error(f"AT command failed: {e}")
            return ""
    
    def perform_o2_sensor_test(self) -> Dict[str, Any]:
        """Perform oxygen sensor tests"""
        results = {}
        
        try:
            # Check O2 sensor presence
            o2_presence = self._send_command("0113")
            if o2_presence and "NODATA" not in o2_presence.upper():
                results["sensors_present"] = o2_presence
            
            # Read O2 sensor voltages (PIDs 0114-011B)
            for i in range(8):
                pid = f"01{0x14 + i:02X}"
                reading = self.read_pid(pid)
                if reading:
                    results[f"sensor_{i+1}_voltage"] = {
                        "value": reading.value,
                        "unit": reading.unit,
                        "status": "OK" if 0.1 <= reading.value <= 0.9 else "Check"
                    }
            
            # Read wide-range O2 sensors (PIDs 0124-012B)
            for i in range(8):
                pid = f"01{0x24 + i:02X}"
                reading = self.read_pid(pid)
                if reading:
                    results[f"wr_sensor_{i+1}_lambda"] = {
                        "value": reading.value,
                        "unit": reading.unit,
                        "status": "OK" if 0.5 <= reading.value <= 1.5 else "Check"
                    }
            
        except Exception as e:
            self.logger.error(f"O2 sensor test failed: {e}")
        
        return results
    
    def test_actuators(self) -> Dict[str, Any]:
        """Test various actuators (where supported)"""
        results = {}
        
        try:
            # This is a placeholder - actual actuator tests depend on vehicle
            # and may require special mode commands
            
            # Test EGR valve (if supported)
            egr_test = self._send_command("012C")  # Commanded EGR
            if egr_test and "NODATA" not in egr_test.upper():
                results["egr_test"] = "Supported"
            
            # Test EVAP purge (if supported)
            evap_test = self._send_command("012E")  # Commanded evaporative purge
            if evap_test and "NODATA" not in evap_test.upper():
                results["evap_test"] = "Supported"
            
            # Test throttle actuator (if supported)
            throttle_test = self._send_command("013C")  # Commanded throttle actuator
            if throttle_test and "NODATA" not in throttle_test.upper():
                results["throttle_test"] = "Supported"
            
            results["note"] = "Actuator testing requires vehicle-specific commands"
            
        except Exception as e:
            self.logger.error(f"Actuator test failed: {e}")
        
        return results
    
    def monitor_battery_voltage(self) -> float:
        """Monitor vehicle battery voltage"""
        try:
            response = self._send_command("ATRV")
            if response:
                # Parse voltage from response
                voltage_match = re.search(r'(\d+\.\d+)V', response)
                if voltage_match:
                    voltage = float(voltage_match.group(1))
                    return voltage
                else:
                    # Try to parse just the number
                    try:
                        voltage = float(response.replace('V', '').strip())
                        return voltage
                    except:
                        pass
            
            return 0.0
            
        except Exception as e:
            self.logger.error(f"Battery voltage monitoring failed: {e}")
            return 0.0
    
    def calculate_fuel_economy(self, distance_km: float, fuel_used_l: float) -> Dict[str, float]:
        """Calculate fuel economy metrics"""
        results = {}
        
        try:
            if distance_km > 0 and fuel_used_l > 0:
                # L/100km (metric)
                results["l_per_100km"] = (fuel_used_l / distance_km) * 100
                
                # MPG (imperial)
                miles = distance_km * 0.621371
                gallons = fuel_used_l * 0.264172
                if gallons > 0:
                    results["mpg"] = miles / gallons
                
                # km/L (metric)
                results["km_per_liter"] = distance_km / fuel_used_l
            
        except Exception as e:
            self.logger.error(f"Fuel economy calculation failed: {e}")
        
        return results
    
    def performance_test_0_to_100(self) -> Dict[str, Any]:
        """Perform 0-100 km/h acceleration test"""
        results = {
            "test_type": "0-100 km/h acceleration",
            "start_time": None,
            "end_time": None,
            "duration": None,
            "max_speed": 0,
            "readings": []
        }
        
        print("Starting 0-100 km/h acceleration test...")
        print("Waiting for vehicle speed > 5 km/h to start...")
        
        try:
            # Wait for initial movement
            while True:
                speed_reading = self.read_pid("010D")  # Vehicle speed
                if speed_reading and speed_reading.value > 5:
                    results["start_time"] = datetime.now()
                    print(f"Test started at {results['start_time'].strftime('%H:%M:%S.%f')[:-3]}")
                    break
                time.sleep(0.1)
            
            # Monitor until 100 km/h or timeout
            start_time = time.time()
            while time.time() - start_time < 60:  # 60 second timeout
                speed_reading = self.read_pid("010D")
                rpm_reading = self.read_pid("010C")
                
                if speed_reading:
                    current_speed = speed_reading.value
                    current_time = datetime.now()
                    
                    results["readings"].append({
                        "time": current_time,
                        "speed": current_speed,
                        "rpm": rpm_reading.value if rpm_reading else 0
                    })
                    
                    results["max_speed"] = max(results["max_speed"], current_speed)
                    
                    if current_speed >= 100:
                        results["end_time"] = current_time
                        break
                
                time.sleep(0.1)
            
            # Calculate duration
            if results["start_time"] and results["end_time"]:
                duration = (results["end_time"] - results["start_time"]).total_seconds()
                results["duration"] = duration
                print(f"0-100 km/h completed in {duration:.2f} seconds")
            else:
                print("Test did not complete (didn't reach 100 km/h)")
                
        except Exception as e:
            self.logger.error(f"Performance test failed: {e}")
            results["error"] = str(e)
        
        return results
    
    def get_comprehensive_vehicle_scan(self) -> Dict[str, Any]:
        """Perform comprehensive vehicle scan"""
        scan_results = {
            "timestamp": datetime.now(),
            "connection_info": {
                "host": self.host,
                "port": self.port,
                "connected": self.connected
            }
        }
        
        print("Performing comprehensive vehicle scan...")
        
        # Vehicle information
        print("Gathering vehicle information...")
        scan_results["vehicle_info"] = self.get_vehicle_info()
        
        # Protocol information
        scan_results["protocol"] = self.get_protocol_info()
        
        # Supported PIDs
        print("Scanning supported PIDs...")
        scan_results["supported_pids"] = self.get_supported_pids()
        
        # Current DTCs
        print("Reading diagnostic trouble codes...")
        scan_results["active_dtcs"] = self.get_dtcs()
        scan_results["pending_dtcs"] = self.get_pending_dtcs()
        
        # Readiness monitors
        print("Checking readiness monitors...")
        scan_results["readiness_monitors"] = self.get_readiness_monitors()
        
        # Current sensor readings
        print("Reading current sensor values...")
        current_readings = {}
        common_pids = ["0104", "0105", "010C", "010D", "010F", "0110", "0111"]
        
        for pid in common_pids:
            reading = self.read_pid(pid)
            if reading:
                current_readings[pid] = {
                    "name": reading.name,
                    "value": reading.value,
                    "unit": reading.unit
                }
        
        scan_results["current_readings"] = current_readings
        
        # Battery voltage
        scan_results["battery_voltage"] = self.monitor_battery_voltage()
        
        # O2 sensor test
        print("Testing oxygen sensors...")
        scan_results["o2_sensor_test"] = self.perform_o2_sensor_test()
        
        print("Comprehensive scan completed!")
        return scan_results

class OBDCommandInterface:
    """Command-line interface for OBD scanner"""
    
    def __init__(self):
        self.scanner = None
        self.running = True
        
    def start(self):
        """Start the command interface"""
        self.display_welcome()
        
        while self.running:
            try:
                command = input("\nOBD> ").strip().lower()
                self.process_command(command)
                
            except KeyboardInterrupt:
                print("\nExiting...")
                self.running = False
            except EOFError:
                print("\nExiting...")
                self.running = False
            except Exception as e:
                print(f"Error: {e}")
    
    def display_welcome(self):
        """Display welcome message and help"""
        print("\n" + "="*70)
        print("ELM327 WiFi OBD-II Professional Scanner Tool v1.0.0")
        print("Advanced OBD Tools - Production Ready")
        print("="*70)
        print("Type 'help' for available commands")
        print("Type 'connect' to connect to your ELM327 device")
        print("Default connection: 192.168.0.10:35000")
        print("="*70)
    
    def process_command(self, command: str):
        """Process user commands"""
        parts = command.split()
        
        if not parts:
            return
        
        cmd = parts[0]
        args = parts[1:] if len(parts) > 1 else []
        
        # Command routing
        command_map = {
            'help': self.cmd_help,
            'connect': self.cmd_connect,
            'disconnect': self.cmd_disconnect,
            'status': self.cmd_status,
            'scan': self.cmd_scan,
            'dtc': self.cmd_dtc,
            'clear': self.cmd_clear_dtc,
            'live': self.cmd_live_data,
            'stop': self.cmd_stop_live,
            'pid': self.cmd_read_pid,
            'vin': self.cmd_vin,
            'monitors': self.cmd_monitors,
            'freeze': self.cmd_freeze_frame,
            'protocol': self.cmd_protocol,
            'at': self.cmd_at_command,
            'o2test': self.cmd_o2_test,
            'actuator': self.cmd_actuator_test,
            'battery': self.cmd_battery,
            'performance': self.cmd_performance,
            'log': self.cmd_logging,
            'fuel': self.cmd_fuel_economy,
            'exit': self.cmd_exit,
            'quit': self.cmd_exit,
            "show_pids": self.print_PIDs,
            
        }
        
        if cmd in command_map:
            command_map[cmd](args)
        else:
            print(f"Unknown command: {cmd}. Type 'help' for available commands.")
    
    def cmd_help(self, args):
        """Display help information"""
        help_text = """
Available Commands:
==================

Connection Management:
  connect [host] [port]    - Connect to ELM327 (default: 192.168.0.10:35000)
  disconnect              - Disconnect from ELM327
  status                  - Show connection status

Diagnostic Functions:
  scan                    - Perform comprehensive vehicle scan
  dtc                     - Read diagnostic trouble codes
  clear                   - Clear diagnostic trouble codes
  monitors                - Check readiness monitors
  freeze [index]          - Read freeze frame data

Data Reading:
  live [pids] [--graph]  - Start live data streaming (e.g., live 010C 010D --graph)
  stop                   - Stop live data streaming
  pid <pid>              - Read specific PID (e.g., pid 010C)
  vin                    - Read vehicle identification number

Protocol & Settings:
  protocol [num]         - Set/show OBD protocol (0=auto, 1-A=specific)
  at <command>           - Send raw AT command

Testing & Monitoring:
  o2test                 - Test oxygen sensors
  actuator               - Test actuators (where supported)
  battery                - Monitor battery voltage
  performance            - Run 0-100 km/h acceleration test

Data Management:
  log start [file]       - Start logging to file
  log stop               - Stop logging
  fuel <km> <liters>     - Calculate fuel economy

System:
  help                   - Show this help
  exit/quit              - Exit program
  show_pids

Examples:
  connect 192.168.4.1 35000
  live 010C 010D 0105 --graph  (RPM, Speed, Coolant with graph)
  live 010C 010D             (RPM, Speed without graph)
  pid 010C                   (Read engine RPM)
  protocol 6                 (Set to CAN 11-bit 500k)
  at rv                      (Read voltage with AT command)
        """
        print(help_text)
    
    def cmd_connect(self, args):
        """Connect to ELM327"""
        host = args[0] if args else "192.168.0.10"
        port = int(args[1]) if len(args) > 1 else 35000
        
        print(f"Connecting to ELM327 at {host}:{port}...")
        
        self.scanner = ELM327Scanner(host, port)
        
        if self.scanner.connect():
            print("✓ Connected successfully!")
            print(f"Protocol: {self.scanner.get_protocol_info()}")
            print(f"Battery: {self.scanner.monitor_battery_voltage():.1f}V")
        else:
            print("✗ Connection failed!")
            self.scanner = None
    
    def cmd_disconnect(self, args):
        """Disconnect from ELM327"""
        if self.scanner:
            self.scanner.disconnect()
            self.scanner = None
            print("Disconnected from ELM327")
        else:
            print("Not connected")
    
    def cmd_status(self, args):
        """Show connection status"""
        if self.scanner and self.scanner.connected:
            print("Status: Connected")
            print(f"Host: {self.scanner.host}:{self.scanner.port}")
            print(f"Protocol: {self.scanner.get_protocol_info()}")
            print(f"Battery: {self.scanner.monitor_battery_voltage():.1f}V")
            if self.scanner.vin:
                print(f"VIN: {self.scanner.vin}")
        else:
            print("Status: Disconnected")
    
    def cmd_scan(self, args):
        """Perform comprehensive scan"""
        if not self._check_connection():
            return
        
        results = self.scanner.get_comprehensive_vehicle_scan()
        
        print("\n" + "="*60)
        print("COMPREHENSIVE VEHICLE SCAN RESULTS")
        print("="*60)
        
        # Vehicle Info
        if "vehicle_info" in results:
            print("\nVehicle Information:")
            for key, value in results["vehicle_info"].items():
                print(f"  {key}: {value}")
        
        # Protocol
        if "protocol" in results:
            print(f"\nProtocol: {results['protocol']}")
        
        # DTCs
        if "active_dtcs" in results:
            print(f"\nActive DTCs: {len(results['active_dtcs'])}")
            for dtc in results["active_dtcs"]:
                print(f"  {dtc.code}: {dtc.description}")
        
        if "pending_dtcs" in results:
            print(f"\nPending DTCs: {len(results['pending_dtcs'])}")
            for dtc in results["pending_dtcs"]:
                print(f"  {dtc.code}: {dtc.description}")
        
        # Readiness Monitors
        if "readiness_monitors" in results:
            print("\nReadiness Monitors:")
            for monitor, status in results["readiness_monitors"].items():
                print(f"  {monitor}: {status}")
        
        # Current Readings
        if "current_readings" in results:
            print("\nCurrent Sensor Readings:")
            for pid, reading in results["current_readings"].items():
                print(f"  {reading['name']}: {reading['value']} {reading['unit']}")
        
        # Battery
        if "battery_voltage" in results:
            print(f"\nBattery Voltage: {results['battery_voltage']:.1f}V")
        
        # Supported PIDs
        if "supported_pids" in results:
            print(f"\nSupported PIDs: {len(results['supported_pids'])}")
            pids_per_line = 8
            for i in range(0, len(results["supported_pids"]), pids_per_line):
                pid_group = results["supported_pids"][i:i+pids_per_line]
                print("  " + " ".join(pid_group))
        
        print("\n" + "="*60)
    
    def cmd_dtc(self, args):
        """Read diagnostic trouble codes"""
        if not self._check_connection():
            return
        
        print("Reading diagnostic trouble codes...")
        
        # Active DTCs
        active_dtcs = self.scanner.get_dtcs()
        print(f"\nActive DTCs: {len(active_dtcs)}")
        for dtc in active_dtcs:
            print(f"  {dtc.code}: {dtc.description}")
        
        # Pending DTCs
        pending_dtcs = self.scanner.get_pending_dtcs()
        print(f"\nPending DTCs: {len(pending_dtcs)}")
        for dtc in pending_dtcs:
            print(f"  {dtc.code}: {dtc.description}")
        
        if not active_dtcs and not pending_dtcs:
            print("No diagnostic trouble codes found.")
    
    def cmd_clear_dtc(self, args):
        """Clear diagnostic trouble codes"""
        if not self._check_connection():
            return
        
        confirm = input("Are you sure you want to clear all DTCs? (yes/no): ").strip().lower()
        if confirm in ['yes', 'y']:
            if self.scanner.clear_dtcs():
                print("✓ DTCs cleared successfully")
            else:
                print("✗ Failed to clear DTCs")
        else:
            print("Operation cancelled")
    
    def cmd_live_data(self, args):
        """Start live data streaming"""
        if not self._check_connection():
            return
        
        # Parse arguments
        enable_graph = False
        pids = []
        
        for arg in args:
            if arg.lower() in ['--graph', '-g']:
                if MATPLOTLIB_AVAILABLE:
                    enable_graph = True
                else:
                    print("Warning: matplotlib not available, graphing disabled")
            else:
                pids.append(arg.upper())
        
        if not pids:
            # Default PIDs for live data
            pids = ["010C", "010D", "0105", "0104"]  # RPM, Speed, Coolant, Load
        
        graph_info = " with graphing" if enable_graph else ""
        print(f"Starting live data stream{graph_info} for PIDs: {', '.join(pids)}")
        
        if enable_graph:
            print("Graph window will open. Close it or press Ctrl+C to stop streaming.")
        else:
            print("Press Ctrl+C to stop streaming...")
        
        try:
            self.scanner.start_live_data_stream(pids, interval=0.5, enable_graph=enable_graph)
            
            # Keep running until interrupted
            while self.scanner.streaming:
                time.sleep(1)
                
        except KeyboardInterrupt:
            self.scanner.stop_live_data_stream()
            print("\nLive data streaming stopped")
    
    def cmd_stop_live(self, args):
        """Stop live data streaming"""
        if self.scanner:
            self.scanner.stop_live_data_stream()
            print("Live data streaming stopped")
        else:
            print("No active scanner connection")
    
    def cmd_read_pid(self, args):
        """Read specific PID"""
        if not self._check_connection():
            return
        
        if not args:
            print("Usage: pid <PID> (e.g., pid 010C)")
            return
        
        pid = args[0].upper()
        print(f"Reading PID {pid}...")
        
        reading = self.scanner.read_pid(pid)
        if reading:
            print(f"  {reading.name}: {reading.value} {reading.unit}")
            print(f"  Raw data: {reading.raw_data}")
        else:
            print(f"Failed to read PID {pid} or PID not supported")
    
    def cmd_vin(self, args):
        """Read VIN"""
        if not self._check_connection():
            return
        
        print("Reading Vehicle Identification Number...")
        vin = self.scanner.get_vin()
        
        if vin:
            print(f"VIN: {vin}")
        else:
            print("Failed to read VIN")
    
    def cmd_monitors(self, args):
        """Check readiness monitors"""
        if not self._check_connection():
            return
        
        print("Checking readiness monitors...")
        monitors = self.scanner.get_readiness_monitors()
        
        print("\nReadiness Monitor Status:")
        print("-" * 40)
        
        for monitor, status in monitors.items():
            status_symbol = "✓" if status in ["Ready", "OFF"] else "⚠" if status == "Not Ready" else "●"
            print(f"  {status_symbol} {monitor:<25}: {status}")
    
    def cmd_freeze_frame(self, args):
        """Read freeze frame data"""
        if not self._check_connection():
            return
        
        dtc_index = int(args[0]) if args else 0
        print(f"Reading freeze frame data for DTC index {dtc_index}...")
        
        freeze_data = self.scanner.get_freeze_frame_data(dtc_index)
        
        if freeze_data:
            print("\nFreeze Frame Data:")
            print("-" * 40)
            
            for key, value in freeze_data.items():
                if isinstance(value, dict) and "name" in value:
                    print(f"  {value['name']}: {value['value']} {value['unit']}")
                elif key not in ["timestamp", "raw_data"]:
                    print(f"  {key}: {value}")
        else:
            print("No freeze frame data available")
    
    def cmd_protocol(self, args):
        """Set or show protocol"""
        if not self._check_connection():
            return
        
        if not args:
            # Show current protocol
            protocol = self.scanner.get_protocol_info()
            print(f"Current protocol: {protocol}")
            
            print("\nAvailable protocols:")
            for proto in OBDProtocol:
                print(f"  {proto.value}: {proto.name.replace('_', ' ')}")
        else:
            # Set protocol
            try:
                protocol_num = args[0].upper()
                
                # Find matching protocol
                protocol = None
                for proto in OBDProtocol:
                    if proto.value == protocol_num:
                        protocol = proto
                        break
                
                if protocol:
                    if self.scanner.set_protocol(protocol):
                        print(f"✓ Protocol set to {protocol.name.replace('_', ' ')}")
                    else:
                        print("✗ Failed to set protocol")
                else:
                    print(f"Invalid protocol: {protocol_num}")
                    
            except Exception as e:
                print(f"Error setting protocol: {e}")
    
    def cmd_at_command(self, args):
        """Send raw AT command"""
        if not self._check_connection():
            return
        
        if not args:
            print("Usage: at <command> (e.g., at rv)")
            return
        
        command = " ".join(args)
        print(f"Sending AT command: {command}")
        
        response = self.scanner.send_at_command(command)
        if response:
            print(f"Response: {response}")
        else:
            print("No response or command failed")
    
    def cmd_o2_test(self, args):
        """Test oxygen sensors"""
        if not self._check_connection():
            return
        
        print("Testing oxygen sensors...")
        results = self.scanner.perform_o2_sensor_test()
        
        if results:
            print("\nOxygen Sensor Test Results:")
            print("-" * 50)
            
            for sensor, data in results.items():
                if isinstance(data, dict) and "value" in data:
                    status_symbol = "✓" if data["status"] == "OK" else "⚠"
                    print(f"  {status_symbol} {sensor}: {data['value']:.3f} {data['unit']} ({data['status']})")
                else:
                    print(f"  {sensor}: {data}")
        else:
            print("No oxygen sensor data available")
    
    def cmd_actuator_test(self, args):
        """Test actuators"""
        if not self._check_connection():
            return
        
        print("Testing actuators...")
        results = self.scanner.test_actuators()
        
        if results:
            print("\nActuator Test Results:")
            print("-" * 30)
            
            for test, result in results.items():
                print(f"  {test}: {result}")
        else:
            print("No actuator test data available")
    
    def cmd_battery(self, args):
        """Monitor battery voltage"""
        if not self._check_connection():
            return
        
        voltage = self.scanner.monitor_battery_voltage()
        
        if voltage > 0:
            status = "Good" if voltage >= 12.4 else "Low" if voltage >= 11.8 else "Critical"
            status_symbol = "✓" if status == "Good" else "⚠" if status == "Low" else "✗"
            
            print(f"Battery Voltage: {voltage:.1f}V ({status}) {status_symbol}")
            
            if voltage < 12.0:
                print("Warning: Low battery voltage detected")
        else:
            print("Failed to read battery voltage")
    
    def cmd_performance(self, args):
        """Run performance test"""
        if not self._check_connection():
            return
        
        print("Starting 0-100 km/h acceleration test...")
        print("Make sure vehicle is in a safe location for testing!")
        
        confirm = input("Continue with performance test? (yes/no): ").strip().lower()
        if confirm not in ['yes', 'y']:
            print("Test cancelled")
            return
        
        results = self.scanner.performance_test_0_to_100()
        
        print("\nPerformance Test Results:")
        print("-" * 40)
        
        if results.get("duration"):
            print(f"  0-100 km/h time: {results['duration']:.2f} seconds")
            print(f"  Maximum speed: {results['max_speed']:.1f} km/h")
            print(f"  Data points: {len(results['readings'])}")
        else:
            print("  Test incomplete or failed")
            if "error" in results:
                print(f"  Error: {results['error']}")
    
    def cmd_logging(self, args):
        """Control data logging"""
        if not self._check_connection():
            return
        
        if not args:
            print("Usage: log <start|stop> [filename]")
            return
        
        action = args[0].lower()
        
        if action == "start":
            filename = args[1] if len(args) > 1 else None
            self.scanner.start_logging(filename)
            print(f"✓ Data logging started to {self.scanner.log_file}")
            
        elif action == "stop":
            if self.scanner.log_file:
                self.scanner.stop_logging()
                print("✓ Data logging stopped")
            else:
                print("No active logging session")
        else:
            print("Invalid action. Use 'start' or 'stop'")
    
    def cmd_fuel_economy(self, args):
        """Calculate fuel economy"""
        if len(args) != 2:
            print("Usage: fuel <distance_km> <fuel_liters>")
            print("Example: fuel 100 8.5")
            return
        
        try:
            distance = float(args[0])
            fuel = float(args[1])
            
            results = self.scanner.calculate_fuel_economy(distance, fuel) if self.scanner else {}
            
            print(f"\nFuel Economy Calculation:")
            print(f"Distance: {distance} km")
            print(f"Fuel used: {fuel} L")
            print("-" * 30)
            
            if "l_per_100km" in results:
                print(f"Consumption: {results['l_per_100km']:.2f} L/100km")
            
            if "mpg" in results:
                print(f"Economy: {results['mpg']:.1f} MPG")
            
            if "km_per_liter" in results:
                print(f"Economy: {results['km_per_liter']:.1f} km/L")
                
        except ValueError:
            print("Invalid numbers provided")
    
    def cmd_exit(self, args):
        """Exit the program"""
        if self.scanner:
            self.scanner.disconnect()
        
        print("Goodbye!")
        self.running = False
    
    def _check_connection(self) -> bool:
        """Check if connected to scanner"""
        if not self.scanner or not self.scanner.connected:
            print("Not connected to ELM327. Use 'connect' command first.")
            return False
        return True
    
    def print_PIDs(self, args):
        pids = PIDDatabase.PIDS.keys()

        for pid in pids:
            print(f"# {pid} -> {PIDDatabase.PIDS[pid]['name']}")


def main():
    """Main entry point"""
    try:
        interface = OBDCommandInterface()
        interface.start()
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Fatal error: {e}")
        logging.error(f"Fatal error: {e}", exc_info=True)

if __name__ == "__main__":
    main()