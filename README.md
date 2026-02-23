# MecaChess Robotic Arm - Raspberry Pi Pico Edition

A complete source code repository for a voice-controlled robotic arm system that plays chess using Raspberry Pi Pico microcontroller. This project combines kinematics, computer vision, speech recognition, and AI to create an autonomous chess-playing robotic system.

## Project Overview

**MecaChess** is an advanced robotic arm system designed to:
- Recognize and play chess moves via voice commands (Spanish language)
- Calculate forward and inverse kinematics for precise arm positioning
- Detect chess pieces using computer vision and OpenCV
- Execute moves autonomously through Pico PWM servo control
- Integrate DeepSeek AI for intelligent chess decision-making

## System Architecture

### Hardware
- **Microcontroller:** Raspberry Pi Pico (RP2040)
- **Serial Communication:** COM15 at 115200 baud
- **Servo Control:** 6 PWM servos (GPIO 8-12) for 6-DOF manipulator
- **Power Supply:** USB Powered for development
- **Vision Input:** IP camera via Iriun app (http://10.81.34.95:8080)

### Arm Dimensions
- l1 (Base to Joint 1): 96mm
- l2 (Joint 1 to Joint 2): 13.9mm
- l3 (Joint 2 to Joint 3): 150mm
- l4 (Joint 3 to Joint 4): 143.24mm
- l5 (Joint 4 to End-Effector): 127.42mm

## Files Included

### 1. **Voice_Controlled_Robotic_Arm_Pico.ipynb**
Speech recognition pipeline for hands-free chess move input
- Uses Google Speech Recognition (Spanish: es-ES)
- Keyboard controls: 'H' to start recording, 'Q' to exit
- Coordinate correction dictionary for phonetic errors
- Serial communication with Pico via pyserial
- **Key Functions:**
  - `recognize_coordinates()` - Speech-to-text processing
  - `send_command_to_pico()` - Serial move transmission

### 2. **Forward_Inverse_Kinematics_GUI_Geometric_Pico.ipynb**
PyQt5 graphical interface for geometric kinematics calculations
- Real-time forward kinematics from joint angles
- Inverse kinematics using geometric methods
- Interactive GUI sliders for joint angle control
- Visual arm visualization
- **Key Classes:**
  - `KinematicsGUI` - PyQt5 interface
  - `forward_kinematics()` - FK computation
  - `inverse_kinematics()` - IK using law of cosines

### 3. **Forward_Inverse_Kinematics_Analytical_Pico.ipynb**
Matrix-based analytical kinematics using Denavit-Hartenberg parameters
- Homogeneous transformation matrices (A0_1, A1_2, A2_3, A3_4)
- SymPy symbolic math for exact calculations
- Numerical methods for inverse kinematics
- **Key Functions:**
  - `forward_kinematics_analytical()` - Matrix-based FK
  - `inverse_kinematics_analytical()` - Analytical IK solution

### 4. **Chess_General_Code_Pico.ipynb**
Computer vision pipeline for chess board detection and piece recognition
- Board corner detection via edge detection
- Piece recognition using HSV color space
- Pixel-to-chess-notation conversion (a1-h8 mapping)
- **Key Functions:**
  - `detect_chess_pieces()` - Piece location identification
  - `detect_board_corners()` - Board boundary detection
  - `map_piece_to_square()` - Coordinate conversion

### 5. **MecaChess_Robotic_Arm_Final_V2_Pico.ipynb**
Complete integrated system combining all subsystems
- Unified system architecture with modular design
- System initialization and shutdown procedures
- Main orchestration controller
- **Key Classes:**
  - `SystemConfig` - Configuration parameters (ports, bounds, arm geometry)
  - `PicoController` - Serial communication handler
  - `Kinematics` - FK/IK computation
  - `ChessBoardDetection` - Vision pipeline
  - `MecaChessController` - Main system orchestrator

### 6. **main.cpp**
Raspberry Pi Pico C firmware for servo control and real-time kinematics
- PWM servo control initialization (50Hz, 500-2400µs range)
- Inverse kinematics computation in embedded C
- Serial UART communication (115200 baud)
- Board coordinate database (64 squares + storage locations)
- Status LED indicators (GPIO 22/26/30)
- **Key Functions:**
  - `Mover_pieza()` - Move piece from source to destination
  - `Comer_pieza()` - Capture opponent's piece
  - `Calculos_cinematica_inversa()` - Real-time IK calculation

## Dependencies

### Python Packages
```
speech_recognition
pyserial
pynput
PyQt5
opencv-python (cv2)
numpy
matplotlib
sympy
google-generativeai (optional)
openai (for DeepSeek API)
python-chess
```

### Installation
```bash
pip install speech_recognition pyserial pynput PyQt5 opencv-python numpy matplotlib sympy openai python-chess
```

### C/Pico Dependencies
- pico-sdk (for compilation)
- ARM GCC Toolchain
- CMake 3.12+

## Usage

### Running Voice Control
Open `Voice_Controlled_Robotic_Arm_Pico.ipynb` in Jupyter and execute:
1. Cell 1: Load libraries
2. Cell 2: Define functions (Presionar_tecla, Reconocer_coordenadas, etc.)
3. Cell 3: Main loop
   - Press 'H' to start listening
   - Speak chess coordinates (e.g., "E4 E5")
   - Press 'Q' to quit

### Running Kinematics GUI
Execute `Forward_Inverse_Kinematics_GUI_Geometric_Pico.ipynb`:
- Use PyQt5 sliders to control joint angles
- Observe real-time end-effector position
- Calculate inverse kinematics from Cartesian coordinates

### Running Vision System
Execute `Chess_General_Code_Pico.ipynb`:
- Load chess board image
- Auto-detect pieces and board corners
- Generate move encoding for Pico

### Compiling Pico Firmware
```bash
mkdir build
cd build
cmake ..
make
```
Upload `.uf2` file to Pico in bootloader mode.

## Configuration

All system parameters are defined in `MecaChess_Robotic_Arm_Final_V2_Pico.ipynb` under `SystemConfig`:

```python
class SystemConfig:
    PICO_PORT = 'COM15'           # Serial port
    BAUD_RATE = 115200            # Communication speed
    ARM_L1, ARM_L2, ARM_L3 = ...   # Segment lengths
```

## Communication Protocol

### Serial Format (Python → Pico)
```
<source_square> <dest_square> <capture_flag>\n
Example: E4 E5 0\n
```

### Pico Response
ASCII acknowledgment: '1' for success, '0' for error

## Coordinate System

Chess board mapping (OpenCV image coordinates → chess notation):
- a-h: Columns (left to right)
- 1-8: Rows (bottom to top)
- Storage positions: Beyond board boundary for captured pieces

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Port COM15 not found" | Check Pico USB connection, verify port in Device Manager |
| Speech not recognized | Check microphone, ensure quiet environment, speak clearly in Spanish |
| Servo not moving | Verify PWM pins, check power supply, test with `servo_write()` function |
| Kinematics error | Validate arm segment lengths, check joint angle bounds |

## Future Improvements

- Real-time OpenCV calibration GUI
- Multi-language speech recognition
- Stockfish integration for stronger chess AI
- 3D arm visualization with PyOpenGL
- Bluetooth wireless control

## License

Open source - MecaChess Project

## Authors

**Team:** MecaChess Development Team
**Last Updated:** February 2026

---

For support or questions, refer to individual notebook documentation and inline code comments.
