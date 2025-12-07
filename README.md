Smart Environment Monitoring System (STM32F401RE)
PIR-based Motion Detection | LDR-based Light Sensing | OLED Display | Buzzer Alerts (Bare-Metal)

This project implements a real-time environment monitoring and alert system using the STM32F401RE microcontroller.
The system detects motion, measures ambient light level, shows readings on an SSD1306 OLED display, and activates an audio alert using an active buzzer.

The entire code is written in pure bare-metal C (no HAL, no CMSIS drivers) — including I2C driver, ADC configuration, SysTick timer, TIM2 timer, and GPIO control.

📌 Features

✔ PIR Motion Detection (PA0)
✔ Ambient Light Measurement using LDR + ADC (PA1)
✔ OLED Display Output (SSD1306, I²C on PB8/PB9)
✔ Automatic LED Lighting (PA5) when motion + darkness
✔ Active Buzzer Alert – 3 short beeps every time NEW motion is detected (PA6)
✔ Non-blocking state machine for buzzer
✔ Software debounced motion detection
✔ Thoroughly optimized bare-metal code – no HAL
✔ OLED text rendering with custom 6×8 font
✔ Random-pixel-free page-by-page rendering

📷 Demonstration Output (OLED)
LDR:1980
BRIGHT
NO MOTION


OR when dark + motion:

LDR:450
DARK
MOTION


(You may include actual photos once system is tested.)

📡 Hardware Requirements
Components

STM32F401RE (Nucleo board or bare MCU)

PIR Sensor (HC-SR501)

LDR (2-pin photoresistor)

10kΩ fixed resistor (for LDR voltage divider)

SSD1306 128×64 OLED display (I2C)

Active Buzzer (3.3V compatible)

LED + 220Ω resistor

Jumper wires

Breadboard / PCB

🪛 Pin Connections
PIR Sensor
PIR Pin	STM32 Pin
OUT	PA0
VCC	5V
GND	GND
LDR Sensor (Voltage Divider)
3.3V ---- 10kΩ ----(PA1 ADC)---- LDR ---- GND

OLED Display (SSD1306 I2C)
OLED Pin	STM32 Pin
VCC	3.3V
GND	GND
SCL	PB8
SDA	PB9

(Ensure your display uses I2C address 0x3C; if 0x3D, edit the code.)

LED
Component	STM32 Pin
LED (+)	PA5
LED (–)	GND (via 220Ω resistor)
Buzzer
Buzzer Pin	STM32 Pin
+ (VCC)	PA6
– (GND)	GND
⚙️ Functional Overview
1. Motion + Dark → LED ON + Buzzer Beeps

When LDR < 2000 (dark) AND PIR detects motion:

LED turns ON

Buzzer beeps 3 fast beeps (100ms ON / 100ms OFF)

2. LED Auto-Turn-OFF Timer

If no new motion is detected for 3 seconds, LED turns OFF.

3. OLED UI

Every 150ms the display updates:

Line 1: LDR ADC value

Line 2: DARK / BRIGHT

Line 3: MOTION / NO MOTION

4. Non-blocking Buzzer Handling

Buzzer runs using TIM2 interrupt timing —
does NOT block PIR or OLED operations.

🧠 Software Architecture

SysTick (1ms)
Global time base for delays & LED timer

TIM2 (1ms interrupt)
Drives buzzer timing state machine

ADC1 (PA1)
Reads ambient light

I2C1 (PB8/PB9)
Fully manual bare-metal driver
Includes:

Start condition

Address send

Timeout protection

Page burst writes

Anti-ghost spacing byte

OLED Rendering Module

Minimal 6×8 ASCII font

Safe clean drawing

No leftover random pixels

Page-by-page clear

📁 Project Structure
/Core
  /Src
    main.c          <-- Entire logic + drivers
  /Inc
    stm32f4xx.h
README.md

🚀 Running the Project

Flash using ST-Link from STM32CubeIDE.

Ensure OLED shows text within 1 second.

Wave hand in front of PIR →
LED should turn ON and buzzer should beep.

Until the environment becomes bright or no motion occurs for 3s,
LED remains ON.

🛠️ Customization

You may tune thresholds:

const uint16_t DARK_THRESHOLD = 2000;
const uint32_t HOLD_TIME = 3000; // ms


You may also invert OLED orientation by changing:

OLED_Command(0xA1); // segment remap
OLED_Command(0xC8); // COM scan direction
