# ME 333 — Intro to Mechatronics
**Northwestern University · McCormick School of Engineering**

Foundational mechatronics course covering closed-loop control systems, embedded C firmware, and hardware-software integration. Primary project: a fully closed-loop intelligent motor driver controlled via a PC user interface.

---

## Project — Intelligent Motor Driver

### Overview
Built a closed-loop motor control system using a PIC32 microcontroller. The motor responds to real-time keyboard commands from a PC user interface over USB, with live position and current plots streamed back to the host. Supports interactive controller tuning without reflashing firmware.

### System Architecture
```
PC (Python UI)
    │  USB Serial (CDC)
    ▼
PIC32 Microcontroller
    ├── Current Sense (ADC) ──► PI Current Controller
    ├── Quadrature Encoder ──► PI Position Controller
    └── PWM Output ──────────► H-Bridge ──► DC Motor
```

### Features
- **PI Current Controller** — inner loop, regulates motor current
- **PI Position Controller** — outer loop, tracks position setpoint
- **Quadrature encoder** feedback for position/velocity measurement
- **Interactive tuning** — adjust Kp, Ki gains live from the PC
- **Real-time plotting** — position and current history streamed over USB and plotted in MATLAB
- **Keyboard command interface** — hold, track, quit commands

### Controller Parameters
```
Current loop:  Kp = 0.8,  Ki = 0.3,  Kd = 0.007
Position loop: Kp = 0.02, Ki = 0.02, Kd = 0.1
               Jp = 0.08, Ji = 0.03
```

### Results
Validated via MATLAB step-response plots showing stable position tracking and current regulation. Score: 21.4 (position), 8.1 (current).

---

## Stack
`C` `Python` `MATLAB` `PIC32` `USB CDC` `PWM` `ADC`

---

*Northwestern University — ME 333, Winter 2024 · Intro to Mechatronics*
