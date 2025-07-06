# OpenSteamDeckController (v0.1)

A custom modular wireless controller inspired by the Steam Deck and Steam Controller. Features trackpads, detachable halves, and full HID support. Built from scratch using nRF52840 microcontrollers and the Enhanced ShockBurst protocol. [Video](https://youtu.be/ycMgIToLav8?si=cVztny4IiIS_V4oI) 

Join the [Discord!](https://discord.gg/ZtV34Hwx7y)

> **Note**  
> This project is in an early development stage — v0.1. Expect breaking changes, bugs, and frequent updates.

---

## 🚀 Project Goals

- 🎮 Emulate a Steam Deck-like controller with modular, wireless halves
- 🖱️ Support mouse, keyboard, and gamepad HID modes
- 🧲 Custom-built trackpads using the Azoteq IQS7211E sensor
- 🔋 Magnetic pogo-pin charging and attachment system
- 🛠️ Fully open-source hardware and firmware

---

## ✅ v0.1 Development Checklist

### Hardware
- [x] Left and right controller halves (nRF52840)
- [x] USB dongle (nRF52840)
- [x] Custom trackpad PCBs (IQS7211E)
- [ ] 3D printable version of the Shell
- [ ] Controller PCBs(left + right)
- [ ] Joystick PCBs(to drive down cost)
- [ ] gp2040ce + nrf dongle

### Firmware
- [x] ESB-based wireless communication
- [x] Mouse + Gamepad + Keyboard HID reports
- [x] Input parsing for trackpads and buttons
- [ ] Haptics / rumble support
- [ ] Gyro Support
- [ ] Calibration routines + save to internal memory
- [ ] Configurable modes (gesture/tap/mouse/gamepad)
- [ ] Steam Input compatibility or XInput compatibility
- [ ] Capacitive Joystick

### Other
- [ ] Improve documentation and diagrams
- [ ] Design a UI or config utility(either software or adding screens)
- [ ] Firmware migration to Zephyr (optional future)
- [ ] Optimize latency and packet loss handling
- [ ] Reduces costs

---

## 📷 Media

Coming soon — demo images, build progress, and gameplay tests.

---

## 🛠️ Build Instructions

_(Coming soon)_

- Schematic and board files (.zip or link)
- Firmware flashing instructions
- Dependency list and setup

---

## 🤝 Contributing

Contributions are welcome! Feel free to fork the repo, open issues, or submit pull requests.

If you're not sure where to start, check out the [Checklist](#-v01-development-checklist) for open tasks.

---

## 📄 License

MIT License — see `LICENSE` file for details.

---

## 💬 Contact

For questions or suggestions, reach out via GitHub Issues or [YouTube](https://www.youtube.com/@TommyBee456).
