# OpenSplitDeck (v0.3)

A custom modular wireless controller inspired by the Steam Deck and Steam Controller. Features trackpads, detachable halves, and full HID support. Built from scratch using nRF52840 microcontrollers and the Enhanced ShockBurst protocol. [Video](https://youtu.be/ycMgIToLav8?si=cVztny4IiIS_V4oI) 

Join the [Discord!](https://discord.gg/ZtV34Hwx7y)

> **Note**  
> This project is in an early development stage — v0.3. Expect breaking changes, bugs, and frequent updates.

---

## 🚀 Project Goals

- 🎮 Emulate a Steam Deck-like controller with modular, wireless halves
- 🖱️ Support mouse, keyboard, and gamepad HID modes
- 🧲 Custom-built trackpads using the Azoteq IQS7211E sensor
- 🔋 Magnetic pogo-pin charging and attachment system
- 🛠️ Fully open-source hardware and firmware

---

## 😎 Current Status

- 🎮 Emulates a DS4 Controller - This lets us use steam input for the trackpads. Ideally I want it to show up as a steamdeck controller but that is seeming like a much harder task than I thought.
- 🖱️ Support mouse, keyboard, and gamepad HID modes. - Kinda. Mouse and keyboard are properly setup in the HID desccriptors, I just have not implpemented them yet.
- 🧲 Custom-built trackpads using the Azoteq IQS7211E sensor - Yes.
- 🔋 Magnetic pogo-pin charging and attachment system. - Yes but they only support charging. I would like to find a way to make the 2 halves of the controller communicate with each other.
- 🛠️ Fully open-source hardware and firmware - 😎


Controller is still definetly in the early phases. However it does function and work as expected. There is still a lot to improve, optimize and other features to add. Stay tuned for the progression of this project and feel free to build one yourself! Let me know if my instructions are bad.

---

## ✅ v0.3 Development Checklist

### Hardware
- [x] Left and right controller halves (nRF52840)
- [x] USB dongle (nRF52840)
- [x] Custom trackpad PCBs (IQS7211E)
- [ ] 3D printable version of the Shell
- [x] Controller PCBs(left + right)
- [ ] Joystick PCBs(to drive down cost)
- [ ] gp2040ce + nrf dongle

### Firmware
- [x] ESB-based wireless communication
- [x] Mouse + Gamepad + Keyboard HID reports
- [x] Input parsing for trackpads and buttons
- [x] Haptics
- [X] Rumble support (Could still be improved)
- [x] Gyro Support
- [x] Calibration routines + save to internal memory
- [ ] Configurable modes (gesture/tap/mouse/gamepad)
- [x] Steam Input compatibility or XInput compatibility(This is now done through DS4)
- [ ] Capacitive Joystick

### 3d Modeling
- [ ] 3d Printable Case(Started, its in the STEP file)
- [ ] 3d Printable Shoulder Button Structure(Experimental Version in STEP file)
- [ ] 3d Printable Trigger Insert
- [ ] Trigger Locks

### Other
- [ ] Improve documentation and diagrams
- [x] Add Screens
- [ ] Design a UI or config utility(either software or adding screens)
- [X] Firmware migration to Zephyr
- [X] Optimize latency and packet loss handling (could still be better)
- [ ] Reduces costs

---

## 📷 Media

[Valve Took Too Long… So I Built the Steam Controller 2](https://youtu.be/ycMgIToLav8?si=yKXbfcrKJxGGkxHx)

[Valve Announced a New Steam Controller… and I Upgraded Mine](https://youtu.be/R8YwpwIV1gQ?si=VNfhFrsPYMe3HJeA)

[LTT Video](https://youtu.be/eNb55ZwnCRc?si=DEBvHWtwEqF-e4JU)

---

## 🛠️ Build Instructions
- Schematic and board files:

  [LeftPCB](https://oshwlab.com/fbrains69/opensteamdeck-buttonpcb)

  [RightPCB](https://oshwlab.com/fbrains69/opensteamdeck-rightpcb)

  [Trackpad](https://oshwlab.com/fbrains69/steamdeck-controller-trackpad-v1-2)
  
- BOM and Tools:

  [Current BOM - Living Sheet (Im Trying to find the best way to source everything still)](https://docs.google.com/spreadsheets/d/1LNsZTH84R0wy0S3vBu-m3nQxl8zSZT1IippI82DKSO4/edit?usp=sharing)

- Build Guide:

  [Video Build Guide](https://youtu.be/64-y5n9kIJc)

  [Written Build Guide](https://github.com/tommybee456/OpenSplitDeck/blob/main/Docs/OpenSplitDeck_Build_Guide.pdf)

  

---

## 🤝 Contributing

Contributions are welcome! Feel free to fork the repo, open issues, or submit pull requests.

If you're not sure where to start, check out the [Checklist](#-v03-development-checklist) for open tasks.

Special Thanks to:
- TheTitaniumTitan for Suggesting S-Input and providing working example code
- RogueRen for helping make some PCB ordering documentation
- Kuekenzange for making a charging dock 3d Model

---

## 📄 License

MIT License — see `LICENSE` file for details.

---

## 💬 Contact

For questions or suggestions, reach out via GitHub Issues or [YouTube](https://www.youtube.com/@TommyBee456).
