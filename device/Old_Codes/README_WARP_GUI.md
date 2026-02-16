# ✅ WARP GUI Recovery Guide (Industrial Shields RPi)

## Overview

This guide explains how to relaunch the WARP control GUI after the Raspberry Pi is powered off and restarted. It assumes:

- You are using the Industrial Shields Raspberry Pi image  
- You have a working virtual environment (`venv`) with system packages
- All sequences and safety features are committed and pushed to Git

---

## 🔁 On Every Reboot — Follow These Steps

### 1. Open a terminal or connect via SSH/VNC

---

### 2. Navigate to the WARP project folder

```bash
cd ~/WARP
```

---

### 3. Activate the Python environment

```bash
source venv/bin/activate
```

> 💡 This environment was created with `--system-site-packages` so it already includes system-installed `PyQt5`, `RPi.GPIO`, and `industrialshields`.

---

### 4. (Only If Needed) Reinstall Industrial Shields platform

> Do this **only once** unless the system was reset or re-imaged.

```bash
wget https://apps.industrialshields.com/main/rpi/rpiplc_v6/install.sh
chmod +x install.sh
sudo ./install.sh
sudo reboot
```

---

### 5. Run the WARP GUI

```bash
python3 pump_valves_pressure_flow_pid_gui.py
```

---

## 💡 Optional Commands

### 🔹 Update dependencies

```bash
pip install -r requirements.txt
```

---

### 🔹 Reset and rebuild the virtual environment (if corrupted)

```bash
rm -rf venv
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

---

### 🔹 Git push or pull

```bash
git status
git pull   # to get updates
git add .
git commit -m "Your message"
git push
```
