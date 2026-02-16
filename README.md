# WARP cUF System

WARP control software for the Industrial Shields Raspberry PLC platform.

This repository keeps the legacy operator workflow from `device/Old_Codes/MainGUI_v5.py` and provides it as:
- `device`: FastAPI backend for hardware control and sequence execution
- `ui/warp-console`: Next.js operator console with the same MainGUI_v5 visual style (dark theme, valve grid, controls, sequences, event log)

## Project Structure

- `device/src`: backend domain, hardware drivers, and API
- `device/config`: device YAML configuration files
- `ui/warp-console/app`: Next.js app entrypoint and global styles
- `ui/warp-console/components/legacy-console.tsx`: rebuilt MainGUI_v5-style UI
- `device/Old_Codes`: archived legacy scripts and references

## Quick Start

### 1. Backend

```bash
cd device
python -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python -m src.main --config config/device1.yaml
```

Backend default endpoint:
- `http://<device-host>:8001`

### 2. UI

```bash
cd ui/warp-console
npm install
npm run dev
```

UI default endpoint:
- `http://<device-host>:3001`

The UI auto-targets backend port `8001` on the same host unless `NEXT_PUBLIC_API_BASE` is set.

## MainGUI_v5 Compatibility

The rebuilt UI preserves the MainGUI_v5 operator layout and controls:
- Valve buttons (V1, V2, V3, V8, V9, V10, Collect Elution)
- Process metrics panel (flow, hall, pressure slots)
- Pump controls (enable, direction, speed)
- PID controls (enable, setpoint, home)
- Sequence panel (Full Sequence, Deaeration, Concentration, Elution, Clean 1, Clean 2)
- Reset Metrics, STOP, and Event Log

Legacy sequence buttons are mapped to available backend sequence handlers.

## Recovery After Reboot (Operator Checklist)

1. Open terminal/SSH on the PLC.
2. Start backend service or run backend command from `device`.
3. Start UI service or run `npm run start` / `npm run dev` in `ui/warp-console`.
4. Open the UI in browser and verify status/log updates.

## Notes

- `device/Old_Codes` remains as an archive/reference area.
- Hardware-specific behavior still depends on the configured PLC pins and connected devices.
