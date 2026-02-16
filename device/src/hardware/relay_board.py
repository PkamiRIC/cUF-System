from infra.config import RelayConfig
from hardware.plc_utils import plc, ensure_plc_init


class RelayBoard:
    """
    Relay control adapter.

    Primary path (MainGUI_v5-compatible): use PLC relay outputs R1.1..R1.8.
    This backend intentionally follows MainGUI_v5 wiring via PLC R1 outputs.
    """

    def __init__(self, config: RelayConfig) -> None:
        self.config = config
        self._relay_pin_map = {ch: f"R1.{ch}" for ch in range(1, 9)}
        ensure_plc_init()
        if plc:
            for pin in self._relay_pin_map.values():
                try:
                    plc.pin_mode(pin, plc.OUTPUT)
                except Exception:
                    # Leave fallback path available if PLC relay pin setup fails.
                    pass

    def _set_via_plc(self, relay_num: int, enabled: bool):
        if not plc:
            return None
        pin = self._relay_pin_map.get(relay_num)
        if not pin:
            return False
        try:
            level = plc.HIGH if enabled else plc.LOW
            plc.digital_write(pin, level)
            plc.delay(10)
            return True
        except Exception:
            return False

    def on(self, relay_num: int) -> bool:
        if not (1 <= relay_num <= 8):
            raise ValueError("relay_num must be 1..8")
        plc_ok = self._set_via_plc(relay_num, True)
        return bool(plc_ok)

    def off(self, relay_num: int) -> bool:
        if not (1 <= relay_num <= 8):
            raise ValueError("relay_num must be 1..8")
        plc_ok = self._set_via_plc(relay_num, False)
        return bool(plc_ok)

    def all_on(self) -> bool:
        return all(bool(self._set_via_plc(ch, True)) for ch in range(1, 9))

    def all_off(self) -> bool:
        return all(bool(self._set_via_plc(ch, False)) for ch in range(1, 9))
