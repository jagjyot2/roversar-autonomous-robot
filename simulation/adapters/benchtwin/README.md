# BenchTwin Preserved Adapter (`benchtwin/`)

This adapter manifest preserves the **BenchTwin** hardware-in-the-loop staging package located in `benchtwin/` while **Wokwi** serves as the active microcontroller/hardware simulator.

## Preserved Files (Untouched)
- `benchtwin/roversar.btw`
- `benchtwin/roversar-benchtwin.json`
- `benchtwin/roversar.net`
- `benchtwin/BOM.csv`
- `benchtwin/roversar_base_plate.stl`
- `benchtwin/BENCHTWIN-SIMULATION-NOTES.md`

## Future Reactivation
1. Open `benchtwin/roversar.btw` in BenchTwin.
2. Confirm `roversar-benchtwin.json` matches `firmware/include/roversar_config.h` pin assignments.
3. Implement a `BenchTwinRoverAdapter` conforming to `simulation/adapters/common/rover_hal_interface.py` (`RoverHalProtocol`).
4. Switch the active simulator profile from `wokwi` to `benchtwin` without modifying firmware logic.
