# Sensor Fusion & Calibration Architecture

## Overview
This project separates sensor calibration and sensor fusion into distinct modules for clarity, maintainability, and reliability. Calibration computes and stores sensor biases, scales, and alignment matrices; sensor fusion applies these corrections and performs data fusion.

---

## Calibration Module (`calibration.c`)
- **Purpose:** Computes and stores all sensor biases, scales, and alignment matrices required for accurate sensor fusion.
- **Key Routines:**
  - `calibrate_gyro_bias`: Determines gyroscope bias (zero-rate offset).
  - `calibrate_accel`: Determines accelerometer bias and scale (per-axis, including gravity alignment).
  - `calibrate_mag`: Determines magnetometer bias (hard-iron) and scale (soft-iron).
  - `cross_calibrate`: Computes cross-sensor alignment (rotation matrices) between accelerometer, gyro, and mag frames.
  - `calibrate_gyro_scale` / `calibrate_gyro_cross_coupling`: (Planned) Determines gyro scale/cross-coupling using turn-table or known-rate sources.
  - `run_full_calibration`: Runs all calibration stages in sequence, interactively or via CLI.
  - `save_calibration`/`load_calibration`: Persists calibration results to disk (e.g., `/etc/sensor_calib.json`).
- **Data Structure:**
  - `calib_params_t` (see `calibration.h`) holds all calibration results: biases, scales, alignment matrices, and advanced parameters.

---

## Calibration Output & Persistence
- **Output Formats:**
  - **C Header:** Optionally generates a C header file with calibration constants for firmware builds.
  - **JSON:** Default output is a JSON file (e.g., `/etc/sensor_calib.json`) for easy parsing and persistence.
- **Persistence:**
  - Calibration results are saved after each routine and can be loaded at startup or on demand.
  - Supports optional persistent storage (e.g., flash, SD card, or filesystem).

---

## Running Calibration
- **CLI Usage:**
  - Run the calibration suite via CLI (planned):
    ```sh
    ./calibration --full
    ./calibration --gyro
    ./calibration --accel
    ./calibration --mag
    ./calibration --cross
    ```
  - Each routine will prompt for required actions (e.g., "Keep sensor still", "Rotate in all directions", "Place on turn-table").
  - Results are saved automatically after each stage.
- **Integration:**
  - Calibration can be launched from the main application with a CLI flag or menu option.

---

## Sensor Fusion Module (`sensor_fusion.c`)
- **Purpose:** Fuses sensor data, applies precomputed biases/scales, manages random walk noise. Does NOT compute or update biases.
- **Key Points:**
  - Loads calibration results at startup using `load_calibration_results()` (calls `load_calibration()` from calibration module).
  - Applies biases/scales and alignment matrices to raw sensor readings before fusion.
  - If calibration file is missing, uses default (zero) biases and logs a warning.

---

## Data Flow
1. **Calibration:**
   - User runs calibration process (full or partial).
   - Results are saved to a file (e.g., `/calibration.json`) and/or C header.
2. **Sensor Fusion:**
   - On startup, loads calibration file.
   - Populates global bias/scale/alignment variables.
   - Applies these corrections to sensor data before fusion.

---

## Example: Loading Calibration in Fusion
```c
if (load_calibration_results("/calibration.json") != 0) {
    fprintf(stderr, "[Fusion] WARNING: Using default (zero) biases and scales!\n");
}
```

---

## Extending/Modifying
- To add new calibration parameters, extend `calib_params_t` in `calibration.h` and update save/load logic in `calibration.c`.
- To add new calibration routines, implement them in `calibration.c` and update the CLI interface.
- Always keep calibration and fusion responsibilities separate.

---

## Future Plans
- **Advanced Routines:**
  - Turn-table support for precise gyro scale and cross-coupling calibration
  - Automated mag/accel alignment with robust outlier rejection
  - Optional persistence to embedded flash or EEPROM
  - Output of calibration results in multiple formats (C, JSON, CSV)
- **Testing:**
  - Add unit tests for calibration routines and persistence
  - Provide example datasets and reference outputs

---

## References
- `src/core/fusion/calibration.c`, `include/core/fusion/calibration.h`
- `src/core/fusion/sensor_fusion.c`
- `include/common/common_defs.h`

---

## Contributing & Contact
- Contributions, bug reports, and feature requests are welcome!
- See code comments for detailed documentation.
- For questions or to contribute, contact the maintainers (see AUTHORS or project repository).
