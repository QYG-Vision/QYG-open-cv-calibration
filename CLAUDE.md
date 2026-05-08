# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

C++17, CMake 3.14+, depends on `libopencv-dev libyaml-cpp-dev libfmt-dev libeigen3-dev` and the bundled HIK SDK (`hikSDK/lib/{amd64,arm64}` selected by `CMAKE_SYSTEM_PROCESSOR`).

```bash
cmake -S . -B build
cmake --build build -j
```

Tests are wired into CMake via CTest + GoogleTest (FetchContent). Enable with `-DBUILD_TESTING=ON`:
```bash
cmake -S . -B build -DBUILD_TESTING=ON && cmake --build build -j
ctest --test-dir build --output-on-failure
```
Tests live in `tests/unit/` (library tests) and `tests/integration/` (CLI black-box). Test data under `tests/data/` is kept separate from production assets.

Run a single test binary directly:
```bash
./build/test_intrinsic_calibrator
# or with gtest_filter:
./build/test_intrinsic_calibrator --gtest_filter=IntrinsicCalibrator.*
```

`.clang-format` and `.clang-tidy` exist at the repo root — invoke them directly (`clang-format -i <file>`, `clang-tidy -p build <file>`). `compile_commands.json` is exported for clangd.

Public APIs in `include/` carry Doxygen comments: prefer `///` on declarations in headers, `/** ... */` for file-level blocks and larger entry points in `.cpp` files. Start with `@brief`, add `@details` only when behavior or constraints are not obvious. Use `@param`, `@param[out]`, and `@return`.

All executables live in `build/`. The CLI entry points accept `--config-path=<yaml>` (default `config/calibration.yaml`):

- `calibrateCamera` — produces `camera_calibration.yaml` at repo root.
- `calibrateHandEye` — produces `handeye_calibration.yaml`. Use `-l -d <dir>` to skip live capture and recompute from a previously saved `image_*.jpg` + `pose_*.yaml` directory.
- `calculateError` — live RMSE display, no output files.
- `validateHandEye` — `--handeye-path=<yaml>` to override the result file under test.
- `previewCamera` — live preview only; rejects `device: IMG`.
- `calibrationWorkbench` — browser-driven task UI on `http://localhost:8080`, with config editing, task switching, recalibration backups, and intrinsic write-back confirmation.

## Architecture

Each executable is a thin `main` (`apps/calibrateCamera.cpp`, `apps/calibrateHandEye.cpp`, `apps/calculateError.cpp`, `apps/validateHandEye.cpp`, `apps/previewCamera.cpp`, `apps/calibrationWorkbench.cpp`). All shared logic lives under `src/` + `include/` and is built once into the `calibration_core` static library, which every executable links. When adding shared functionality, put it in `src/` so all binaries get it for free; only put loop/CLI glue in `apps/`.

**Device abstraction.** `qd::Device::Device` (`include/device.hpp`) is a pure virtual `read(img, timestamp)` interface. Three implementations: `Hik_Camera` (HIK SDK + background thread + `ThreadSafeQueue`), `UVC_Camera` (`cv::VideoCapture`), `Image_Reader` (offline directory; the only one that ever returns `is_exhausted() == true`). `qd::app::create_device(config_path)` (`src/device_factory.cpp`, declared in `include/device_factory.hpp`) is the single switch on the YAML `device:` field — add new sources here. The factory returns a `DeviceContext` struct containing the device ptr and a `wait_time` hint (0 for `IMG`, 1 for live cameras) used as the `WebViewer::waitKey` delay. When `device:` is `IMG` and all images are consumed, `calibrateCamera` auto-triggers `calibrate_camera()` and exits.

**UI is a Web viewer, not OpenCV HighGUI.** `qd::WebViewer` (`include/web_viewer.hpp`) runs an embedded HTTP/MJPEG server on port 8080 and exposes a `cv::imshow`/`cv::waitKey`/`cv::namedWindow`-shaped API. The user opens `http://localhost:8080` and must click the page so it has focus before keys (`s`/`a`/`c`/`r`/`ESC`) are captured. Do **not** add `cv::imshow` or `cv::waitKey` calls — keep using the viewer instance the executables already construct.

**Calibration core** is now a facade + three delegate classes. `qd::calibrate::Calibrate` (`include/calibrate.hpp` / `src/calibrate.cpp`) is a thin public API that delegates to:

- `IntrinsicCalibrator` (`include/intrinsic_calibrator.hpp` / `src/intrinsic_calibrator.cpp`) — chessboard/circle-grid detection, auto-collector integration, `cv::calibrateCamera`, output save.
- `ExtrinsicCalibrator` (`include/extrinsic_calibrator.hpp` / `src/extrinsic_calibrator.cpp`) — PnP solve per frame, `cv::calibrateRobotWorldHandEye`, offline folder loading, gimbal2camera YAML save.
- `CalibrationValidation` (`include/calibration_validation.hpp` / `src/calibration_validation.cpp`) — reprojection-error overlay (`display_error`), handeye YAML loading (supports old `R_camera2gimbal` and new `gimbal2camera` formats), online position-consistency validation.

`Paramer` (in `calibrate.hpp`) parses board geometry from YAML. Shared free functions for corner finding, board-orientation drawing, sharpness computation, and frame analysis also live in `calibrate.hpp`.

**Auto-collection (`include/auto_collector.hpp` / `src/auto_collector.cpp`)** is a direct port of ROS `image_pipeline/camera_calibration`'s `calibrator.py` (BSD-3, attribution preserved in headers). It deduplicates samples in a 4-D normalized parameter space (X/Y/Size/Skew), tracks per-axis coverage progress, and draws the ROS-style progress bars on the live image. The thresholds (`auto_collect_param_distance`, `auto_collect_param_ranges`, `auto_collect_goodenough_samples`, `auto_collect_max_chessboard_speed`) match ROS defaults; `auto_collect_sharpness_threshold` is a project-specific Laplacian-variance gate that is **not** in the ROS original. `auto_collect_interval_ms` is an additional project-level rate limiter. Toggle at runtime with `a`.

**Serial / IMU.** `Serial_driver` (`include/serial_driver.hpp`) reads quaternion data from a UART (`UartTransporter`) on a daemon thread and serves time-aligned poses via linear interpolation against the camera frame timestamp. Used by `calibrateHandEye` and `validateHandEye`; not used by camera-only flows. Default port `/dev/rm_usb0` (configurable under `Serial:` in YAML).

**Workbench stack.** `calibrationWorkbench` is not just another viewer loop. `TaskController` (`include/task_controller.hpp` / `src/task_controller.cpp`) owns the task state machine, serial lifecycle, intrinsic review/accept flow, and recalibration backups. `ConfigManager` (`include/config_manager.hpp` / `src/config_manager.cpp`) is the JSON/YAML bridge for the embedded SPA, `BackupManager` (`include/backup_manager.hpp` / `src/backup_manager.cpp`) copies old outputs into `backups/<task>/<timestamp>/`, and `WORKBENCH_PAGE` lives as an embedded HTML/JS string in `include/workbench_page.hpp`.

**Workbench wiring is duplicated in three places.** When you add a new task type or frontend action, update `TaskType` / `TaskAction`, the string decoders in `apps/calibrationWorkbench.cpp`, and the `taskDefs` / action handling inside `include/workbench_page.hpp`. When you add an editable config field, keep `ConfigManager::to_json()`, `ConfigManager::update_from_json()`, and the page form fields in sync.

## Configuration

`config/calibration.yaml` is the single source of runtime configuration for all executables. Important coupling to be aware of:

- The `camera_matrix` / `distort_coeffs` arrays in this file are inputs to hand-eye calibration and reprojection error. In the classic CLI flow, after running `calibrateCamera`, you still need to copy values from `camera_calibration.yaml` back into `config/calibration.yaml` before running hand-eye flows. In `calibrationWorkbench`, that same write-back happens only after the user accepts the reviewed intrinsic result.
- `device:` selects which `*_Camera` implementation gets constructed; the corresponding sub-section (`HIK:` / `UVC:` / `IMG:`) is read by that implementation's constructor.

## Platform setup (HIK)

HIK USB cameras need a udev rule for non-root access — see README §1. Without it, `Hik_Camera` will fail to enumerate the device.
