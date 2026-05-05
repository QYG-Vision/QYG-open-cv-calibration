# AGENTS.md

Companion to `CLAUDE.md` (read that first for full architecture). This file covers the highest-signal gotchas.

## Commands

```bash
cmake -S . -B build && cmake --build build -j
```

Build with tests enabled:

```bash
cmake -S . -B build -DBUILD_TESTING=ON && cmake --build build -j
ctest --test-dir build --output-on-failure
```

No lint target in CMake. Format/tidy manually:

```bash
clang-format -i <file>
clang-tidy -p build <file>
```

## Architecture

Four thin `main` files at repo root (`calibrateCamera.cpp`, `calibrateHandEye.cpp`,
`calculateError.cpp`, `validateHandEye.cpp`). Shared logic lives in `src/` + `include/`
and compiles into the `calibration_core` static library. New shared code goes in `src/`,
not in the `main` files.

## Device abstraction

`qd::Device::Device` (`include/device.hpp`) — pure virtual `read(img, timestamp)`.
Three impls: `Hik_Camera`, `UVC_Camera`, `Image_Reader`. The factory is
`qd::app::create_device()` in `src/device_factory.cpp` — single switch on YAML `device:`.
The factory returns a `DeviceContext` struct with the device ptr and a `wait_time` hint:
`0` for IMG (no delay needed), `1` for live cameras.

When `device:` is `IMG` and all images are consumed, `is_exhausted()` returns `true`.
In `calibrateCamera`, this triggers an automatic `calibrate_camera()` call and exit.

## UI is a Web viewer, not OpenCV HighGUI

`qd::WebViewer` (`include/web_viewer.hpp`) runs an embedded HTTP/MJPEG server on port 8080
and exposes `imshow`/`waitKey`/`namedWindow` methods. User must open `http://localhost:8080`
and click the page to give it focus before keypresses (`s`/`a`/`c`/`r`/`ESC`) register.

**Never** add `cv::imshow` or `cv::waitKey` — use the viewer instance instead.

## Config coupling (critical)

After running `calibrateCamera`, the `camera_matrix` and `distort_coeffs` arrays in the
generated `camera_calibration.yaml` must be copied back into `config/calibration.yaml`
before running hand-eye flows. Hand-eye calibration and reprojection-error calculation
read intrinsics from the config file, not from the camera calibration output.

## CLI args

Uses `cv::CommandLineParser` with hyphenated keys:

```bash
./build/calibrateCamera --config-path=config/calibration.yaml
./build/calibrateHandEye -l -d ./handeye_calib_data   # offline recompute
./build/validateHandEye --handeye-path=handeye_calibration.yaml
```

Default config path is `config/calibration.yaml`.

## HIK cameras

Require a udev rule for non-root access (vendor `2bdf`). Without it, `Hik_Camera` fails
to enumerate. See README §1 for the `sudo tee` command.

## Auto-collector

Ported from ROS `image_pipeline/camera_calibration` calibrator.py. Toggle with `a` at
runtime. Thresholds in `config/calibration.yaml` under `auto_collect_*` keys match ROS
defaults, except `auto_collect_sharpness_threshold` (Laplacian-variance gate) which is
project-specific.
