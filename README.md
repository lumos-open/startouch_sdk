# StarTouch SDK

StarTouch SDK provides the Python interface and prebuilt C++ runtime libraries
for controlling StarTouch/FastTouch robotic arms.

Current SDK version: `0.1.5`.

Version note: `2026-05-22`, author `Charlie`.

## Documentation

- [Installation and Runtime Guide](README_INSTALL.md)
  - Supported Ubuntu versions
  - System and Python dependencies
  - Build/install commands
  - CAN setup
  - Runtime configuration, including IK fallback
- [Python API Reference](README_API.md)
  - Arm initialization
  - Joint, pose, gripper, and motion APIs
  - IK behavior and fallback notes
  - Example usage

Start with `README_INSTALL.md` for environment setup, then use
`README_API.md` while writing control scripts.

## Quick Version Check

```bash
python -c "import startouch; from startouchclass import __version__; print(startouch.__version__, __version__)"
```

Expected output for this release:

```text
0.1.5 0.1.5
```

## Supported Ubuntu Runtime Libraries

The SDK ships prebuilt `libstartouch.so` variants:

- Ubuntu 20.04: `src/libstartouch.so.20`
- Ubuntu 22.04: `src/libstartouch.so.22`
- Ubuntu 24.04: `src/libstartouch.so.24`

The build selects the matching library for the local system. When C++ symbols or
runtime behavior change, all three variants should be rebuilt and verified
before publishing.

