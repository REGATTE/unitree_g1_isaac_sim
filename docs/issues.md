# Known Issues And Setup Notes

## Native Unitree SDK2 Bridge Crashes With `free(): invalid pointer`

### Symptom

The native bridge or a stock `unitree_sdk2` example exits immediately with:

```text
free(): invalid pointer
```

This can happen when the process loads a mixed CycloneDDS runtime:

- `libddscxx` from `~/unitree_sdk2/thirdparty/lib/<arch>`
- `libddsc` from another install, such as `/opt/ros/humble`

This mismatch was observed while building the native G1 bridge on a system that
also had ROS 2 Humble sourced.

### How To Check

Build the native bridge:

```bash
cmake -S native_sdk_bridge -B native_sdk_bridge/build
cmake --build native_sdk_bridge/build -j4
```

Check which DDS libraries the executable resolves:

```bash
ldd native_sdk_bridge/build/unitree_g1_native_bridge | grep ddsc
```

Expected:

```text
libddsc.so.0 => /home/<user>/unitree_sdk2/thirdparty/lib/<arch>/libddsc.so.0
libddscxx.so.0 => /home/<user>/unitree_sdk2/thirdparty/lib/<arch>/libddscxx.so.0
```

Problem case:

```text
libddsc.so.0 => /opt/ros/humble/lib/.../libddsc.so.0
libddscxx.so.0 => /home/<user>/unitree_sdk2/thirdparty/lib/<arch>/libddscxx.so.0
```

### Fix

The repo's native bridge build and launcher are set up to prefer the Unitree
SDK2 bundled DDS libraries:

- `native_sdk_bridge/CMakeLists.txt` emits an `RPATH` to
  `~/unitree_sdk2/thirdparty/lib/<arch>`.
- `src/dds/native_manager.py` prepends that same directory to
  `LD_LIBRARY_PATH` when launching the native bridge subprocess.

If setting up manually on a new machine, make sure this path comes before ROS 2
library paths:

```bash
export LD_LIBRARY_PATH="$HOME/unitree_sdk2/thirdparty/lib/$(uname -m):$LD_LIBRARY_PATH"
```

Then recheck:

```bash
ldd native_sdk_bridge/build/unitree_g1_native_bridge | grep ddsc
```

### Notes

- This issue is not necessarily a broken `unitree_sdk2` install.
- It is usually a dynamic-linker resolution problem caused by ROS 2 and
  Unitree SDK2 both shipping CycloneDDS-related libraries.
- If the bridge still crashes after both DDS libraries resolve from
  `~/unitree_sdk2`, run the stock SDK publisher as a baseline:

```bash
~/unitree_sdk2/build/bin/test_publisher
```

If both the stock SDK publisher and this bridge fail with matching symptoms,
debug the SDK/DDS runtime environment before changing simulator bridge logic.
