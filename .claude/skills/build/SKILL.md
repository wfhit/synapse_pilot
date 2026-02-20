---
name: build
description: Build firmware for a target board using Docker
argument-hint: <board-shortname>
---

# Build Firmware

Build PX4 firmware for a target board using Docker.

## Target Mapping

Map the argument to the full make target:

| Shortname | Make Target |
|-----------|-------------|
| `nxt-front` | `wheel_loader_nxt-dual-wl-front_default` |
| `nxt-rear` | `wheel_loader_nxt-dual-wl-rear_default` |
| `cuav-wl` | `wheel_loader_cuav-x7plus-wl_default` |
| `holybro` | `wheel_loader_holybro-v6xrt-wl_default` |

If `$ARGUMENTS` is empty, ask the user which board to build.
If the argument doesn't match a shortname, try using it as a literal make target.

## Build Command

**All builds MUST use Docker.** Never build on the host directly.

For NuttX targets, run:

```
docker run --rm -v <repo-root>:<repo-root> -w <repo-root> px4io/px4-dev:v1.16.0-rc1-258-g0369abd556 make <target>
```

Where `<repo-root>` is the absolute path to the repository root.

For SITL or tests, run:

```
./Tools/docker_run.sh "make <target>"
```

## Steps

1. Resolve the board shortname from `$ARGUMENTS` to a make target
2. Log: `[build] Starting build for <target> at <timestamp>`
3. Run the Docker build command with a 10-minute timeout, capturing output to a log file alongside stdout:
   ```
   docker run --rm -v <repo-root>:<repo-root> -w <repo-root> px4io/px4-dev:v1.16.0-rc1-258-g0369abd556 make <target> 2>&1 | tee /tmp/px4_build_<target>.log
   ```
4. While building, periodically report progress by showing the last compile unit being processed:
   - Every ~30 seconds print: `[build] In progress... last line: <last non-empty line from log>`
5. On completion:
   - Log: `[build] Build finished — exit code <N> — elapsed <duration>s`
   - Show the last 20 lines of build output
6. If the build succeeds, report: `[build] ✓ Firmware: build/<target>/<target>.px4`
7. If the build fails, show the error context (last 40 lines) and suggest `make clean` if it looks like a stale build issue. Log: `[build] ✗ Build failed — see /tmp/px4_build_<target>.log`

## Clean Build

If the user passes `clean` as an additional argument (e.g., `/build nxt-front clean`), run a clean first:

```
docker run --rm -v <repo-root>:<repo-root> -w <repo-root> px4io/px4-dev:v1.16.0-rc1-258-g0369abd556 make clean
```

Log: `[build] Clean complete, starting build...`

Then proceed with the build.
