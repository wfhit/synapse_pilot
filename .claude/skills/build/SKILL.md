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
| `nxt-dual` | `hkust_nxt-dual_default` |
| `cuav-x7pro` | `cuav_x7pro_default` |
| `cuav-nora` | `cuav_nora_default` |
| `cuav-7nano` | `cuav_7-nano_default` |
| `cuav-x25evo` | `cuav_x25-evo_default` |
| `sitl` | `px4_sitl_default` |
| `tests` | `tests` |

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
2. Run the Docker build command with a 10-minute timeout
3. Show the last 20 lines of build output
4. If the build succeeds, report the firmware path: `build/<target>/<target>.px4`
5. If the build fails, show the error and suggest `make clean` if it looks like a stale build issue

## Clean Build

If the user passes `clean` as an additional argument (e.g., `/build nxt-front clean`), run a clean first:

```
docker run --rm -v <repo-root>:<repo-root> -w <repo-root> px4io/px4-dev:v1.16.0-rc1-258-g0369abd556 make clean
```

Then proceed with the build.
