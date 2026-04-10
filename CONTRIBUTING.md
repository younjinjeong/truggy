# Contributing to TruggyAD

## Development Workflow

```
GitHub Issue (Story) → feature branch → code → build & test → Gazebo test → PR → merge
```

### 1. Pick a Story

All work is tracked as GitHub Issues organized into Epics:

| Epic | Focus |
|------|-------|
| #1 | Developer environment & skills |
| #6 | Foundation (build system, shared types) |
| #14 | Arduino firmware & actuation |
| #18 | Perception pipeline |
| #24 | MPPI controller port |
| #30 | State estimation (EKF) |
| #33 | Integration & testing |
| #39 | Orin Nano upgrade |

Pick a Story from the current Epic. Assign yourself.

### 2. Create Feature Branch

Branch from `main` using the pattern: `<issue-number>/<short-name>`

```bash
git checkout main
git pull origin main
git checkout -b 7/project-scaffold
```

### 3. Implement

- Follow the coding conventions below
- Reference the Story issue in commits: `Refs #7`
- Keep commits focused and atomic

### 4. Build & Test Locally

```bash
# Native build on Jetson
cmake -B build -DTARGET_NANO_2GB=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j4

# Cross-compile on WSL2 (if Jetson unavailable)
cmake -B build-cross -DCMAKE_TOOLCHAIN_FILE=cmake/jetson-nano-toolchain.cmake -DTARGET_NANO_2GB=ON
cmake --build build-cross -j$(nproc)
```

### 5. Test with Gazebo (if applicable)

Not all stories require Gazebo testing. Use Gazebo for:
- Perception pipeline changes
- MPPI controller changes
- State estimation changes
- Integration testing

```bash
cmake -B build -DUSE_GAZEBO_SIM=ON -DTARGET_NANO_2GB=ON
cmake --build build -j4
gz sim sim/gazebo/track.sdf &
./build/truggy --sim --config config/truggy.yaml
```

### 6. Create Pull Request

```bash
git push -u origin 7/project-scaffold
gh pr create --title "feat: create project scaffold" --body "Closes #7"
```

PR description should include:
- What was changed and why
- How to test
- Reference to the Story issue (`Closes #N`)

### 7. Merge

After review, merge to `main`. The Story issue auto-closes from `Closes #N`.

## Coding Conventions

### C++ Style

- **Standard:** C++17
- **Style:** C-like C++ — structs over classes, free functions over methods where possible
- **Naming:** `snake_case` for everything (variables, functions, types, files)
- **No exceptions:** Use return codes or status enums
- **No RTTI:** No `dynamic_cast`, no `typeid`
- **Minimal STL:** Prefer fixed-size arrays, manual memory management in hot paths
- **Headers:** `.h` for C++ headers, `.cuh` not used (we keep CUDA in `.cu` files)

### Example

```cpp
// Good
struct costmap_t {
    float* data;
    int width;
    int height;
    float cell_size;
};

bool generate_costmap(const float* depth, const uint8_t* mask,
                      costmap_t* out, const camera_params_t* cam);

// Avoid
class CostmapGenerator {
public:
    virtual ~CostmapGenerator() = default;
    std::shared_ptr<Costmap> generate(const cv::Mat& depth);
};
```

### CUDA Style

- Kernel names: `snake_case` with descriptive suffix (e.g., `rollout_kernel`, `generate_costmap_kernel`)
- Use `--maxrregcount=32` and `-use_fast_math`
- Prefer `__shared__` memory over global memory accesses
- Document thread/block dimensions in comments above kernel launch
- Use `__syncthreads()` with comment explaining what synchronization is needed

### File Organization

```
src/module/
  module.h      ← public interface (structs + function declarations)
  module.cpp    ← CPU implementation
  module.cu     ← CUDA implementation (if any)
```

### Commit Messages

Format: `<type>: <short description>`

Types:
- `feat:` — new feature or module
- `fix:` — bug fix
- `port:` — porting code from AutoRally
- `docs:` — documentation
- `test:` — test tools or benchmarks
- `build:` — CMake or build system changes
- `tune:` — parameter tuning

```
feat: implement seqlock-based shared bus
port: MPPI rollout kernel from AutoRally
fix: CRC-8 mismatch in telemetry parsing
tune: increase track_coeff to 300.0 for better track following
```

## Build Environments

| Environment | Use Case | CUDA Build |
|-------------|----------|------------|
| Jetson Nano 2GB | Primary development & testing | Native (sm_53) |
| WSL2 + RTX 4080 | Fast compilation, CUDA dev | Cross-compile to aarch64 |
| Jetson Orin Nano | Future upgrade | Native (sm_87) |

If you encounter CUDA or NVIDIA package build issues on WSL2, document the problem in the PR and tag for help.

## Hardware Access

- **Arduino:** `/dev/ttyACM0` (add user to `dialout` group)
- **ZED Camera:** USB 3.0 port on Jetson
- **Serial permission:** `sudo usermod -aG dialout $USER`
