#include "common/types.h"
#include "common/config.h"
#include "common/timing.h"
#include "actuation/bridge.h"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <pthread.h>
#include <sched.h>

#include <cuda_runtime.h>

using namespace truggy;

static shared_bus_t* g_bus = nullptr;

static void signal_handler(int sig) {
    (void)sig;
    if (g_bus) {
        fprintf(stderr, "\n[main] SIGINT received, shutting down...\n");
        g_bus->alive.store(false, std::memory_order_release);
    }
}

// ── Thread entry points ─────────────────────────────────────────────────────

static void* perception_thread(void* arg) {
    auto* bus = (shared_bus_t*)arg;
    fprintf(stderr, "[T0] Perception thread started\n");
    // TODO(epic3): call perception_loop(bus, cfg)
    while (bus->alive.load(std::memory_order_relaxed)) {
        sleep_until_us(now_us() + 33333);  // ~30 Hz placeholder
    }
    fprintf(stderr, "[T0] Perception thread stopped\n");
    return nullptr;
}

static void* planning_thread(void* arg) {
    auto* bus = (shared_bus_t*)arg;
    fprintf(stderr, "[T1] Planning thread started\n");
    // TODO(epic4): call planning_loop(bus, cfg)
    while (bus->alive.load(std::memory_order_relaxed)) {
        sleep_until_us(now_us() + 20000);  // ~50 Hz placeholder
    }
    fprintf(stderr, "[T1] Planning thread stopped\n");
    return nullptr;
}

static void* state_estimation_thread(void* arg) {
    auto* bus = (shared_bus_t*)arg;
    fprintf(stderr, "[T2] State estimation thread started\n");
    // TODO(epic5): call state_estimation_loop(bus, cfg)
    while (bus->alive.load(std::memory_order_relaxed)) {
        sleep_until_us(now_us() + 10000);  // ~100 Hz placeholder
    }
    fprintf(stderr, "[T2] State estimation thread stopped\n");
    return nullptr;
}

struct thread_args_t {
    shared_bus_t* bus;
    config_t* cfg;
};

static void* actuation_thread(void* arg) {
    auto* ta = (thread_args_t*)arg;
    actuation_loop(ta->bus, *ta->cfg);
    return nullptr;
}

// ── Thread helpers ──────────────────────────────────────────────────────────

static void set_thread_affinity(pthread_t thread, int core) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
        fprintf(stderr, "[main] Warning: failed to set CPU affinity for core %d\n", core);
    }
}

static void set_thread_priority(pthread_t thread, int priority) {
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(thread, SCHED_FIFO, &param) != 0) {
        fprintf(stderr, "[main] Warning: failed to set SCHED_FIFO priority %d "
                        "(run as root or set CAP_SYS_NICE)\n", priority);
    }
}

// ── Main ────────────────────────────────────────────────────────────────────

static void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [--config <path>] [--costs <path>] [--sim]\n", prog);
    fprintf(stderr, "  --config  Path to truggy.yaml (default: config/truggy.yaml)\n");
    fprintf(stderr, "  --costs   Path to mppi_costs.yaml (default: config/mppi_costs.yaml)\n");
    fprintf(stderr, "  --sim     Simulation mode (skip hardware init)\n");
}

int main(int argc, char** argv) {
    const char* config_path = "config/truggy.yaml";
    const char* costs_path  = "config/mppi_costs.yaml";
    bool sim_mode = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else if (strcmp(argv[i], "--costs") == 0 && i + 1 < argc) {
            costs_path = argv[++i];
        } else if (strcmp(argv[i], "--sim") == 0) {
            sim_mode = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    // ── Load configuration ──────────────────────────────────────────────
    config_t cfg;
    if (!load_config(cfg, config_path, costs_path)) {
        fprintf(stderr, "[main] Failed to load configuration\n");
        return 1;
    }
    fprintf(stderr, "[main] Configuration loaded\n");
    fprintf(stderr, "[main] MPPI: %d rollouts, %d timesteps, %.2f Hz\n",
            MPPI_NUM_ROLLOUTS, cfg.mppi.num_timesteps, (float)cfg.mppi.hz);
    fprintf(stderr, "[main] Costmap: %dx%d, %.2f m/cell\n",
            cfg.costmap.width, cfg.costmap.height, cfg.costmap.cell_size);
    fprintf(stderr, "[main] Mode: %s\n", sim_mode ? "SIMULATION" : "HARDWARE");

    // ── Initialize CUDA ─────────────────────────────────────────────────
    int device_count = 0;
    cudaGetDeviceCount(&device_count);
    if (device_count == 0) {
        fprintf(stderr, "[main] No CUDA devices found\n");
        return 1;
    }
    cudaSetDevice(0);
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    fprintf(stderr, "[main] CUDA device: %s (sm_%d%d, %d cores, %zu MB)\n",
            prop.name, prop.major, prop.minor,
            prop.multiProcessorCount * 128,  // approximate
            prop.totalGlobalMem / (1024 * 1024));

    // ── Allocate shared bus ─────────────────────────────────────────────
    auto* bus = new(std::align_val_t{128}) shared_bus_t{};
    g_bus = bus;

    // ── Signal handler ──────────────────────────────────────────────────
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // ── Start threads ───────────────────────────────────────────────────
    pthread_t t0, t1, t2, t3;
    thread_args_t ta = {bus, &cfg};

    fprintf(stderr, "[main] Starting threads...\n");

    // T3: Actuation first (establish Arduino link)
    if (!sim_mode) {
        pthread_create(&t3, nullptr, actuation_thread, &ta);
        set_thread_affinity(t3, 2);
        set_thread_priority(t3, 80);
        fprintf(stderr, "[main] T3 (Actuation) started, waiting for first IMU...\n");
        // Wait for first IMU packet (up to 3 seconds)
        for (int i = 0; i < 300 && bus->alive.load(); i++) {
            if (bus->imu.timestamp_us > 0) break;
            sleep_until_us(now_us() + 10000);
        }
    }

    // T2: State Estimation
    pthread_create(&t2, nullptr, state_estimation_thread, bus);
    set_thread_affinity(t2, 1);

    // T0: Perception
    pthread_create(&t0, nullptr, perception_thread, bus);
    set_thread_affinity(t0, 3);

    // T1: Planning (highest priority)
    pthread_create(&t1, nullptr, planning_thread, bus);
    set_thread_affinity(t1, 0);
    set_thread_priority(t1, 90);

    fprintf(stderr, "[main] All threads running. Press Ctrl+C to stop.\n");

    // ── Wait for shutdown ───────────────────────────────────────────────
    pthread_join(t1, nullptr);
    pthread_join(t0, nullptr);
    pthread_join(t2, nullptr);
    if (!sim_mode) {
        pthread_join(t3, nullptr);
    }

    // ── Cleanup ─────────────────────────────────────────────────────────
    fprintf(stderr, "[main] All threads joined. Cleaning up.\n");
    cudaDeviceReset();
    operator delete(bus, std::align_val_t{128});

    fprintf(stderr, "[main] Shutdown complete.\n");
    return 0;
}
