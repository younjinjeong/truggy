# clangd LSP on Kubernetes

Deploy clangd as a persistent pod on your local k8s/k3s cluster for C++/CUDA development.

## Quick Start

### 1. Build the Docker image

```bash
cd deploy/clangd
docker build -t truggy/clangd:latest .
```

If using k3s with containerd (no Docker), import the image:
```bash
docker save truggy/clangd:latest | sudo k3s ctr images import -
```

### 2. Update hostPath

Edit `k8s-deployment.yaml` and set the `hostPath` to your local truggy source directory:
```yaml
volumes:
- name: workspace
  hostPath:
    path: /home/younjinjeong/Workspace/TruggyAD/truggy  # ← adjust this
```

### 3. Deploy

```bash
kubectl apply -f k8s-deployment.yaml
kubectl -n truggy-dev get pods    # wait for Running
```

### 4. Generate compile_commands.json

clangd needs `compile_commands.json` for accurate indexing. Generate it:
```bash
cd /path/to/truggy
cmake -B build -DTARGET_NANO_2GB=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# compile_commands.json is created in build/
```

### 5. Connect your editor

#### Option A: kubectl exec (simplest)

Use your editor's remote LSP support to connect to clangd inside the pod:

```bash
# Get pod name
POD=$(kubectl -n truggy-dev get pod -l app=clangd-lsp -o jsonpath='{.items[0].metadata.name}')

# Interactive clangd session (for testing)
kubectl -n truggy-dev exec -it $POD -- clangd --background-index --clang-tidy

# For VS Code: use the "clangd" extension with a custom command:
# "clangd.path": "kubectl -n truggy-dev exec -i <pod-name> -- clangd --background-index"
```

#### Option B: Port forward (for editors that need TCP)

Some editors/plugins support connecting to clangd over a network socket. You can use socat inside the container to bridge stdin/stdout to a TCP port, then port-forward:

```bash
# Inside the pod (one-time setup)
kubectl -n truggy-dev exec -it $POD -- bash -c \
  "apt-get update && apt-get install -y socat && \
   socat TCP-LISTEN:50051,reuseaddr,fork EXEC:'clangd --background-index --clang-tidy'"

# Port forward
kubectl -n truggy-dev port-forward svc/clangd-lsp 50051:50051
```

#### Option C: Local clangd (no k8s needed)

If you prefer to run clangd locally:
```bash
sudo apt install clangd-15
# Ensure compile_commands.json exists in build/
# Your editor will pick it up automatically
```

## .clangd Configuration

The repo includes a `.clangd` file:
```yaml
CompileFlags:
  CompilationDatabase: build
  Add: [-std=c++17, --cuda-gpu-arch=sm_53]
```

## Troubleshooting

### clangd can't find CUDA headers
Add to `.clangd`:
```yaml
CompileFlags:
  Add: [-I/usr/local/cuda-10.2/include]
```

### Index is slow
First-time indexing of CUDA/Eigen headers takes 2-5 minutes. The index is cached in `/root/.cache/clangd` (persisted via emptyDir volume).

### Pod keeps crashing
Check logs: `kubectl -n truggy-dev logs -l app=clangd-lsp`
Common cause: hostPath doesn't exist or wrong permissions.
