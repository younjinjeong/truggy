FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# Install clangd and build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    clangd-15 \
    cmake \
    build-essential \
    libeigen3-dev \
    libyaml-cpp-dev \
    zlib1g-dev \
    wget \
    ca-certificates \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-15 100 \
    && rm -rf /var/lib/apt/lists/*

# Install CUDA headers only (no runtime — this is for LSP indexing, not execution)
# Using CUDA 10.2 to match JetPack 4.6 on Nano 2GB
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb \
    && dpkg -i cuda-keyring_1.0-1_all.deb \
    && rm cuda-keyring_1.0-1_all.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       cuda-cudart-dev-10-2 \
       cuda-nvcc-10-2 \
       cuda-curand-dev-10-2 \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="/usr/local/cuda-10.2/bin:${PATH}"
ENV CUDA_HOME="/usr/local/cuda-10.2"

# Create workspace mount point
RUN mkdir -p /workspace
WORKDIR /workspace

# Expose clangd on a port for remote LSP clients
# clangd doesn't natively listen on a port, so we use socat or
# the client connects via kubectl exec / port-forward
ENTRYPOINT ["clangd", "--background-index", "--clang-tidy", "--header-insertion=iwyu", "--log=info"]
