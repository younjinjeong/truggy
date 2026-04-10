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
# Using CUDA 12.2 for header compatibility (clangd only needs headers, not matching target)
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && rm cuda-keyring_1.1-1_all.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       cuda-cudart-dev-12-2 \
       cuda-nvcc-12-2 \
       cuda-curand-dev-12-2 \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="/usr/local/cuda-12.2/bin:${PATH}"
ENV CUDA_HOME="/usr/local/cuda-12.2"

# Create workspace mount point
RUN mkdir -p /workspace
WORKDIR /workspace

# Expose clangd on a port for remote LSP clients
# clangd doesn't natively listen on a port, so we use socat or
# the client connects via kubectl exec / port-forward
ENTRYPOINT ["clangd", "--background-index", "--clang-tidy", "--header-insertion=iwyu", "--log=info"]
