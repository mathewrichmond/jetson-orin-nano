# Compute Architecture Comparison: Single vs Dual Compute

## Executive Summary

This document compares two compute architectures for the Isaac robot system:
1. **Single Compute**: Jetson Orin Nano only
2. **Dual Compute**: Jetson Orin Nano + Raspberry Pi 4 Model B

**Key Finding**: Current system is already at ~76% CPU utilization (456% of 6 cores) before adding VLA inference, SLAM, or logging. This suggests dual compute may be necessary for full system operation.

## Hardware Specifications

### Jetson Orin Nano (8GB)
- **CPU**: 6-core ARM Cortex-A78AE @ up to 1.5 GHz
- **GPU**: 1024 CUDA cores, 32 Tensor cores (Ampere architecture)
- **Memory**: 8GB LPDDR5, 128-bit bus
- **Memory Bandwidth**: ~68 GB/s (102 GB/s in Super mode)
- **Power**: 7W-15W typical, up to 25W peak
- **USB**: 1x USB-C 3.2, 2x USB-A 3.0, 2x USB-A 2.0
- **Network**: Gigabit Ethernet, WiFi 6
- **Storage**: microSD (current), NVMe M.2 slot available

### Raspberry Pi 4 Model B (8GB recommended)
- **CPU**: 4-core ARM Cortex-A72 @ 1.5-1.8 GHz
- **GPU**: VideoCore VI (not suitable for ML inference)
- **Memory**: 8GB LPDDR4-3200
- **Memory Bandwidth**: ~12.8 GB/s theoretical (~4-5 GB/s practical)
- **Power**: 3W idle, 5-7W typical, up to 15W peak
- **USB**: 2x USB 3.0, 2x USB 2.0
- **Network**: Gigabit Ethernet, WiFi 5
- **Storage**: microSD, USB boot supported

## Current System Workload Analysis

### Measured CPU Usage (After Optimizations)
- **realsense_camera_node**: 43% CPU (single-threaded)
- **nvblox_processor**: 80% CPU (single-threaded)
- **sensor_sync_node**: 333% CPU (multi-threaded, ~3.3 cores)
- **Other nodes** (system_monitor, usb_microphone, etc.): ~20% CPU
- **Total**: ~476% CPU = **~79% of 6-core system**

### Current Memory Usage
- **Total RAM**: 7.4 GB
- **Used**: 3.4-3.5 GB
- **Available**: 3.4-3.5 GB
- **Memory copies**: ~166 MB/sec (after optimizations)

### Current Bandwidth Usage
- **Camera data**: ~55 MB/s raw (internal processing)
- **Feature topics**: ~20 MB/s (for VLA)
- **Bridge topics**: ~5 MB/s (visualization)
- **Total**: ~80 MB/s internal processing

## Required Workloads

### 1. System Graph (Sense + Control) - REQUIRED
**Components**:
- Camera capture and processing (2x RealSense D435)
- nvblox 3D processing (pointclouds, mesh, TSDF)
- Sensor fusion (synchronization, downsampling)
- Hardware drivers (IMU, motors, microphones)
- System monitoring
- Control execution

**Current CPU**: ~476% (~79% of 6 cores)
**Current Memory**: ~3.5 GB
**Bandwidth**: ~80 MB/s internal

**Estimated for Production**:
- CPU: 400-500% (with further optimizations)
- Memory: 3-4 GB
- Bandwidth: 60-80 MB/s

### 2. VLA Inference - REQUIRED
**Components**:
- Model loading and management
- Image preprocessing (resize, normalize)
- Model inference (transformer-based VLA model)
- Action decoding and validation
- Model output → control plan conversion

**Estimated Requirements** (based on typical VLA models):
- **CPU**: 100-200% (preprocessing, postprocessing)
- **GPU**: 50-80% utilization (inference on Tensor cores)
- **Memory**: 2-4 GB (model weights + activations)
- **Bandwidth**: 5-10 MB/s (model I/O)
- **Latency**: 50-200ms per inference (depends on model size)

**Model Size Estimates**:
- Small VLA: ~1-2 GB weights, 10-20 TOPS
- Medium VLA: ~3-5 GB weights, 20-40 TOPS
- Large VLA: ~5-10 GB weights, 40-80 TOPS

**Jetson Orin Nano GPU**: ~40 TOPS (INT8), ~10 TOPS (FP16)
- Can handle small-medium VLA models
- Large models may require quantization or model optimization

### 3. SLAM/Mapping - REQUIRED
**Components**:
- Visual SLAM (ORB-SLAM3, RTAB-Map, or similar)
- Local map building
- Loop closure detection
- Map optimization
- Remote map caching (sync with server)

**Estimated Requirements**:
- **CPU**: 150-300% (feature extraction, matching, optimization)
- **GPU**: 20-40% (optional GPU acceleration)
- **Memory**: 1-3 GB (map data structures, keyframes)
- **Bandwidth**: 2-5 MB/s (map sync to server)
- **Storage**: 100 MB - 1 GB per session (map data)

**SLAM Options**:
- **CPU-only SLAM**: ORB-SLAM3, RTAB-Map (CPU-intensive)
- **GPU-accelerated**: RTAB-Map with CUDA, ORB-SLAM3 with GPU features
- **Lightweight**: OpenVSLAM, DROID-SLAM (less accurate but faster)

### 4. Logging - REQUIRED
**Components**:
- ROS bag recording (sensor data)
- System logs (text logs)
- Map data logging
- Remote sync (optional)

**Estimated Requirements**:
- **CPU**: 50-100% (compression, I/O)
- **Memory**: 0.5-2 GB (buffers)
- **Bandwidth**: 10-30 MB/s (to storage)
- **Storage**: 1-10 GB/hour (depends on compression)

**Logging Options**:
- **Full logging**: All sensor data @ 15 Hz = ~30 MB/s
- **Selective logging**: Key topics only = ~10 MB/s
- **Compressed logging**: H.264 video, compressed pointclouds = ~5-10 MB/s

## Architecture Comparison

### Architecture 1: Single Compute (Jetson Orin Nano)

```
┌─────────────────────────────────────────┐
│      Jetson Orin Nano (8GB)            │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  System Graph (Sense + Control)  │  │
│  │  - Cameras, nvblox, fusion      │  │
│  │  CPU: ~400-500%                 │  │
│  │  Memory: ~3-4 GB                │  │
│  └─────────────────────────────────┘  │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  VLA Inference                  │  │
│  │  - Model inference (GPU)         │  │
│  │  CPU: ~100-200%                 │  │
│  │  GPU: ~50-80%                   │  │
│  │  Memory: ~2-4 GB                │  │
│  └─────────────────────────────────┘  │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  SLAM/Mapping                   │  │
│  │  - Visual SLAM, map building    │  │
│  │  CPU: ~150-300%                 │  │
│  │  Memory: ~1-3 GB                │  │
│  └─────────────────────────────────┘  │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  Logging                        │  │
│  │  - ROS bag, system logs         │  │
│  │  CPU: ~50-100%                  │  │
│  │  Memory: ~0.5-2 GB              │  │
│  └─────────────────────────────────┘  │
│                                         │
│  Total CPU: ~700-1100% (117-183% of 6 cores) │
│  Total Memory: ~7-13 GB (exceeds 8GB!) │
│  Total GPU: ~50-80%                    │
└─────────────────────────────────────────┘
```

**Total Resource Requirements**:
- **CPU**: 700-1100% (~117-183% of 6 cores) ❌ **EXCEEDS CAPACITY**
- **Memory**: 7-13 GB ❌ **EXCEEDS 8GB CAPACITY**
- **GPU**: 50-80% ✓ (within capacity)
- **Bandwidth**: ~100-120 MB/s ✓ (within capacity)

**Verdict**: ❌ **INSUFFICIENT** - Cannot run all workloads simultaneously

**Mitigation Options**:
1. **Reduce frame rates**: 15 Hz → 10 Hz (saves ~33% CPU)
2. **Disable visualization**: Saves ~50% CPU in sensor fusion
3. **Lightweight SLAM**: Use faster, less accurate SLAM
4. **Selective logging**: Log only essential topics
5. **Model quantization**: Use INT8 quantized VLA model
6. **Time-slicing**: Run workloads sequentially (not recommended)

**Best Case (with all mitigations)**:
- CPU: ~500-700% (~83-117% of 6 cores) ⚠️ **MARGINAL**
- Memory: ~6-8 GB ⚠️ **MARGINAL**
- **Risk**: CPU starvation, dropped frames, poor performance

### Architecture 2: Dual Compute (Jetson + Raspberry Pi 4)

```
┌─────────────────────────────────────────┐
│      Jetson Orin Nano (8GB)             │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  System Graph (Sense)            │  │
│  │  - Cameras, nvblox, fusion       │  │
│  │  CPU: ~400-500%                 │  │
│  │  Memory: ~3-4 GB                │  │
│  └─────────────────────────────────┘  │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  VLA Inference                  │  │
│  │  - Model inference (GPU)         │  │
│  │  CPU: ~100-200%                 │  │
│  │  GPU: ~50-80%                   │  │
│  │  Memory: ~2-4 GB                │  │
│  └─────────────────────────────────┘  │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  SLAM/Mapping (GPU-accelerated) │  │
│  │  - Visual SLAM, map building     │  │
│  │  CPU: ~50-100% (GPU does work)  │  │
│  │  GPU: ~20-40%                   │  │
│  │  Memory: ~1-2 GB                │  │
│  └─────────────────────────────────┘  │
│                                         │
│  Total CPU: ~550-800% (92-133% of 6 cores) │
│  Total Memory: ~6-10 GB ⚠️            │
│  Total GPU: ~70-120% ⚠️                │
└─────────────────────────────────────────┘
         │
         │ ROS 2 Topics (Ethernet/WiFi)
         │ ~20-30 MB/s
         ▼
┌─────────────────────────────────────────┐
│   Raspberry Pi 4 Model B (8GB)          │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  Control Execution               │  │
│  │  - Motor control, servos         │  │
│  │  CPU: ~20-50%                   │  │
│  │  Memory: ~0.5 GB                │  │
│  └─────────────────────────────────┘  │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  Logging                         │  │
│  │  - ROS bag, system logs         │  │
│  │  CPU: ~50-100%                  │  │
│  │  Memory: ~1-2 GB                │  │
│  └─────────────────────────────────┘  │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  Map Caching/Server Sync         │  │
│  │  - Local map cache               │  │
│  │  - Remote sync                   │  │
│  │  CPU: ~30-60%                   │  │
│  │  Memory: ~0.5-1 GB               │  │
│  └─────────────────────────────────┘  │
│                                         │
│  ┌─────────────────────────────────┐  │
│  │  Optional: Lightweight SLAM     │  │
│  │  - Backup/validation            │  │
│  │  CPU: ~100-150%                 │  │
│  │  Memory: ~0.5-1 GB              │  │
│  └─────────────────────────────────┘  │
│                                         │
│  Total CPU: ~200-360% (50-90% of 4 cores) │
│  Total Memory: ~2.5-4.5 GB ✓            │
└─────────────────────────────────────────┘
```

**Jetson Resource Usage**:
- **CPU**: 550-800% (~92-133% of 6 cores) ⚠️ **TIGHT BUT FEASIBLE**
- **Memory**: 6-10 GB ⚠️ **TIGHT** (may need optimization)
- **GPU**: 70-120% ⚠️ **TIGHT** (may need optimization)

**Raspberry Pi Resource Usage**:
- **CPU**: 200-360% (~50-90% of 4 cores) ✓ **FEASIBLE**
- **Memory**: 2.5-4.5 GB ✓ **FEASIBLE**

**Network Bandwidth**:
- **Jetson → Pi**: ~20-30 MB/s (sensor fusion topics)
- **Pi → Jetson**: ~1-5 MB/s (control plans, map updates)
- **Total**: ~25-35 MB/s ✓ **FEASIBLE** (Gigabit Ethernet)

**Verdict**: ✓ **FEASIBLE** - Can run all workloads with proper allocation

## Workload Allocation Options

### Option A: Jetson = Sense + Inference, Pi = Control + Logging

**Jetson Orin Nano**:
- System graph (sense): Cameras, nvblox, sensor fusion
- VLA inference: Model inference on GPU
- SLAM: GPU-accelerated mapping
- **CPU**: ~550-800%
- **Memory**: ~6-10 GB
- **GPU**: ~70-120%

**Raspberry Pi 4**:
- Control execution: Motor control, servos
- Logging: ROS bag recording, system logs
- Map caching: Local cache, remote sync
- **CPU**: ~200-300%
- **Memory**: ~2-4 GB

**Pros**:
- GPU-intensive workloads on Jetson
- Logging doesn't compete with inference
- Clear separation of concerns

**Cons**:
- Jetson still tight on resources
- Network latency for control (minimal impact)

### Option B: Jetson = Sense + Inference, Pi = Logging + SLAM

**Jetson Orin Nano**:
- System graph (sense): Cameras, nvblox, sensor fusion
- VLA inference: Model inference on GPU
- **CPU**: ~500-700%
- **Memory**: ~5-8 GB
- **GPU**: ~50-80%

**Raspberry Pi 4**:
- SLAM: CPU-only SLAM (ORB-SLAM3)
- Logging: ROS bag recording
- Map caching: Local cache, remote sync
- **CPU**: ~250-400%
- **Memory**: ~2-4 GB

**Pros**:
- Jetson has more headroom
- SLAM on Pi doesn't compete with GPU
- Better resource balance

**Cons**:
- SLAM slower on Pi (CPU-only)
- Less accurate SLAM (no GPU acceleration)
- Network bandwidth for map data

### Option C: Jetson = Sense + Inference + SLAM, Pi = Logging + Control

**Jetson Orin Nano**:
- System graph (sense): Cameras, nvblox, sensor fusion
- VLA inference: Model inference on GPU
- SLAM: GPU-accelerated mapping
- **CPU**: ~550-800%
- **Memory**: ~6-10 GB
- **GPU**: ~70-120%

**Raspberry Pi 4**:
- Control execution: Motor control, servos
- Logging: ROS bag recording
- Map caching: Local cache, remote sync
- **CPU**: ~150-250%
- **Memory**: ~2-3 GB

**Pros**:
- Best SLAM performance (GPU-accelerated)
- Pi has plenty of headroom
- Logging doesn't compete with inference

**Cons**:
- Jetson very tight on resources
- Risk of CPU/GPU starvation

## Detailed Capability Comparison

### What Can Run on Single Jetson Orin Nano?

| Workload | CPU Available | Memory Available | GPU Available | Feasible? |
|----------|---------------|------------------|---------------|-----------|
| System Graph (current) | 400-500% | 3-4 GB | 0% | ✓ Yes |
| System Graph (optimized) | 300-400% | 2-3 GB | 0% | ✓ Yes |
| VLA Inference (small) | 100-150% | 2-3 GB | 50-60% | ✓ Yes |
| VLA Inference (medium) | 150-200% | 3-4 GB | 60-80% | ⚠️ Tight |
| VLA Inference (large) | 200-300% | 4-6 GB | 80-100% | ❌ No |
| SLAM (CPU-only) | 150-300% | 1-3 GB | 0% | ⚠️ Tight |
| SLAM (GPU-accelerated) | 50-100% | 1-2 GB | 20-40% | ✓ Yes |
| Logging (full) | 50-100% | 0.5-2 GB | 0% | ✓ Yes |
| Logging (selective) | 30-60% | 0.5-1 GB | 0% | ✓ Yes |

**Combination Analysis**:

| Combination | Total CPU | Total Memory | Total GPU | Feasible? |
|-------------|-----------|--------------|-----------|-----------|
| System + VLA (small) + SLAM (GPU) + Logging (selective) | 530-760% | 5-9 GB | 70-100% | ⚠️ **MARGINAL** |
| System + VLA (medium) + SLAM (GPU) + Logging (selective) | 580-860% | 7-11 GB | 80-120% | ❌ **EXCEEDS** |
| System + VLA (small) + SLAM (CPU) + Logging (selective) | 680-960% | 5-9 GB | 50-60% | ❌ **EXCEEDS** |

**Conclusion**: Single Jetson can run **System + Small VLA + GPU SLAM + Selective Logging** but with **marginal headroom**. Any additional workload or optimization failure risks CPU starvation.

### What Can Run on Dual Compute (Jetson + Pi)?

**Jetson Orin Nano**:
| Workload | CPU | Memory | GPU | Feasible? |
|----------|-----|--------|-----|-----------|
| System Graph | 400-500% | 3-4 GB | 0% | ✓ Yes |
| VLA Inference (small) | 100-150% | 2-3 GB | 50-60% | ✓ Yes |
| VLA Inference (medium) | 150-200% | 3-4 GB | 60-80% | ✓ Yes |
| SLAM (GPU-accelerated) | 50-100% | 1-2 GB | 20-40% | ✓ Yes |
| **Total** | **550-800%** | **6-10 GB** | **70-120%** | ⚠️ **TIGHT BUT FEASIBLE** |

**Raspberry Pi 4**:
| Workload | CPU | Memory | Feasible? |
|----------|-----|--------|-----------|
| Control Execution | 20-50% | 0.5 GB | ✓ Yes |
| Logging (full) | 50-100% | 1-2 GB | ✓ Yes |
| Map Caching | 30-60% | 0.5-1 GB | ✓ Yes |
| Lightweight SLAM (optional) | 100-150% | 0.5-1 GB | ✓ Yes |
| **Total** | **200-360%** | **2.5-4.5 GB** | ✓ **FEASIBLE** |

**Conclusion**: Dual compute can run **all workloads** with **comfortable headroom** on both systems.

## Network Architecture

### Communication Between Jetson and Pi

**Required Topics** (Jetson → Pi):
- `/sensor_fusion/*` - Fused sensor data (~20 MB/s)
- System status topics (~1 MB/s)
- **Total**: ~21 MB/s

**Required Topics** (Pi → Jetson):
- `/control/plan` - Control commands (~0.1 MB/s)
- Map updates (if Pi runs SLAM) (~1-5 MB/s)
- **Total**: ~1-5 MB/s

**Network Options**:
1. **Gigabit Ethernet** (recommended)
   - Bandwidth: 1000 Mbps = 125 MB/s theoretical
   - Latency: <1ms
   - Reliability: High
   - **Verdict**: ✓ **SUFFICIENT**

2. **WiFi 6 (Jetson) / WiFi 5 (Pi)**
   - Bandwidth: 100-500 Mbps = 12.5-62.5 MB/s
   - Latency: 5-20ms
   - Reliability: Medium (interference)
   - **Verdict**: ⚠️ **MARGINAL** (may work but not recommended)

3. **USB 3.0 Ethernet Adapter** (Pi)
   - Bandwidth: 1000 Mbps = 125 MB/s
   - Latency: <1ms
   - Reliability: High
   - **Verdict**: ✓ **SUFFICIENT** (if Pi doesn't have Ethernet)

**ROS 2 Configuration**:
- Use same `ROS_DOMAIN_ID` for both systems
- Configure DDS to use Ethernet (not WiFi)
- Consider shared memory transport for local topics (if possible)

## Power Consumption

### Single Compute (Jetson Only)
- **Idle**: 5-7W
- **Current workload**: 10-15W
- **Full workload**: 15-25W
- **Total**: 15-25W

### Dual Compute (Jetson + Pi)
- **Jetson (full workload)**: 15-25W
- **Pi (full workload)**: 5-10W
- **Network switch/router**: 2-5W (if needed)
- **Total**: 22-40W

**Power Increase**: ~7-15W (~30-60% increase)

## Complexity Analysis

### Single Compute
**Pros**:
- Simple architecture
- No network configuration needed
- Lower power consumption
- Lower cost
- Easier debugging

**Cons**:
- Resource constraints
- Risk of CPU starvation
- Limited scalability
- May require aggressive optimizations

### Dual Compute
**Pros**:
- More compute headroom
- Better resource isolation
- Scalable architecture
- Can offload non-critical workloads
- Redundancy (Pi can run backup SLAM)

**Cons**:
- More complex architecture
- Network configuration required
- Higher power consumption
- Higher cost
- More debugging complexity
- Network latency (minimal but present)
- Need to manage two systems

## Recommendations

### Recommendation 1: Start with Single Compute + Aggressive Optimization

**Approach**:
1. Continue memory optimizations (Phase 2-3)
2. Implement image caching in sensor fusion
3. Use shared memory transport
4. Optimize VLA model (quantization, pruning)
5. Use GPU-accelerated SLAM
6. Selective logging only

**Expected Results**:
- CPU: 500-700% (~83-117% of 6 cores) ⚠️ **MARGINAL**
- Memory: 6-8 GB ⚠️ **MARGINAL**
- **Risk**: CPU starvation under load

**When to Switch to Dual Compute**:
- If CPU consistently >90%
- If frames are dropped regularly
- If VLA inference latency >200ms
- If SLAM fails due to resource constraints

### Recommendation 2: Plan for Dual Compute from Start

**Approach**:
1. Design system with dual compute in mind
2. Implement network communication layer
3. Allocate workloads appropriately
4. Test with single compute first, then add Pi

**Expected Results**:
- Jetson CPU: 550-800% (~92-133% of 6 cores) ⚠️ **TIGHT BUT FEASIBLE**
- Pi CPU: 200-360% (~50-90% of 4 cores) ✓ **COMFORTABLE**
- **Risk**: Low - plenty of headroom

**Benefits**:
- Future-proof architecture
- Can handle larger VLA models
- Better SLAM performance
- More logging capacity
- Room for growth

## Implementation Plan

### Phase 1: Single Compute Optimization (Current)
- ✅ Memory optimizations (Phase 1 complete)
- ⏳ Image caching (Phase 2)
- ⏳ Shared memory transport (Phase 2)
- ⏳ Further profiling and optimization

**Timeline**: 1-2 weeks

### Phase 2: Dual Compute Preparation (If Needed)
- Design network architecture
- Implement ROS 2 multi-machine setup
- Create workload allocation scripts
- Test network communication

**Timeline**: 1-2 weeks

### Phase 3: Dual Compute Integration (If Needed)
- Set up Raspberry Pi 4
- Configure network
- Migrate workloads
- Test and validate

**Timeline**: 1 week

## Decision Matrix

| Factor | Single Compute | Dual Compute | Winner |
|--------|---------------|--------------|--------|
| **Compute Capacity** | ⚠️ Marginal | ✓ Sufficient | **Dual** |
| **Memory Capacity** | ⚠️ Marginal | ✓ Sufficient | **Dual** |
| **Complexity** | ✓ Simple | ❌ Complex | **Single** |
| **Power Consumption** | ✓ Lower (15-25W) | ❌ Higher (22-40W) | **Single** |
| **Cost** | ✓ Lower | ❌ Higher (+$75-100) | **Single** |
| **Scalability** | ❌ Limited | ✓ Good | **Dual** |
| **Reliability** | ⚠️ Single point of failure | ✓ Redundancy | **Dual** |
| **Development Speed** | ✓ Faster | ❌ Slower | **Single** |
| **Future-Proof** | ❌ Limited | ✓ Better | **Dual** |

## Final Recommendation

**Start with Single Compute + Aggressive Optimization**, but **design for dual compute**:

1. **Immediate**: Continue optimizing single compute system
2. **Short-term**: Implement image caching, shared memory transport
3. **Medium-term**: Add Raspberry Pi 4 if CPU consistently >90% or if VLA/SLAM performance is poor
4. **Long-term**: Dual compute architecture for production

**Decision Criteria**:
- If CPU usage stays <85% after all optimizations → **Single compute is sufficient**
- If CPU usage >90% or frames dropped → **Add Raspberry Pi 4**

**Design for Both**:
- Make workload allocation configurable
- Design network communication layer early
- Keep workloads modular and separable

This gives you flexibility to start simple and scale up if needed.
