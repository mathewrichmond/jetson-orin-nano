# Mobile Power Design

## Overview

This document outlines the mobile power design for the Isaac robot system, enabling operation without wall power. The system needs to power the Jetson Orin Nano, USB hub/cameras, and servo actuators from the iRobot Create 2's battery.

**Last Updated**: 2026-01-27

**Battery**: Upgraded to 6800mAh Li-ion (Maxofpowr ASMT01-01-107-US, 2.27× standard NiMH capacity)

---

## Current Power Requirements

### Power Budget Summary

| System | Voltage | Current (Peak) | Power (Peak) | Notes |
|--------|---------|----------------|--------------|-------|
| **Jetson Orin Nano** | 19V (or 7-20V) | ~1.2A @ 19V | ~23W | Barrel jack or USB-C |
| **USB Hub + Devices** | 5V | ~5A | ~25W | RealSense cameras, mic, serial |
| **Servo System** | 6V | ~12A | ~72W | 4× MG996R servos via Auto pHAT |
| **Total** | - | - | **~120W** | Peak power consumption |

### Component Details

#### Main System (Jetson + USB Hub)
- **Jetson**: 5W idle, 15W typical, 23W peak @ 19V
- **USB Hub**: 1-2W (hub logic)
- **RealSense D435**: ~2W
- **RealSense D455**: ~3W
- **USB Microphone**: <1W
- **USB-Serial**: <1W
- **Auto pHAT logic**: ~0.3W
- **Total**: ~25W peak @ 5V (if using USB-C) or ~23W @ 19V (barrel jack)

#### Servo System
- **4× MG996R Servos**: 16-24W typical, 72W peak @ 6V
- **Peak current**: 12A @ 6V (3A per servo)
- **Typical current**: 4-6A @ 6V

---

## iRobot Create 2 Battery Specifications

### Battery Details
- **Type**: **Lithium Ion (Li-ion)** - Upgraded capacity replacement battery
- **Model**: Maxofpowr ASMT01-01-107-US
- **Voltage**: 14.4V nominal
- **Capacity**: **6800mAh** (upgraded from standard 3000mAh NiMH)
- **Power**: ~98Wh (14.4V × 6.8Ah)
- **Weight**: 0.6 kg
- **Peak discharge**: Capable of high current (motors draw 5-10A)
- **Note**: Upgraded battery provides **2.27× longer runtime** compared to standard battery

**Li-ion vs NiMH Characteristics**:
- **Voltage range**: Li-ion typically maintains more stable voltage during discharge (e.g., 14.4V → ~12V vs NiMH 14.4V → ~10V)
- **Discharge curve**: More linear voltage drop, better capacity utilization
- **Energy density**: Higher energy density than NiMH (lighter weight for same capacity)
- **Charging**: Requires compatible Li-ion charger (may differ from NiMH charger)
- **Safety**: Li-ion requires protection circuit (should be built into battery pack)
- **Low voltage cutoff**: Should not discharge below ~11V (to protect cells)

### Serial Port Power Limitations

**CRITICAL**: The serial port Vpwr pin (Pin 1) is **NOT suitable** for powering the Jetson:
- **Fuse**: 200mA PTC resettable fuse (continuous)
- **Peak**: 500mA before fuse resets
- **Pin rating**: 0.5A per pin
- **PCB traces**: ~1A capacity
- **Conclusion**: Maximum ~200mA continuous, insufficient for Jetson (~1-2A needed)

**Recommendation**: Do NOT use serial port power pin. Use direct battery connection instead.

---

## Mobile Power Design Options

### Option 1: Direct Battery Connection (Recommended)

**Architecture**: Tap power directly from iRobot Create 2 battery terminals, bypassing serial port limitations.

#### Power Distribution

```
iRobot Create 2 Battery (14.4V nominal, 12-16V range)
│
├─ [Fuse/Circuit Breaker] (10A recommended)
│  │
│  ├─ DC-DC Boost Converter (14.4V → 19V @ 2A)
│  │  └─ Jetson Orin Nano (barrel jack)
│  │     └─ ~23W peak
│  │
│  ├─ DC-DC Buck Converter (14.4V → 5V @ 6A)
│  │  ├─ USB Hub (5V power input)
│  │  └─ ~25W peak
│  │
│  └─ DC-DC Buck Converter (14.4V → 6V @ 12A)
│     └─ Auto pHAT USB-C (servo power)
│        └─ ~72W peak
```

#### Component Requirements

**Boost Converter (14.4V → 19V)**
- **Input**: 12-16V (battery voltage range)
- **Output**: 19V @ 2A (38W)
- **Efficiency**: >85% (to minimize heat)
- **Example**: LM2587-based boost module, or adjustable boost converter
- **Notes**: Jetson accepts 7-20V, so 19V is optimal but 14.4V direct may work (test first)

**Buck Converter 1 (14.4V → 5V)**
- **Input**: 12-16V
- **Output**: 5V @ 6A (30W)
- **Efficiency**: >90%
- **Example**: LM2596-based buck module, or MP2307-based module
- **Notes**: Powers USB hub and all USB devices

**Buck Converter 2 (14.4V → 6V)**
- **Input**: 12-16V
- **Output**: 6V @ 12A (72W)
- **Efficiency**: >85% (high current = more heat)
- **Example**: High-current buck converter (e.g., XL4015-based, 5A+ modules in parallel)
- **Notes**: Servo power via Auto pHAT USB-C

#### Battery Connection Methods

**Method A: External Battery Terminals (Recommended)**
- Access battery terminals directly (requires opening Create 2)
- Install terminal blocks or quick-connect terminals
- Add inline fuse (10A) for protection
- **Pros**: Clean, reliable, no modification to serial port
- **Cons**: Requires access to battery compartment

**Method B: Tap from Internal Power Rails**
- Identify power rails inside Create 2 (after fuse protection)
- Solder connection points
- **Pros**: No battery compartment access needed
- **Cons**: More complex, risk of damage if done incorrectly

**Method C: External Battery Pack**
- Use separate battery pack (e.g., 4S LiPo, 14.8V)
- Mount on robot chassis
- **Pros**: Independent of Create 2 battery, longer runtime
- **Cons**: Additional weight, charging complexity

#### Safety Considerations

1. **Fuse Protection**: Add 10A fuse/breaker on main battery tap
2. **Reverse Polarity Protection**: Use diode or MOSFET protection
3. **Undervoltage Protection**: Monitor battery voltage, shutdown if <11V
4. **Overcurrent Protection**: Each converter should have current limiting
5. **Heat Management**: Buck converters generate heat at high current (especially 6V/12A)
6. **Ground Isolation**: Common ground via Auto pHAT, but verify isolation where needed

---

### Option 2: Separate Battery Pack (Alternative)

**Architecture**: Use dedicated battery pack for Jetson system, independent of Create 2.

#### Power Distribution

```
Dedicated Battery Pack (e.g., 4S LiPo, 14.8V nominal)
│
├─ DC-DC Boost (14.8V → 19V @ 2A) → Jetson
├─ DC-DC Buck (14.8V → 5V @ 6A) → USB Hub
└─ DC-DC Buck (14.8V → 6V @ 12A) → Servos

iRobot Create 2 Battery (independent)
└─ Powers only Create 2 motors/electronics
```

#### Advantages
- **Independent operation**: Jetson system doesn't drain Create 2 battery
- **Longer runtime**: Can use larger capacity battery
- **Isolation**: No risk of affecting Create 2 operation
- **Easier debugging**: Separate power systems

#### Disadvantages
- **Weight**: Additional battery adds weight
- **Charging**: Need separate charger/charging system
- **Cost**: Additional battery and charging equipment
- **Space**: Need mounting location for battery

---

## Recommended Implementation: Option 1 (Direct Battery Connection)

### Phase 1: Basic Setup (Jetson + USB Hub Only)

**Goal**: Get Jetson and USB hub running from Create 2 battery.

**Components Needed**:
1. **Battery tap**: Terminal blocks or quick-connect (10A rated)
2. **Fuse**: 10A automotive fuse + holder
3. **Boost converter**: 14.4V → 19V @ 2A (for Jetson barrel jack)
4. **Buck converter**: 14.4V → 5V @ 6A (for USB hub)

**Wiring**:
```
Create 2 Battery (+)
│
├─ [10A Fuse]
│  │
│  ├─ Boost Converter (14.4V → 19V)
│  │  └─ Jetson barrel jack
│  │
│  └─ Buck Converter (14.4V → 5V)
│     └─ USB hub power input
│
Create 2 Battery (-)
│
└─ Common ground (to both converters)
```

**Testing**:
1. Verify battery voltage (should be 12-16V)
2. Test boost converter output (should be ~19V)
3. Test buck converter output (should be ~5V)
4. Power on Jetson, verify operation
5. Connect USB hub, verify cameras enumerate
6. Monitor voltage/current during operation

### Phase 2: Add Servo Power

**Goal**: Add servo power from same battery.

**Additional Components**:
1. **Buck converter**: 14.4V → 6V @ 12A (high current)
2. **Heat sink**: For high-current buck converter
3. **USB-C connector**: For Auto pHAT power input

**Wiring Addition**:
```
Create 2 Battery (+)
│
├─ [10A Fuse]
│  │
│  ├─ Boost Converter (14.4V → 19V) → Jetson
│  ├─ Buck Converter 1 (14.4V → 5V) → USB Hub
│  └─ Buck Converter 2 (14.4V → 6V) → Auto pHAT USB-C
│
Create 2 Battery (-)
│
└─ Common ground (all converters + Auto pHAT)
```

**Testing**:
1. Verify 6V output under load (servos connected)
2. Monitor current draw during servo movement
3. Check converter temperature (add heat sink if needed)
4. Test all servos simultaneously (peak load)

---

## Component Recommendations

### Boost Converter (14.4V → 19V)

**Option A: Adjustable Boost Module**
- **Model**: LM2587-based adjustable boost converter
- **Specs**: Input 3-40V, Output 1.25-40V adjustable, 3A max
- **Cost**: ~$5-10
- **Notes**: Set output to 19V, verify under load

**Option B: Fixed 19V Boost**
- **Model**: Pre-configured 19V boost module
- **Specs**: Input 12-16V, Output 19V @ 2A
- **Cost**: ~$8-15
- **Notes**: Simpler, but less flexible

### Buck Converter (14.4V → 5V)

**Option A: LM2596 Module**
- **Model**: LM2596-based buck converter
- **Specs**: Input 4-40V, Output 1.25-35V adjustable, 3A max
- **Cost**: ~$2-5
- **Notes**: May need parallel modules for 6A, or use higher-current option

**Option B: MP2307 Module**
- **Model**: MP2307-based buck converter
- **Specs**: Input 4.75-23V, Output 0.925-20V adjustable, 3A max
- **Cost**: ~$3-6
- **Notes**: More efficient than LM2596, but still may need parallel for 6A

**Option C: High-Current Buck (5V)**
- **Model**: XL4015-based or similar
- **Specs**: Input 8-36V, Output 1.25-32V adjustable, 5A max
- **Cost**: ~$5-10
- **Notes**: Single module sufficient for 5V/6A

### Buck Converter (14.4V → 6V, High Current)

**Option A: XL4015 Module (Single)**
- **Model**: XL4015-based buck converter
- **Specs**: Input 8-36V, Output 1.25-32V adjustable, 5A max
- **Cost**: ~$5-10
- **Notes**: May be insufficient for 12A peak, but typical load is 4-6A

**Option B: Parallel XL4015 Modules**
- **Model**: 2× XL4015 modules in parallel
- **Specs**: 10A total capacity
- **Cost**: ~$10-20
- **Notes**: Requires current sharing resistors, more complex

**Option C: High-Current Buck Module**
- **Model**: Dedicated 10A+ buck converter
- **Specs**: Input 12-40V, Output 6V @ 12A
- **Cost**: ~$15-30
- **Notes**: Professional solution, better heat management

**Option D: Use 5V for Servos**
- **Model**: Single XL4015 set to 5V
- **Specs**: 5V @ 5A (25W)
- **Cost**: ~$5-10
- **Notes**: Servos work at 5V (lower torque: 10 kg·cm vs 12 kg·cm @ 6V), simpler

### Protection Components

**Fuse**:
- **Type**: Automotive blade fuse, 10A
- **Holder**: Inline fuse holder
- **Location**: Between battery positive and converters

**Reverse Polarity Protection**:
- **Option**: Schottky diode (10A, low forward voltage drop)
- **Location**: After fuse, before converters
- **Notes**: Adds ~0.3V drop, may need to account for in converter input

**Undervoltage Protection**:
- **Option**: Battery monitor circuit or software monitoring
- **Function**: Shutdown if battery <11V (**critical for Li-ion** - prevents cell damage)
- **Notes**: Can be implemented in power management node
- **Li-ion specific**: Never discharge below 11V (2.75V per cell in 4S configuration)
- **Battery protection**: Built-in protection circuit should prevent overdischarge, but add software monitoring as backup

---

## Serial-to-USB Cable Considerations

### Current Setup
- **Issue**: Serial-to-USB cable is too long for mobile build
- **Solution**: Use shorter USB cable or custom cable

### Options

**Option A: Shorter USB Cable**
- Use standard USB-A to USB-A cable (1-2ft)
- Connect USB-to-serial adapter to shorter cable
- **Pros**: Simple, no modification needed
- **Cons**: Still have adapter + cable

**Option B: Custom Serial Cable**
- Create custom cable with USB connector on one end, serial connector on other
- Use shorter USB cable (6-12 inches)
- **Pros**: Cleaner, shorter overall length
- **Cons**: Requires custom cable assembly

**Option C: Direct Serial Connection**
- If Jetson has UART pins available, connect directly
- Use level shifter if needed (Create 2 uses 5V logic)
- **Pros**: No USB adapter needed, shorter path
- **Cons**: Requires GPIO/UART pins, may conflict with Auto pHAT

**Recommendation**: Option A (shorter USB cable) is simplest and sufficient.

---

## Power Management Software

### Battery Monitoring

The existing `battery_monitor_node.py` can be extended to monitor Create 2 battery:

**Additions Needed**:
1. **Voltage monitoring**: Read battery voltage via ADC or iRobot OI protocol
2. **Current monitoring**: Optional, via current sensor
3. **Low battery shutdown**: Trigger graceful shutdown if voltage <11V
4. **Power state publishing**: Publish battery state to `/power/battery_state`

**iRobot Battery Info**:
- Battery capacity available via OI protocol (0-16000 units)
- Voltage can be estimated from capacity, or measured directly
- Low battery threshold: ~20% capacity or <12V

### Power Sequencing

**Startup**:
1. Create 2 battery powers converters
2. Converters power Jetson, USB hub, servos
3. Jetson boots, enumerates USB devices
4. ROS 2 nodes start, initialize hardware

**Shutdown**:
1. ROS 2 nodes stop (servos move to safe position)
2. Jetson graceful shutdown
3. Converters remain powered (minimal current draw)
4. Manual power switch or low battery shutdown

---

## Testing & Validation

### Phase 1 Testing (Jetson + USB Hub)

1. **Voltage Verification**:
   - Measure battery voltage (should be 12-16V)
   - Measure boost converter output (should be 19V ±5%)
   - Measure buck converter output (should be 5V ±5%)

2. **Current Draw**:
   - Measure Jetson current (should be 0.5-1.2A @ 19V)
   - Measure USB hub current (should be 1-5A @ 5V)
   - Verify total <10A from battery

3. **Runtime Test**:
   - Run system for extended period
   - Monitor battery voltage over time
   - Verify stable operation

### Phase 2 Testing (With Servos)

1. **Servo Power**:
   - Measure 6V converter output under load
   - Verify current capacity (4-6A typical, 12A peak)
   - Check converter temperature (add heat sink if >60°C)

2. **Peak Load Test**:
   - All servos moving simultaneously
   - Cameras streaming
   - Jetson under load
   - Verify battery voltage stays >11V

3. **Runtime Test**:
   - Full system operation
   - Monitor battery capacity via iRobot OI
   - Estimate runtime based on power consumption

---

## Estimated Runtime

### Power Consumption Estimates

**Typical Operation** (not peak):
- Jetson: ~12W @ 19V = 0.63A @ 19V = 0.88A @ 14.4V
- USB Hub + Devices: ~10W @ 5V = 2A @ 5V = 0.69A @ 14.4V
- Servos: ~20W @ 6V = 3.3A @ 6V = 1.38A @ 14.4V
- **Total**: ~42W = ~2.95A @ 14.4V

**Peak Operation**:
- Jetson: ~23W = 1.6A @ 14.4V
- USB Hub + Devices: ~25W = 1.74A @ 14.4V
- Servos: ~72W = 5A @ 14.4V
- **Total**: ~120W = ~8.3A @ 14.4V

### Runtime Calculation

**Battery Capacity**: **6800mAh @ 14.4V** (upgraded battery)

**Typical Operation** (2.95A):
- Runtime: 6800mAh / 2950mA = **~2.3 hours** (up from ~1 hour)

**Peak Operation** (8.3A):
- Runtime: 6800mAh / 8300mA = **~49 minutes** (up from ~22 minutes)

**Mixed Operation** (average 4A):
- Runtime: 6800mAh / 4000mA = **~1.7 hours** (up from ~45 minutes)

**Conservative Estimate** (accounting for efficiency losses, ~85% usable):
- Typical: **~2 hours** continuous operation
- Mixed: **~1.4 hours** continuous operation
- Peak: **~40 minutes** continuous operation

**Li-ion Battery Advantages**:
- **Better voltage stability**: Li-ion maintains higher voltage longer during discharge, improving DC-DC converter efficiency
- **More usable capacity**: Flatter discharge curve means more energy available before low-voltage cutoff
- **Lighter weight**: 0.6 kg vs ~1.2 kg for equivalent NiMH capacity
- **Better low-temperature performance**: Li-ion performs better in cold conditions

**Note**: Actual runtime depends on:
- Battery age/condition
- Servo usage (idle vs active)
- Jetson workload (idle vs processing)
- Battery voltage cutoff (**11V minimum for Li-ion** - critical to prevent cell damage)
- DC-DC converter efficiency (~85-90%)
- Battery discharge curve (Li-ion has flatter curve than NiMH, better capacity utilization)
- Temperature (Li-ion performance degrades below 0°C and above 45°C)

---

## Next Steps

1. **Battery Installation**:
   - Verify battery fits in Create 2 battery compartment
   - Check that Create 2 charger is compatible with Li-ion (may need Li-ion-specific charger)
   - **Important**: Li-ion batteries require different charging profile than NiMH
   - Verify battery has built-in protection circuit (overcharge, overdischarge, short circuit protection)
   - Test initial charge/discharge cycle

2. **Order Components**:
   - Boost converter (14.4V → 19V)
   - Buck converter (14.4V → 5V, 6A)
   - Buck converter (14.4V → 6V, 12A) or use 5V option
   - Fuse and holder (10A)
   - Terminal blocks or quick-connect terminals
   - Heat sinks for high-current converters

2. **Power Module Design** (See [POWER_MODULE_DESIGN.md](POWER_MODULE_DESIGN.md)):
   - Design custom power module PCB
   - Select specific components (converters, connectors, protection)
   - Create schematic and PCB layout
   - Order PCB and components

3. **Battery Access**:
   - Determine method to access Create 2 battery terminals
   - Install terminal blocks or connection points
   - Add fuse protection

4. **Assembly**:
   - Assemble power module PCB
   - Wire converters to battery
   - Connect outputs to Jetson, USB hub, servos
   - Verify all connections, add strain relief

5. **Testing**:
   - Phase 1: Bench testing (no load, then with loads)
   - Phase 2: Integration testing (Jetson + USB hub only)
   - Phase 3: Full system testing (add servo power)
   - Validate runtime and stability

6. **Software Updates**:
   - Extend battery monitoring for Create 2 battery
   - Add low battery shutdown logic
   - Test power sequencing

7. **Documentation**:
   - Document final wiring diagram
   - Update BOM with selected components
   - Add photos of installation

---

## References

- [ELECTRICAL.md](ELECTRICAL.md) - Current electrical specifications
- [irobot.md](irobot.md) - iRobot Create 2 documentation
- [sparkfun_auto_phat.md](sparkfun_auto_phat.md) - Auto pHAT power requirements
- [Jetson Orin Nano Datasheet](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide)
