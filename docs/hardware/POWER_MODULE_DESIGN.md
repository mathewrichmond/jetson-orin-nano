# Power Module Design

## Overview

This document describes the custom power module design for mobile robot operation. The power module converts the 14.4V Li-ion battery voltage to the three required voltages:
- **19V @ 2A** (Jetson barrel jack)
- **5V @ 6A** (USB hub power)
- **6V @ 12A** (servo power via Auto pHAT USB-C)

**Last Updated**: 2026-01-27

**Status**: Design Phase

---

## System Architecture

```
14.4V Li-ion Battery (Maxofpowr ASMT01-01-107-US, 6800mAh)
│
├─ [10A Fuse] (protection)
│  │
│  ├─ Boost Converter (14.4V → 19V @ 2A)
│  │  └─ Barrel Jack Output → Jetson Orin Nano
│  │     (~38W peak, ~23W typical)
│  │
│  ├─ Buck Converter 1 (14.4V → 5V @ 6A)
│  │  └─ USB Hub Power Input
│  │     (~30W peak, ~10W typical)
│  │
│  └─ Buck Converter 2 (14.4V → 6V @ 12A)
│     └─ USB-C Output → Auto pHAT USB-C
│        (~72W peak, ~20W typical)
│
└─ Common Ground (all outputs)
```

---

## Design Requirements

### Input Specifications
- **Voltage**: 12-16V (14.4V nominal, Li-ion battery range)
- **Current**: Up to 10A peak (fused)
- **Connector**: Terminal block or quick-connect (battery tap)

### Output Specifications

| Output | Voltage | Current (Peak) | Power (Peak) | Connector | Load |
|--------|---------|----------------|-------------|-----------|------|
| **Jetson** | 19V | 2A | 38W | Barrel jack (5.5×2.1mm) | Jetson Orin Nano |
| **USB Hub** | 5V | 6A | 30W | Barrel jack or terminal | RSHTECH USB hub |
| **Servos** | 6V | 12A | 72W | USB-C female | Auto pHAT USB-C input |

### Performance Requirements
- **Efficiency**: >85% for all converters (minimize heat)
- **Regulation**: ±5% voltage tolerance
- **Ripple**: <100mV peak-to-peak
- **Temperature**: Operate up to 60°C ambient
- **Protection**: Overcurrent, reverse polarity, undervoltage

---

## Component Selection

### Option A: Discrete Converter Modules (Recommended for Prototype)

**Advantages**: Faster to prototype, easier to debug, can test individually

#### Boost Converter (14.4V → 19V)

**Module Option**: Adjustable Boost Converter Module
- **IC**: LM2587 or TPS61088-based
- **Input**: 3-40V
- **Output**: 1.25-40V adjustable
- **Current**: 3A max
- **Efficiency**: ~85-90%
- **Example**: LM2587 adjustable boost module (eBay/AliExpress, ~$5-10)
- **Adjustment**: Set output to 19V using trim pot

**Alternative**: Fixed 19V Boost Module
- Pre-configured 19V output
- Simpler (no adjustment needed)
- Less flexible

#### Buck Converter 1 (14.4V → 5V)

**Module Option**: XL4015 Buck Converter Module
- **IC**: XL4015
- **Input**: 8-36V
- **Output**: 1.25-32V adjustable
- **Current**: 5A max
- **Efficiency**: ~90-95%
- **Example**: XL4015 adjustable buck module (~$3-6)
- **Adjustment**: Set output to 5V using trim pot

**Alternative**: MP2307 or LM2596 modules (3A max, may need parallel for 6A)

#### Buck Converter 2 (14.4V → 6V, High Current)

**Module Option**: XL4015 Buck Converter Module (Single)
- **IC**: XL4015
- **Input**: 8-36V
- **Output**: 1.25-32V adjustable
- **Current**: 5A max
- **Efficiency**: ~85-90%
- **Note**: May be insufficient for 12A peak, but typical load is 4-6A
- **Example**: XL4015 adjustable buck module (~$3-6)

**Alternative**: Parallel XL4015 Modules
- 2× XL4015 modules in parallel
- Requires current sharing resistors
- More complex, but provides 10A capacity
- **Cost**: ~$6-12

**Alternative**: High-Current Buck Module
- Dedicated 10A+ buck converter module
- Professional solution, better heat management
- **Cost**: ~$15-30

**Alternative**: Use 5V for Servos
- Single XL4015 set to 5V
- Servos work at 5V (lower torque: 10 kg·cm vs 12 kg·cm @ 6V)
- Simpler, sufficient for most applications
- **Cost**: ~$3-6

### Option B: Discrete IC Design (For Custom PCB)

**Advantages**: Optimized layout, better integration, lower cost at scale

#### Boost Converter IC (19V)
- **LM2587**: 3A boost converter, adjustable
- **TPS61088**: 3A boost converter, higher efficiency
- **Package**: TO-220 or SOIC (hand-solderable)

#### Buck Converter ICs (5V and 6V)
- **MP2307**: 3A buck converter, high efficiency
- **LM2596**: 3A buck converter, common and reliable
- **XL4015**: 5A buck converter (for 6V output)
- **Package**: TO-220 or SOIC (hand-solderable)

**Note**: Option B requires custom PCB design. Option A (modules) is recommended for initial prototype.

---

## Protection Components

### Fuse Protection
- **Type**: Automotive blade fuse, 10A
- **Holder**: Inline fuse holder
- **Location**: Between battery positive input and converters
- **Purpose**: Protect against overcurrent/short circuit

### Reverse Polarity Protection
- **Option 1**: Schottky diode (10A, low forward voltage drop)
  - **Part**: SB1040 or similar (10A, ~0.5V drop)
  - **Location**: After fuse, before converters
  - **Note**: Adds ~0.5V drop, may need to account for in converter input

- **Option 2**: P-channel MOSFET (lower voltage drop)
  - **Part**: IRF9540N or similar
  - **Location**: After fuse, before converters
  - **Advantage**: Lower voltage drop (~0.1V vs 0.5V)
  - **Complexity**: Requires gate drive circuit

**Recommendation**: Start with Schottky diode (simpler), upgrade to MOSFET if voltage drop is critical.

### Undervoltage Protection
- **Option**: Battery monitor circuit or software monitoring
- **Function**: Shutdown if battery <11V (critical for Li-ion)
- **Implementation**:
  - Hardware: Undervoltage lockout IC (e.g., MAX809)
  - Software: Monitor via iRobot OI protocol or ADC
- **Note**: Can be implemented in power management node initially

### Overcurrent Protection
- **Built-in**: Most converter modules have current limiting
- **Additional**: Fuse provides backup protection
- **Monitoring**: Optional current sense resistors + ADC

---

## Connector Selection

### Battery Input
- **Option 1**: Terminal Block (5mm pitch)
  - **Part**: 2-position terminal block
  - **Advantage**: Secure, easy to connect/disconnect
  - **Example**: Phoenix Contact or generic 5mm terminal block

- **Option 2**: Quick-Connect Terminals
  - **Part**: 0.25" quick-connect (automotive style)
  - **Advantage**: Fast connection, common in automotive
  - **Example**: Molex or generic quick-connect terminals

- **Option 3**: XT60 Connector
  - **Part**: XT60 (common in RC/robotics)
  - **Advantage**: Locking, high current capacity
  - **Example**: Generic XT60 connector pair

**Recommendation**: Terminal block for prototype (easiest), XT60 for production (more robust).

### Output Connectors

#### Jetson Power (19V)
- **Type**: Barrel jack (5.5×2.1mm, center positive)
- **Part**: DC power jack, panel mount
- **Example**: Generic 5.5×2.1mm barrel jack

#### USB Hub Power (5V)
- **Option 1**: Barrel jack (match USB hub input)
- **Option 2**: Terminal block (if hub has terminal input)
- **Option 3**: USB-A connector (if hub accepts USB power input)
- **Note**: Check USB hub power input connector type

#### Servo Power (6V)
- **Type**: USB-C female connector
- **Part**: USB-C receptacle, through-hole
- **Example**: Generic USB-C receptacle (16-pin, through-hole)
- **Note**: Must support power-only (no data lines needed)

### Ground Connections
- **Common Ground**: All outputs share common ground
- **Connector**: Terminal block or multiple ground terminals
- **Note**: Ensure proper ground connection to Auto pHAT

---

## PCB Design Considerations

### PCB Specifications
- **Layers**: 2-layer (sufficient for power module)
- **Thickness**: 1.6mm (standard)
- **Copper Weight**: 2oz (for high current traces)
- **Size**: ~80×60mm (estimate, depends on module sizes)
- **Mounting**: 4× M3 mounting holes

### Power Trace Sizing

**Battery Input (10A)**:
- **Trace Width**: 200 mils (5mm) minimum
- **Via Count**: Minimize (use multiple vias if needed)
- **Copper Weight**: 2oz recommended

**19V Output (2A)**:
- **Trace Width**: 50 mils (1.3mm) minimum
- **Standard**: 100 mils (2.5mm) for safety margin

**5V Output (6A)**:
- **Trace Width**: 150 mils (3.8mm) minimum
- **Standard**: 200 mils (5mm) for safety margin

**6V Output (12A)**:
- **Trace Width**: 300 mils (7.6mm) minimum
- **Standard**: 400 mils (10mm) for safety margin
- **Via Count**: Use multiple vias (2-3) for high current

### Component Placement
- **Input**: Battery connector on one edge
- **Converters**: Space for heat dissipation (especially 6V buck)
- **Outputs**: Connectors on opposite edge
- **Fuse**: Accessible for replacement
- **Heat Sinks**: Space for heat sinks on high-current converters

### Thermal Management
- **Heat Sinks**: Required for 6V buck converter (12A)
- **Ventilation**: Ensure airflow around converters
- **Thermal Vias**: Under converter ICs (if using discrete design)
- **Spacing**: Minimum 5mm between converters

### Safety Features
- **Silkscreen**: Clear labels for all connectors
- **Polarity Marking**: Mark + and - on all connectors
- **Fuse Label**: Mark fuse rating and type
- **Warning Labels**: High voltage/current warnings

---

## Bill of Materials (BOM)

### Power Converters

| Item | Description | Qty | Part Number | Cost | Notes |
|------|-------------|-----|-------------|------|-------|
| Boost Module | 14.4V → 19V, 3A | 1 | LM2587 module | $5-10 | Adjustable output |
| Buck Module 1 | 14.4V → 5V, 5A | 1 | XL4015 module | $3-6 | Adjustable output |
| Buck Module 2 | 14.4V → 6V, 5A | 1 | XL4015 module | $3-6 | Or use 5V option |
| Heat Sink | For 6V buck | 1 | Generic TO-220 | $2-5 | Optional but recommended |

### Protection Components

| Item | Description | Qty | Part Number | Cost | Notes |
|------|-------------|-----|-------------|------|-------|
| Fuse | 10A blade fuse | 1 | Generic automotive | $1-2 | Standard size |
| Fuse Holder | Inline fuse holder | 1 | Generic automotive | $2-4 | - |
| Diode | Schottky 10A | 1 | SB1040 or similar | $2-5 | Reverse polarity protection |

### Connectors

| Item | Description | Qty | Part Number | Cost | Notes |
|------|-------------|-----|-------------|------|-------|
| Battery Input | Terminal block 2-pos | 1 | 5mm pitch | $2-4 | Or XT60 |
| Barrel Jack 19V | 5.5×2.1mm panel mount | 1 | Generic | $1-2 | Jetson power |
| Barrel Jack 5V | Match USB hub input | 1 | Generic | $1-2 | USB hub power |
| USB-C Receptacle | 16-pin through-hole | 1 | Generic | $2-4 | Servo power |

### PCB and Assembly

| Item | Description | Qty | Cost | Notes |
|------|-------------|-----|------|-------|
| PCB | 2-layer, 80×60mm | 1 | $50-100 | Small batch (5-10 boards) |
| Mounting Hardware | M3 screws, standoffs | 4 | $2-5 | - |
| Enclosure | Optional | 1 | $10-20 | For protection |

### Total Cost Estimate
- **Components**: $25-50
- **PCB**: $50-100
- **Assembly**: DIY (hand-solderable)
- **Total**: **$75-150** (per unit, first prototype)

---

## Assembly Instructions

### Step 1: PCB Preparation
1. Order PCB from manufacturer (JLCPCB, PCBWay, or similar)
2. Verify PCB dimensions and hole locations
3. Check all traces and connections

### Step 2: Component Placement
1. **Place converters**: Mount converter modules on PCB
   - Use standoffs if modules have mounting holes
   - Ensure proper spacing for heat dissipation
2. **Place connectors**: Mount all connectors on PCB edges
3. **Place protection**: Install fuse holder and diode

### Step 3: Wiring
1. **Battery Input**:
   - Connect battery positive → fuse → diode → converter inputs
   - Connect battery negative → common ground
2. **Converter Outputs**:
   - Connect boost output → 19V barrel jack
   - Connect buck 1 output → 5V barrel jack/terminal
   - Connect buck 2 output → USB-C receptacle
3. **Grounds**: Connect all grounds together (common ground)

### Step 4: Adjustment
1. **Boost Converter**: Adjust trim pot to 19V output (measure with multimeter)
2. **Buck Converter 1**: Adjust trim pot to 5V output
3. **Buck Converter 2**: Adjust trim pot to 6V output (or 5V if using 5V option)

### Step 5: Testing
1. **No-Load Test**: Measure all outputs (should be correct voltages)
2. **Load Test**: Connect loads and verify voltage stability
3. **Current Test**: Measure current draw under load
4. **Thermal Test**: Check converter temperatures (should be <60°C)

---

## Testing Procedures

### Phase 1: Bench Testing (No Load)

**Equipment Needed**:
- Multimeter
- Variable power supply (12-16V) or battery

**Procedure**:
1. Connect power supply to battery input (start with 14.4V)
2. Measure all outputs with multimeter:
   - 19V output: Should read 19V ±5%
   - 5V output: Should read 5V ±5%
   - 6V output: Should read 6V ±5%
3. Verify fuse doesn't blow (should be no load)
4. Check for any shorts or incorrect connections

**Expected Results**:
- All outputs within ±5% of target voltage
- No excessive heat
- No fuse blowing

### Phase 2: Load Testing

**Equipment Needed**:
- Electronic load or resistors
- Multimeter
- Current probe or multimeter with current measurement

**Procedure**:
1. **19V Output**:
   - Connect load (resistor or electronic load)
   - Measure voltage and current
   - Verify voltage stays within ±5% under load
   - Test up to 2A load
2. **5V Output**:
   - Connect load
   - Measure voltage and current
   - Verify voltage stability
   - Test up to 6A load
3. **6V Output**:
   - Connect load
   - Measure voltage and current
   - Verify voltage stability
   - Test up to 12A load (or 5A if single module)

**Expected Results**:
- Voltage regulation within ±5% under load
- No excessive voltage drop
- Converters stay cool (<60°C)

### Phase 3: Integration Testing

**Equipment Needed**:
- Complete robot system
- Battery (14.4V Li-ion)

**Procedure**:
1. **Connect to Battery**:
   - Tap battery terminals (with fuse protection)
   - Connect to power module input
2. **Connect Outputs**:
   - 19V → Jetson barrel jack
   - 5V → USB hub power input
   - 6V → Auto pHAT USB-C
3. **Power On**:
   - Verify Jetson boots
   - Verify USB hub powers cameras
   - Verify servos receive power
4. **Runtime Test**:
   - Run system for extended period
   - Monitor battery voltage
   - Check converter temperatures
   - Measure power consumption

**Expected Results**:
- System operates normally
- Battery voltage stays >11V
- Converters stay cool
- No voltage drops or instability

---

## Safety Considerations

### Electrical Safety
1. **Fuse Protection**: Always use fuse on battery input (10A)
2. **Reverse Polarity**: Diode or MOSFET protection required
3. **Undervoltage**: Monitor battery voltage, shutdown at 11V (Li-ion)
4. **Overcurrent**: Converters should have current limiting
5. **Short Circuit**: Fuse provides protection

### Thermal Safety
1. **Heat Sinks**: Use heat sinks on high-current converters (especially 6V)
2. **Ventilation**: Ensure airflow around converters
3. **Temperature Monitoring**: Check converter temperatures during operation
4. **Shutdown**: If converters exceed 80°C, reduce load or improve cooling

### Mechanical Safety
1. **Mounting**: Secure PCB to prevent movement/vibration
2. **Strain Relief**: Secure all cables to prevent pull-out
3. **Enclosure**: Consider enclosure for protection (optional)
4. **Labels**: Clear labels for all connectors and warnings

### Battery Safety
1. **Li-ion Protection**: Never discharge below 11V
2. **Charging**: Use compatible Li-ion charger
3. **Storage**: Store battery properly when not in use
4. **Monitoring**: Monitor battery voltage during operation

---

## Troubleshooting

### No Output Voltage
- **Check**: Fuse is not blown
- **Check**: Battery input voltage (should be 12-16V)
- **Check**: Reverse polarity protection (diode direction)
- **Check**: Converter connections

### Incorrect Output Voltage
- **Check**: Converter trim pot adjustment
- **Check**: Input voltage (affects output)
- **Check**: Load (too high load can cause voltage drop)
- **Adjust**: Re-adjust trim pot with multimeter

### Excessive Heat
- **Check**: Load current (may be too high)
- **Check**: Heat sink installation (6V converter)
- **Check**: Ventilation (ensure airflow)
- **Solution**: Reduce load or improve cooling

### Voltage Drops Under Load
- **Check**: Trace width (may be too narrow)
- **Check**: Connector resistance (may be high)
- **Check**: Converter current capacity (may be insufficient)
- **Solution**: Use larger traces or higher-capacity converter

### Fuse Blowing
- **Check**: Short circuit (check all connections)
- **Check**: Overcurrent (reduce load)
- **Check**: Fuse rating (should be 10A)
- **Solution**: Fix short or reduce load

---

## Future Enhancements

### Optional Features
1. **Voltage Monitoring**: ADC to monitor battery voltage
2. **Current Monitoring**: Current sense resistors + ADC
3. **Power Switches**: Enable/disable individual outputs
4. **LED Indicators**: Status LEDs for each output
5. **Undervoltage Lockout**: Hardware shutdown at 11V

### Integration Options
1. **Combine with USB Hub**: Option 2 from design analysis
2. **Combine with pHAT**: Option 3 from design analysis
3. **Custom Enclosure**: 3D printed or custom enclosure

---

## References

- [MOBILE_POWER_DESIGN.md](MOBILE_POWER_DESIGN.md) - Overall mobile power design
- [ELECTRICAL.md](ELECTRICAL.md) - Electrical specifications
- [BOM.md](BOM.md) - Bill of materials

### External Resources
- LM2587 Datasheet: Texas Instruments
- XL4015 Datasheet: XLSEMI
- USB-C Connector Spec: USB-IF
- PCB Design Guidelines: Various manufacturers (JLCPCB, PCBWay)

---

## Next Steps

1. **Finalize Component Selection**: Choose specific part numbers
2. **Create Schematic**: Draw schematic diagram (KiCad, Eagle, or similar)
3. **Design PCB Layout**: Create PCB layout with proper trace sizing
4. **Order Components**: Purchase converters, connectors, protection components
5. **Order PCB**: Send PCB design to manufacturer
6. **Assemble Prototype**: Hand-assemble first prototype
7. **Test**: Run through testing procedures
8. **Iterate**: Fix any issues and create revision 2 if needed

---

**Status**: Ready for schematic design and PCB layout
