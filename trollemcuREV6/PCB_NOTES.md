# PCB_NOTES — Required Pre-Fabrication Review

1. **Do not fabricate directly from this generated revision.** Open in KiCad, convert the legacy schematic, update PCB from schematic, refill zones, and run ERC + DRC.
2. **Verify STM32 footprint/orientation** against STM32G474RET6 LQFP64 pin 1 and the selected manufacturer footprint.
3. **ESC/X9C104 is the highest-risk interface.** Measure the original ESC potentiometer terminals with a DMM/oscilloscope. Confirm terminal voltages, resistance range, current, common-mode voltage, and whether the X9C104 can legally/safely replace it. The X9C104 is not an isolated device.
4. The design document simultaneously specifies an X9C104-potentiometer ESC approach and eight motor PWM outputs. Rev C exposes both. Freeze one control method after ESC bench testing.
5. **RCWL-1601 echo protection:** U3 is a 74LVC125A-class buffer powered from 3.3 V. Confirm the exact ordered logic part accepts 5 V on its inputs while powered at 3.3 V. If the sensor's echo is already guaranteed 3.3 V, U3 may be simplified.
6. **HSE crystal:** replace the placeholder crystal/footprint and calculate C11/C12 from the actual crystal CL and board parasitics. Keep the crystal and capacitors close to PF0/PF1 with short, symmetric traces and a quiet ground reference.
7. **TLV73333 thermal/current budget:** account for STM32 + three UWB modules (~102 mA stated in the design) + any other 3.3 V loads. Verify regulator dissipation and capacitor requirements from the exact TI datasheet.
8. **Battery divider:** the present 100 kΩ / 10 kΩ values support up to ~36 V at a 3.3 V ADC full scale. Confirm the selected Aegis battery's maximum charge voltage and transient margin.
9. **Traction power stays off this PCB.** Route battery-to-ESC motor current in appropriately sized wiring, fusing, connectors, and hardware E-stop/contactors. Do not carry four-motor current through controller headers or ground traces.
10. **Grounding:** the controller, sensors, UWB modules, digital-pot modules, and ESC low-voltage control interface need a deliberate common reference unless an isolated interface is used. Keep motor return currents out of the MCU/sensor ground path.
11. **E-stop:** J4 only gives the MCU a normally-closed sense contact. Use another pole/contact for an independent hardware motor disable. Firmware must treat an open J4, broken wire, reset, or invalid sensor state as stop.
12. **HX711:** Rev C powers the module from 3.3 V to keep its digital interface MCU-safe. Confirm the exact SparkFun board functions correctly at 3.3 V with the chosen load-cell excitation and required measurement resolution.
13. **UWB UART wiring:** connector labels are MCU-perspective. MCU TX must connect to module RX and MCU RX to module TX.
14. **Module footprints:** J10-J40 are generic 2.54 mm headers/placeholders. Replace them with the actual mechanical connector/header footprints used in your harness before ordering.
15. **Mechanical:** board outline is approximately 130 x 90 mm with four M3 holes. Confirm enclosure/undercarriage constraints.


## USB-C / DFU additions
- J6 is `Connector_USB:USB_C_Receptacle_GCT_USB4085` and is placed on the top board edge.
- PA11 is now USB_DM and PA12 is now USB_DP. `MOTOR_KILL_CMD` moved to PC4.
- D+/D- are routed directly to the STM32; the STM32G474 includes the required USB full-speed output line impedance, so no external series termination resistors were added.
- CC1 and CC2 each have 5.1 kΩ pull-downs to GND for device/UFP operation.
- USB VBUS is intentionally **not** connected to +5 V. It is only sensed through R10/R11.
- SW2 raises BOOT0 while pressed; use it with RESET to enter the factory USB DFU bootloader.
- Before fabrication, add a low-capacitance USB ESD device close to J6 if the course/lab design rules require external ESD protection, and re-run DRC / differential-pair inspection in KiCad.

All user-specified footprint library IDs have been applied. Because the exact footprints are larger than the earlier placeholders, rerun KiCad DRC and inspect courtyard spacing before fabrication.
