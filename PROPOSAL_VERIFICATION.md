# Proposal Verification

## Implemented in firmware

| Proposal requirement | Implementation | Status |
| --- | --- | --- |
| STM32F103C8T6 controller | CubeMX project targets STM32F103C8T6 | Complete |
| Ultrasonic level measurement | TIM2 channel 2 generates a 10 us trigger on PA1; PA0 measures echo time | Complete |
| Tank-level percentage | Distance is mapped from 0 cm full to 400 cm empty | Complete |
| Pump ON below 20% | PA2 is asserted when the valid level is below 20% | Complete |
| Pump OFF at 95% | PA2 is cleared at 95% or higher | Complete |
| Emergency shutdown | PA3 uses falling-edge EXTI and immediately clears PA2 | Complete |
| Local OLED display | SSD1306 I2C1 driver uses the full 128x64 framebuffer and displays enlarged level and pump status | Complete |
| USART telemetry | USART1 sends level, distance, and pump status at 115200 baud | Complete as serial telemetry |
| Wokwi demonstration | `diagram.json` connects the sensor, OLED, switch, relay proxy LED, and serial monitor | Complete for demonstrated functions |
| Wireless telemetry | USART output is available, but no ESP-01 or SIM800L is present in the circuit or firmware | Not complete |

## Simulation limitations

- `diagram.json` uses Wokwi's HC-SR04 as the ultrasonic proxy for the proposed JSN-SR04T.
- The red LED is the pump/relay-load proxy; a real 5 V optocoupler relay and pump are not electrically simulated.
- The pushbutton is the emergency float/limit-switch proxy.
- USART1 is connected to Wokwi's serial monitor. ESP-01 or SIM800L cloud transmission still requires the selected wireless module, wiring, and protocol firmware.
- Physical deployment still requires the proposed hardware, level calibration, relay driver safety checks, and a 5 V/3.3 V interface review.

## 394 cm reading

A valid 394 cm echo is accepted as a measurement and transmitted over USART. With the default 400 cm empty calibration it represents approximately 1% level, so the pump turns on below the 20% threshold. `ERROR` is reserved for a missing, timed-out, or physically out-of-range echo.

## OLED improvement

The OLED remains a 128x64 SSD1306 display and now shows only enlarged level percentage and pump state. Distance diagnostics remain available through USART. The panel's physical resolution cannot be increased by firmware.

## Final assessment

The sensing, PWM trigger, level calculation, threshold control, emergency EXTI shutdown, OLED display, serial telemetry, and Wokwi demonstration requirements are implemented. Wireless/cloud telemetry is the only non-optional functional requirement from the proposal that is not yet demonstrated; it needs an ESP-01 or SIM800L and corresponding communication firmware. The JSN-SR04T, relay/pump, and float switch are represented by Wokwi-compatible proxies until physical hardware is connected.