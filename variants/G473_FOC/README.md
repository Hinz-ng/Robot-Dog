# G473 FOC variant

Derived by copying the ESC1 variant from Arduino STM32 **2.8.1**,
PlatformIO package `framework-arduinoststm32@4.20801.240815`, directory
`variants/STM32G4xx/G431C(6-8-B)U_G441CBU`. Upstream copyright notices retained.

| Local file | Upstream file / modification |
|---|---|
| variant_G473_FOC.h | variant_B_G431B_ESC1.h; device/alias guards added, HSE_VALUE 24 -> 8 MHz |
| variant_G473_FOC.cpp | variant_B_G431B_ESC1.cpp, unchanged pin table and HSI/PLL clock setup |
| PeripheralPins.c | PeripheralPins_B_G431B_ESC1.c, unchanged selected AF mappings |
| PinNamesVar.h | unchanged |
| ldscript.ld | unchanged parameterized layout; corrected device/memory description |

The board JSON selects **STM32G473xx** and the framework's G473 startup.
`LD_MAX_SIZE=524288`, `LD_MAX_DATA_SIZE=98304`; ordinary SRAM is
0x20000000..0x20017FFF. CCM is deliberately unused, including its SRAM alias.

`ARDUINO_B_G431B_ESC1` is a compatibility selector, not the device identity.
Audit on 2026-11-08: in SimpleFOC 2.3.1 it enables the three `b_g431` files and
excludes the generic STM32/G4 current-sense implementation. In the copied variant
it enables the pin tables and clock implementation. No uses were found in the
pinned Arduino core, libraries or system tree, or the PlatformIO builder tree.
Only these G473 environments add the alias; no installed package is patched.

R003 shunts, existing analog gain and PCB routing, 8 MHz HSE and MT6835 wiring
are user-authorized assumptions (2026-11-08), not measurements on this board.
The pin tables retain their upstream G431 provenance because they represent
that assumed wiring. See [port procedure](../../docs/G473_PORT.md).
