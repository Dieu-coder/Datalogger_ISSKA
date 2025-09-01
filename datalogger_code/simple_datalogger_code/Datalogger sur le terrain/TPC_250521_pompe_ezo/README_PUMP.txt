README: Pump configuration (EN) 


Purpose
- This file explains the pump configuration stored in `pump.txt`.

`pump.txt` format (machine-readable)
- Lines 1..3: three flags, each is `0` or `1` and must be followed by a semicolon (`;`) and a newline.
   1) First line: `1;` load injection data from this file, `0;` use values stored in EEPROM.
   2) Second line: `1;` set water-height offset from current sensor reading and store it in EEPROM, `0;` do not set offset.
   3) Third line: `1;` fill tubes on boot (device will run a fill command), `0;` do not fill.


- After the three flag lines, list dose configuration lines, one per line, format:
      lower-upper,dose,numberOfInjections
   Example dose lines:
      10-20,5,2
      30-40,10,1
When you first install the pump on the field, follow the following procedure:
turn the first three lines to one, and wait that everything configures successfully and the device falls back in deepsleep. This ensures that:
 -1 the number of injections is initialised in the eeprom.
 -2 the device sets the offset for water height difference.
 -3 the device fills the tubes with tracer to ensure correct injections.

After that, PUT THOSE 3 LINES BACK TO 0, this is important, because it ensures that the pump will not reset the available number of injections and the offset if there is a reboot, and that there will not be accidental fill up injections.

Notes & behavior

- After editing `/pump.txt`, reboot the device to apply changes.
- When `Setoffset` is enabled, the measured water height is saved to EEPROM address 0.
- Pump logging is merged into `/data.csv`. There is no longer a separate `/pumpdata.csv`.

Also, do not forget to check that the device clock is correctly set up. This can be changed with the conf.txt file.