# ISSKA Datalogger base code with a notecard
Base code for datalogger with a Notecard connected to the TCA3 of the multiplexer. This code was rewritten in august 2023.

The conf.txt file must have the following structure:
```
300; //time step in seconds (put at least 240 seconds)
1; //boolean value to choose if you want to set the RTC clock with GSM time (recommended to put 1)
12; // number of measurements to be sent at once
```

To modify the time step or the number of measurements to send at once, add these command on the data.qi file of the device on notehub:

* {"time_step":300} ->this sets the time set to 300 seconds
* {"nb_meas":6} ->this sets the number of measurements sent at once to 6


If you have any questions contact me per email at nicolas.schmid.6035@gmail.com
