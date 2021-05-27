# CSAFE-BTLE
ESP32 ino for reading data from an exercise bike and reporting it as as power meter

Reads HR and Power data using CSAFE protocol, and exposes it using BTLE for syncing to Zwift/Watches, etc.

My bike didn't implement Cadence for RPM, so I used a wire attached to the ESP32's touch sensor to trigger every time the crank came around
