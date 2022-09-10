# PicoW Weather Station
> Simple weather reporting with PicoW

#### Features:
- Measure current temperature & humidity with DHT22 Sensor
- Report measurements to MQTT
- Measure & Report VSYS
- Measure & Report CPU Temp

![Final assembly](https://github.com/Deishelon/pico-w-weather-station/blob/master/assets/final_assembly.jpg?raw=true)

### Install MQTT on PICO W
Make sure you have REPL console.

```python
import network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("Wi-Fi AP", "PASSWORD")

import upip
upip.install('umqtt.simple')
```

### Use
- Upload files onto PICO, adjust params (such as Wi-Fi Credentials // pins // etc ), deploy to your PICO W 
