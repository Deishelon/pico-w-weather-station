import network
import rp2
import time
import machine
from machine import Pin
from umqtt.simple import MQTTClient
import dht
import urequests
import wifi_config
import mqtt_config

# Change this GPIO PIN where your DHT22 sensor is connected
DHT_22_GPIO_PIN = 28

def read_cpu_temp():
    """
        If you print the value of the temperature value you are going to get an integer number between 0 and 65535.
        So, we have to convert this value either to the Celsius degree scales.

        The temperature sensor works by delivering a voltage to the ADC4 pin that is proportional to the temperature.
        From the datasheet, a temperature of 27 degrees Celsius delivers a voltage of 0.706 V.
        With each additional degree the voltage reduces by 1.721 mV or 0.001721 V.
        The first step in converting the 16-bit temperature is to convert it back to volts, which is done based on the 3.3 V maximum voltage used by the Pico board.
        ref: https://how2electronics.com/read-temperature-sensor-value-from-raspberry-pi-pico/
    """
    cpu_temp_conversion_factor = 3.3 / 65535
    cpu_temp_sensor = machine.ADC(4)
    reading = cpu_temp_sensor.read_u16() * cpu_temp_conversion_factor
    temperature_c = 27 - (reading - 0.706) / 0.001721
    return temperature_c

def read_vsys():
    # Set pin 29 for the voltage reading
    # Note, this will break Wi-Fi until the pin is set back to original state
    # See lines below where the state is reset
    Pin(29, Pin.IN)
    Vsys = machine.ADC(3)
    conversion_factor = 3.3 * 3 / 65535
    reading = Vsys.read_u16() * conversion_factor

    # Reset pin back to the original state
    # This original state is found by printing the value of the pin at the start of this function
    # alt=7 seems to be Wi-Fi related, so
    Pin(29, Pin.ALT, Pin.PULL_DOWN, alt=7)

    return reading

def read_dht_22_raw(sensor):
    """
        reads the temperature and humidity from dht.DHT22 sensor.
        returns tuple(temperature, humidity) if no errors
        returns None if there was an error
    """
    try:
        sensor.measure()
        temperature = sensor.temperature()
        humidity = sensor.humidity()
        return temperature, humidity
    except OSError:
        return None
    
def read_dht_22_with_retry(sensor):
    """Same as [read_dht_22_raw] but tries a few times before giving up. Same returns as [read_dht_22_raw]"""
    count = 0
    while count < 2:
        reading = read_dht_22_raw(sensor)
        count += 1
        if reading is not None:
            return reading
        time.sleep(2)
    return None
    
def read_dht_22(sensor):
    """
        When DHT22 runs on 3.3v sometimes the output results are incomplete, at least what I've seen before
        i.e it can return 2deg, 0deg for a measurement, and the normal readings
        This is a hack to see if this solves this problem, we take 2 measurements and if they are not same (or close), discard
    """
    reading_1 = read_dht_22_with_retry(sensor)
    
    if reading_1 is None:
        print("read_dht_22, reading 1 is None. Abort")
        return None
    # print("Reading 1: {}".format(reading_1))
    
    time.sleep(2)
    
    reading_2 = read_dht_22_with_retry(sensor)
    
    if reading_2 is None:
        print("read_dht_22, reading 2 is None. Abort")
        return None
    
    # print("Reading 2: {}".format(reading_2))
    
    diff = abs(reading_1[0] - reading_2[0])
    #print("Reading between 1 and 2 is {}".format(diff))
    if diff > 2:
        print("Reading between 1 and 2 is more than 2 deg apart, {}".format(diff))
        return None
    
    return reading_1

def wlan_up(wlan):
    print("Connoting to Wifi...")
    wlan.active(True)
    print("Wifi chip is active ... wlan.connect now")
    wlan.connect(wifi_config.HOME_WIFI_SSID, wifi_config.HOME_WIFI_PWD)
    print("wlan.connect is done")
    
    # Wait for connect or fail
    max_wait = 10
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
    max_wait -= 1
    print('waiting for connection...')
    time.sleep(1)

    if wlan.status() != 3:
        raise RuntimeError('network connection failed, {}'.format(wlan.status()))
    
    ifconfig = wlan.ifconfig()
    print(ifconfig)
    print("Conneted to Wifi")
    return ifconfig
    
def led_error_code(led, error_code: int):
    """Blink LED for a given error code (int). error code == number of times to blink"""
    print("LED Error Status code: {}".format(error_code))
    
    # Run a quick 'start error code sequence'
    # So we know when LED error sequence starts
    start_sequence_counter = 0
    while start_sequence_counter < 3:
        led.value(True)
        time.sleep(0.1)
        led.value(False)
        time.sleep(0.1)
        start_sequence_counter += 1
    
    # Run real error code sequence
    blink_counter = 0
    while blink_counter < error_code:
        time.sleep(1)
        led.value(True)
        time.sleep(1)
        led.value(False)
        blink_counter += 1
    # Make sure to turn off LED when this subroutine finished
    led.value(False)
    print("LED Error Status code finished for: {}".format(error_code))
    
    
def main():
    print("Start up")

    # Set WiFi Country
    rp2.country('NZ')
    wlan = network.WLAN(network.STA_IF)
    
    mqtt_client = MQTTClient(mqtt_config.MQTT_CLIENT_ID, mqtt_config.MQTT_HOST_NAME)

    sensor = dht.DHT22(Pin(DHT_22_GPIO_PIN))
    
    led = machine.Pin('LED', machine.Pin.OUT)
    led.value(False)
    
    led_error_code(led, 1)

    print("Enter main loop")

    while True:
        led.value(False)
        try:
            ifconfig = wlan_up(wlan)
        except Exception as e:
            print("Trouble to connecting WiFi: {}".format(e))
            # Should we raise a problem vs just try it again ?
            # raise RuntimeError('Trouble to connecting WiFi, {}'.format(e))
            led_error_code(led, 3)
            continue
        
        try:
            mqtt_client.connect()
            print("Connected to MQTT")
        except Exception as e:
            print("Trouble to connecting to MQTT: {}".format(e))
            # Should we raise a problem vs just try it again ?
            # raise RuntimeError('Trouble to connecting to MQTT, {}'.format(e))
            led_error_code(led, 2)
            continue
        
        dht22_reading = read_dht_22(sensor)
        
        debug_str = "None"
        if dht22_reading is not None:
            temp,hum = dht22_reading
            mqtt_client.publish("m/v1/home/{}/0/temperature".format(mqtt_config.MQTT_ZONE_ID), str(temp), retain=True)
            mqtt_client.publish("m/v1/home/{}/0/humidity".format(mqtt_config.MQTT_ZONE_ID), str(hum), retain=True)
            debug_str = "{} ; {}".format(temp, hum,)
        
        cpu_temp = read_cpu_temp()
        vsys_volts = read_vsys()
        
        print("{} ; CPU: {} ; Vsys: {}".format(debug_str, cpu_temp, vsys_volts))
        
        # HW Info
        mqtt_client.publish("m/v1/hw/{}/cpu/temperature".format(mqtt_config.MQTT_HW_ID), str(cpu_temp), retain=True)
        mqtt_client.publish("m/v1/hw/{}/vsys/voltage".format(mqtt_config.MQTT_HW_ID), str(vsys_volts), retain=True)
        mqtt_client.publish("m/v1/hw/{}/wlan/info".format(mqtt_config.MQTT_HW_ID), str(ifconfig), retain=True)
        
        print("Killing the MQTT Connection")
        mqtt_client.disconnect()
        print("Going sleep ...")
        time.sleep(1)
        
        # Prep HW for sleep
        wlan.disconnect()
        wlan.active(False)
        wlan.deinit()
        machine.lightsleep(60000)
        print("Woke up ...")



print('Start/Woke, reset clause {}'.format(machine.reset_cause()))
main()
