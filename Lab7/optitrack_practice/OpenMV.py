# Program that returns the cx, cy, and size of the biggest duck detected
# karenli - Thu Mar 30 2023

import sensor, image, time, os, pyb

omv.disable_fb(True)

SSID='AIRLab-BigLab' # Network SSID
KEY='Airlabrocks2022'  # Network key

# Init wlan module and connect to network
print("Trying to connect... (may take a while)...")
wlan = network.WINC()
wlan.active(True)
wlan.connect(SSID, key=KEY, security=wlan.WPA_PSK)
# We should have a valid IP now via DHCP
print(wlan.ifconfig())

blue_led.on()
green_led.on()
time.sleep_ms(500)
blue_led.off()
green_led.off()
time.sleep_ms(500)

client = MQTTClient("duck", "test.mosquitto.org", port=1883)

sock = socket.socket()
addr = socket.getaddrinfo("test.mosquitto.org", 1883)[0][-1]

client.connect()

def init_sensor(pixformat=sensor.RGB565, framesize=sensor.QVGA, windowsize=None,
                gain=0, autoexposure=False, autowhitebal=False, contrast=0, saturation=0):
    sensor.reset()
    sensor.set_pixformat(pixformat)
    sensor.set_framesize(framesize)
    if windowsize:
        sensor.set_windowing(windowsize)
    sensor.set_auto_gain(False, gaindb=gain)
    sensor.set_auto_exposure(autoexposure, 10000)
    sensor.set_auto_whitebal(autowhitebal)
    sensor.set_contrast(contrast)
    sensor.set_saturation(saturation)
    sensor.skip_frames(time = 2000)


def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob = blob
            max_size = blob.area()
    return max_blob


if __name__ == "__main__":
    red_led = pyb.LED(1)
    green_led = pyb.LED(2)
    blue_led = pyb.LED(3)
    duck_threshold = (55,105,-25,10,40,60)
    clock = time.clock() # 追踪帧率
    init_sensor()
    while(True):
        clock.tick()
        img = sensor.snapshot()
        img.lens_corr(strength=1.6)
        ducks = img.find_blobs([duck_threshold], merge=True, area_threshold=25, pixels_threshold=15)
        if ducks:
            red_led.on()
            green_led.on()
            duck = find_max(ducks)
            # img.draw_rectangle(duck.rect())
            # img.draw_cross(duck.cx(), duck.cy())
            theta = -0.5003834 + 0.00290751 * duck.cx()
            dist = 3.927166302253113 + 1/math.sqrt(duck.area()) * 1107.16448546
            msg = str(theta) + " " + str(dist)
            print(msg)
        else:
            red_led.off()
            green_led.off()
            msg = str(0) + " " + str(0)
            print(msg)
        client.publish("duck", msg)
