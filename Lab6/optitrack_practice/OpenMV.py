import sensor, image, time, math, pyb, socket, network, omv
from mqtt import MQTTClient

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

#omv.disable_fb(True)

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

client = MQTTClient("blob", "test.mosquitto.org", port=1883)

sock = socket.socket()
addr = socket.getaddrinfo("test.mosquitto.org", 1883)[0][-1]

print(addr)

client.connect()

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
red_threshold = (35, 60, 35, 85, 20, 60) # generic_red_thresholds

sensor.reset() # 初始化摄像头
sensor.set_pixformat(sensor.RGB565)  # 格式为 RGB565.
sensor.set_framesize(sensor.QQVGA) # 使用 QQVGA 速度快一些 160x120
sensor.skip_frames(time = 2000) # 跳过2000s，使新设置生效,并自动调节白平衡
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock() # 追踪帧率

def find_max(blobs):
    max_size = 0;
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob = blob
            max_size = blob[2]*blob[3]
    return max_blob

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # 从感光芯片获得一张图像
    blobs = img.find_blobs([red_threshold])
    if blobs:
        red_led.on()
        max_blob = find_max(blobs)
        # These values are stable all the time.
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(max_blob.cx(), max_blob.cy())
        #print("x:", max_blob.cx())
        #print("y:", max_blob.cy())
        size = max_blob[2]*max_blob[3]
        msg = str(80-max_blob.cx()) + " " + str(size)
        print(msg)
        client.publish("blob", msg)
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints([(max_blob.cx(), max_blob.cy(), int(math.degrees(max_blob.rotation())))], size=20)
    else:
        red_led.off()
        print("No blob found")
