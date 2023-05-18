import sensor, image, time, os, pyb, network, rpc, struct, omv
duck_threshold = (50, 75, -25, 20, 45, 75)
def setup_network(ssid, key):
	network_if = network.WLAN(network.STA_IF)
	network_if.active(True)
	network_if.connect(ssid, key)
	print("Trying to connect... (may take a while)...")
	print(network_if.ifconfig)
	interface = rpc.rpc_network_slave(network_if)
	return interface
def init_sensor():
	sensor.reset()
	sensor.set_pixformat(sensor.RGB565)
	sensor.set_framesize(sensor.QQVGA)
	sensor.skip_frames(time = 2000)
	sensor.set_auto_gain(False, gaindb=20)
	sensor.set_auto_whitebal(False)
	sensor.set_contrast(-3)
def find_max(blobs):
	max_size = 0;
	for blob in blobs:
		if blob[2]*blob[3] > max_size:
			max_blob = blob
			max_size = blob.area()
	return max_blob
def stream_generator_cb():
	return sensor.snapshot().compress(quality=90).bytearray()
def jpeg_image_stream_cb():
	interface.stream_writer(stream_generator_cb)
def jpeg_image_stream(data):
	pixformat, framesize = bytes(data).decode().split(",")
	sensor.set_pixformat(eval(pixformat))
	sensor.set_framesize(eval(framesize))
	interface.schedule_callback(jpeg_image_stream_cb)
	return bytes()
def duck_detection(data):
	img = sensor.snapshot()
	img.lens_corr(strength=1.6)
	blobs = img.find_blobs([duck_threshold], merge=True, area_threshold=25, pixels_threshold=10)
	if blobs:
		red_led.on()
		green_led.on()
		max_blob = find_max(blobs)
		clock.tick()
		img.draw_rectangle(max_blob.rect())
		img.draw_cross(max_blob.cx(), max_blob.cy())
		print(max_blob.cx(), max_blob.cy(), max_blob.w(), max_blob.h(), max_blob.w()*max_blob.h())
		return struct.pack("<HHHH", max_blob.cx(), max_blob.cy(), max_blob.w(), max_blob.h())
	else:
		red_led.off()
		green_led.off()
		return struct.pack("<HHHH", 0, 0, 0, 0)
if __name__ == "__main__":
	red_led = pyb.LED(1)
	green_led = pyb.LED(2)
	blue_led = pyb.LED(3)
	red_led.off()
	green_led.off()
	blue_led.off()
	blue_led.on()
	green_led.on()
	time.sleep_ms(500)
	blue_led.off()
	green_led.off()
	time.sleep_ms(500)
	SSID='AIRLab-BigLab'
	KEY='Airlabrocks2022'
	interface = setup_network(SSID, KEY)
	init_sensor()
	clock = time.clock()
	while(True):
		interface.register_callback(duck_detection)
		interface.register_callback(jpeg_image_stream)
		interface.loop()
