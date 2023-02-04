from gpiozero import Device, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
#from pynput import keyboard

Device.pin_factory = PiGPIOFactory()
oservo = Servo(24)
iservo = Servo(23)
# 24 vertical
# 23 horizontal

iservo.value = 0
oservo.value = 0
n = 0
x = 0
servo = 0 # 0 is yellow pin, 1 is green pin

while True:
	print("Choose servo")
	servo = int(input())
	print("Choose value (-100 to 100)")
	n = int(input()) / 100
	if servo == 0 :
		print(f"Servo 0 set to value {n}\n")
		iservo.value = n
	else:
		print(f"Servo 1 set to value {n}\n")
		oservo.value = n

'''while True:
	with keyboard.Events() as events:
		event = events.get(1e6)
		if event.key == keyboard.KeyCode.from_char('w'):
			q = iservo.value + 0.05
			if q > 1:
				q = 1
			iservo.value = q
			print(iservo.value)
		elif event.key == keyboard.KeyCode.from_char('s'):
			q = iservo.value - 0.05
			if q < -1:
				q = -1
			iservo.value = q
			print(iservo.value)
		elif event.key == keyboard.KeyCode.from_char('d'):
			q = oservo.value + 0.05
			if q > 1:
				q = 1
			oservo.value = q
			print(oservo.value)
		elif event.key == keyboard.KeyCode.from_char('a'):
			q = oservo.value - 0.05
			if q < -1:
				q = -1
			oservo.value = q
			print(oservo.value)'''



