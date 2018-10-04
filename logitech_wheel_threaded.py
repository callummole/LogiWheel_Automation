import viz
from ctypes import *
import time
import numpy as np
from scipy.signal import sawtooth
import pandas as pd

import threading
import os

import platform

architecture = platform.architecture()

file_dir = os.path.dirname(__file__)

if architecture[0] == '32bit':
	dll_fname = os.path.join(file_dir, "LogiWheel", "Debug")
elif architecture[0] == '64bit':
	dll_fname = os.path.join(file_dir, "LogiWheel", "x64", "Debug")

print(dll_fname)

class steeringWheel:
	
	def __init__(self, handle):
		
		"""Initialise the steering wheel controller:
			
			args: 
				handle: A window handle. Once a vizard window is initialised you can get its handle like this:
					
					window = viz.window
					handle = window.getHandle()
					
		"""
		
		#Load the dll
#		self.lib = CDLL(r"SteeringControl.dll") 
		#self.lib = CDLL(dll_fname + '\SteeringControl.dll')	
		
		self.lib = CDLL(dll_fname + '\LogiWheel.dll')		
		self.handle = handle				
		
		self.init_vars()	
		
		
	def init_vars(self):
		
		self.error_t_minus_1 = 0 #variable for the pd controller
		
		self.min_output = -32768
		self.max_output = 32767
		
	def init(self):
				
		is_initialised = self.lib.initialise(self.handle)
		
		if is_initialised == 0:
			raise Exception("Wheel did not initialise")
			
	
		
	def play_spring_force(self, offsetPercentage, saturationPercentage, coefficientPercentage):
		
		"""Play a dynamic spring force on the x-axis. 
		
		args:
			offsetPercentage: Specifies the center of the spring force effect. Valid range is -100 to 100. Specifying 0 centers the spring. Any values outside this range are silently clamped
			
			saturationPercentage: Specify the level of saturation of the spring force effect. The saturation stays constant after a certain deflection from the center of the spring. It is comparable to a magnitude. Valid ranges are 0 to 100. Any value higher than 100 is silently clamped.
			
			coefficientPercentage: Specify the slope of the effect strength increase relative to the amount of deflection from the center of the condition. Higher values mean that the saturation level is reached sooner. Valid ranges are -100 to 100. Any value outside the valid range is silently clamped.
		"""
				
		success = self.lib.play_spring_force(offsetPercentage, saturationPercentage, coefficientPercentage)

		if success == 0:
			print("Could not play spring force")
			
	def stop_spring_force(self):
		"""Stop playing the spring force"""
		
		self.lib.stop_spring_force()
		
	def play_rumble_effect(self, magnitudePercentage):
		"""
		args:
			magnitudePercentage : Specifies the magnitude of the dirt road (rumble) effect. Valid ranges for magnitudePercentage are 0 to 100. Values higher than 100 are silently clamped.
		
		"""
		
		self.lib.play_dirt_track(magnitudePercentage)
	
	def stop_rumble_effect(self):
		
		played = self.lib.stop_dirt_track()
	
		if played == 0:
			print("Could not play constant force")
			
			
	def play_constant_force(self, magnitudePercentage):
		"""Play a constant force. A constant force works best when continuously updated with a value tied to the physics engine. Tie the steering wheel/joystick to the car's physics engine via a vector force. This will create a centering spring effect, a sliding effect, a feeling for the car's inertia, and depending on the physics engine it should also give side collisions (wheel/joystick jerks in the opposite way of the wall the car just touched). The vector force could for example be calculated from the lateral force measured at the front tires. This vector force should be 0 when at a stop or driving straight. When driving through a turn or when driving on a banked surface the vector force should have a magnitude that grows in a proportional way.
		args:
			magnitudePercentage : Specifies the magnitude of the constant force effect. A negative value reverses the direction of the force. Valid ranges for magnitudePercentage are -100 to 100. Any values outside the valid range are silently clamped.
		
		"""
		self.lib.play_constant_force(magnitudePercentage)
		
	def stop_constant_force(self):
		"""Stop a constant force"""
		
		self.lib.stop_constant_force()		

	def play_damper_force(self, coefficientPercentage):
		"""Simulate surfaces that are hard to turn on (mud, car at a stop) or slippery surfaces (snow, ice).
		
		args:
			
			coefficientPercentage : specify the slope of the effect strength increase relative to the amount of deflection from the center of the condition. Higher values mean that the saturation level is reached sooner. Valid ranges are -100 to 100. Any value outside the valid range is silently clamped. -100 simulates a very slippery effect, +100 makes the wheel/joystick very hard to move, simulating the car at a stop or in mud.
			"""
			
		self.lib.play_damper_force(coefficientPercentage)
		
	def stop_damper_force(self):
		"""Stop playing damper force"""
		
		self.lib.stop_damper_force()

	def shutdown(self):
		"""Shutdown the wheel"""
		
		self.lib.shutdown()		
	
	def get_state(self, normed = False):
		"""Get the current state of the wheel"""
		
		
		position = self.lib.getState()
				
		if normed:
			return self.norm_state(position)
		else:			
			return position
		
	def unnorm_state(self, pos):
		"""Pass a position value between -1 and 1 and get the raw wheel position"""
		
		m = (self.max_output - self.min_output) / 2.0
		c = self.max_output - m
		
		return c + pos * m
		
	def norm_state(self, raw_pos):
		"""pass a raw position value between the min and max output values and get a value between -1 and 1"""
		
		m = 2.0 / (self.max_output - self.min_output)
		
		c = 1.0 - m * self.max_output
		
		return c + raw_pos * m
		
	def pid_step(self, desired_position, dt, tau_p =400.0, tau_d = 20.0):
		"""Pass the desired position in normalised units"""
		
		norm_current_position = self.norm_state(self.get_state())

		#Error between desired and actual position
		error = desired_position - norm_current_position
		#Derivative of error
				
		d_error = (error - self.error_t_minus_1) / dt		
								
								
		#Set force output		
		force_p = -tau_p * error
		force_d =  - tau_d * d_error
		
		force = force_p + force_d
		clip_force = int(np.clip(force, -100, 100))		
	
		#print(norm_current_position, error, force, force_p, force_d)
						
		self.play_constant_force(clip_force)
					
		self.error_t_minus_1 = error

class steeringWheelThreaded(threading.Thread, steeringWheel):
	
	def __init__(self, handle):
		
		threading.Thread.__init__(self)
		steeringWheel.__init__(self, handle)
		
		self.thread_init()		
		self.control_off()
		self.set_position(0)
		
	def thread_init(self):
		"""Initialise the thread"""
		self.__thread_live = True
		
	def thread_kill(self):
		"""Turn the thread loop off"""
		self.__thread_live = False
	
	def control_on(self):
		"""Turn the pd controller on"""

		print ("LOGI WHEEL: CONTROLLER ON")
		self.__control_live = True
		
	def control_off(self):
		"""Turn the pd controller off"""
		self.__control_live = False
		self.stop_constant_force()

		print ("LOGI WHEEL: CONTROLLER OFF")
		
	
	def getControlState(self):
		"""returns control state"""
		return self.__control_live

	def run(self):
			
		dt = 1/60.0
		while self.__thread_live:
			
			t0 = time.time()			
			
			if self.__control_live:
				self.pid_step(self.desired_position, dt)
			
			t1 = time.time()
			
			dt_real = t1 - t0
			
			time.sleep(dt)

			
	def set_position(self, desired_position):
		"""Set the wheel position using the thread"""
				
		#print ("LOGI WHEEL: CHANGING DESIRED POSITION: " + str(desired_position))
		self.desired_position = desired_position

		if self.__control_live == False:
			self.control_on()
		

def test_sinusoid_thread():
	
	#Set up a vizard window and get it's handle
	viz.go()
	window = viz.window
	handle = window.getHandle()
	
	#Create a steeringWheel instance
	wheel = steeringWheelThreaded(handle)	
	wheel.init() #Initialise the wheel
	wheel.start() #Start the wheels thread
	
	
	wheel.set_position(0.5) #Set the pd control target
	wheel.control_on() #Turn the pd controller on
	
	time.sleep(2) #Give the wheel time to center

	wheel.control_off()

	time.sleep(2)

	wheel.set_position(np.sin(0) * 0.7)
	wheel.control_on()
	
	#Clock counter
	timer = 0
	
	desired_state = []
	true_state = []
	all_time = []
		
	t0 = time.time()
	
	while True:
			
		timer = time.time() - t0 #Get the time

		#Sinusoid state
		state = np.sin(timer * 1) * 0.7
		#Sawtooth state
#		state = sawtooth(timer * np.pi * 1.5, 0.5) * 0.8
		
		wheel.set_position(state)
		#Steering target
	
		desired_state.append(state)
		true_state.append(wheel.get_state(normed = True))
		all_time.append(timer)
		
		time.sleep(1/60.0)
		
		if timer > 5:
			
			break
			
	#Quite a few commands to shutdown the threaded wheel
	wheel.control_off()
	wheel.thread_kill() #This one is mission critical - else the thread will keep going 
	wheel.shutdown()
	
	
	
	import matplotlib.pyplot as plt	
	
	plt.figure()
	plt.plot(all_time, desired_state, 'b')
	plt.plot(all_time, true_state, 'r--')
	
	plt.figure()
	plt.plot(all_time, np.array(desired_state) - np.array(true_state))
	plt.show()
		
def test_constant_pd_thread():
	
	#Set up a vizard window and get it's handle
	viz.go()
	window = viz.window
	handle = window.getHandle()
	
	#Create a steeringWheel instance
	wheel = steeringWheelThreaded(handle)	
	wheel.init()
		
	wheel.start()
	wheel.set_position(0.0)
	wheel.control_on()
	time.sleep(1)
	wheel.set_position(-1.0)
	time.sleep(1)
	wheel.set_position(0.0)
	time.sleep(1)
	wheel.thread_kill()
	
	wheel.join()
	
#	wheel.play_spring_force(-100, 100, -100)
#	time.sleep(1)
#	wheel.stop_spring_force()
#	
#	
#	t0 = time.time()
#	timer = 0.0
#	
#	while timer < 2:
#		
#		old_timer = timer
#		timer = time.time() - t0
#			
#		dt = np.clip(timer - old_timer, 1e-10, 1e10)
#	
#		pos = wheel.get_state(normed = True)
#		#print("Pos: {}".format(pos))
#		wheel.pid_step(0.0, dt)		
#			
#		time.sleep(1/200.0)
#	wheel.shutdown()
#	print("END")
#	


def test_constant_pd():
	
	
	
	#Set up a vizard window and get it's handle
	viz.go()
	window = viz.window
	handle = window.getHandle()
	
	#Create a steeringWheel instance
	wheel = steeringWheel(handle)	
	wheel.init()
	
	
	
	wheel.play_spring_force(-100, 100, -100)
	time.sleep(1)
	wheel.stop_spring_force()
	
	
	t0 = time.time()
	timer = 0.0
	
	while timer < 60:
		
		old_timer = timer
		timer = time.time() - t0
			
		dt = np.clip(timer - old_timer, 1e-10, 1e10)
	
		pos = wheel.get_state(normed = True)
		#print("Pos: {}".format(pos))
		wheel.pid_step(0.0, dt)		
			
		time.sleep(1/200.0)
	wheel.shutdown()
	print("END")
	
	
def test_sinusoid(run = True):
	
	import matplotlib.pyplot as plt	
	
	steering_target = np.sin(np.linspace(0, 10, 60*10) * 0.5) * 0.3

	if not run:
		#Plot steering target
		plt.plot(range(steering_target.size), steering_target)	
		plt.show()
		
	if run:
		#Set up a vizard window and get it's handle
		viz.go()
		window = viz.window
		handle = window.getHandle()
		
		#Create a steeringWheel instance
		wheel = steeringWheel(handle)	
		wheel.init()
		
		#Set wheel to center
#		wheel.play_spring_force(0, 100, 100)
#		time.sleep(1)
#		wheel.stop_spring_force()
#		time.sleep(1)
		
		t0 = time.time()
		timer = 0.0
		
		while timer < 1:
		
			old_timer = timer
			timer = time.time() - t0
				
			dt = np.clip(timer - old_timer, 1e-10, 1e10)
		
			pos = wheel.get_state(normed = True)
			#print("Pos: {}".format(pos))
			wheel.pid_step(0.0, dt)		
				
			time.sleep(1/200.0)
		
		print("CENTRED")
		
		#init log
		
		position = []
		
		t0 = time.time()
		timer = 0.0
		i = 0
		while i < steering_target.size:
			
			old_timer = timer
			timer = time.time() - t0
			
			dt = np.clip(timer - old_timer, 1e-10, 1e10)
			
			print(dt)
			pos = wheel.get_state(normed = True)
			position.append(pos)
			#print("Pos: {}".format(pos))
			wheel.pid_step(steering_target[i], dt)			
				
			time.sleep(1/100.0)
			
			i += 1
		wheel.shutdown()
		print("END")
		
		return position, steering_target
		
def test_trout_playback(fname):
	
	"""Load a playback file and play it back. Then plot the results"""
	
	import matplotlib.pyplot as plt
	
	data = pd.read_csv(fname)
	
	steering_target = data['yawrate'].values
	dt = 1/60.0
	
	print(data.head())
	
	run = True
	if not run:
		#Plot steering target
		plt.plot(range(steering_target.size), steering_target, '-o')	
		plt.show()
		
	if run:
		#Set up a vizard window and get it's handle
		viz.go()
		window = viz.window
		handle = window.getHandle()
		
		#Create a steeringWheel instance
		wheel = steeringWheel(handle)	
		wheel.init()
		
		#Set wheel to center
#		wheel.play_spring_force(0, 100, 100)
#		time.sleep(1)
#		wheel.stop_spring_force()
#		time.sleep(1)
		
		t0 = time.time()
		timer = 0.0
		
		while timer < 1:
		
			old_timer = timer
			timer = time.time() - t0
				
			dt = np.clip(timer - old_timer, 1e-10, 1e10)
		
			pos = wheel.get_state(normed = True)
			#print("Pos: {}".format(pos))
			wheel.pid_step(0.0, dt)		
				
			time.sleep(1/200.0)
		
		print("CENTRED")
		
		#init log
		
		position = []
		
		t0 = time.time()
		timer = 0.0
		i = 0
		while i < steering_target.size:
			
			old_timer = timer
			timer = time.time() - t0
			
			dt = np.clip(timer - old_timer, 1e-10, 1e10)
			
			#print(dt)
			pos = wheel.get_state(normed = True)
			position.append(pos)
			#print("Pos: {}".format(pos))
			wheel.pid_step(steering_target[i], dt)			
				
			time.sleep(1/60.0)
			
			i += 1
		wheel.shutdown()
		
		plt.plot(range(steering_target.size), steering_target, 'k')	
		plt.plot(range(steering_target.size), position, 'ro-')	
		
		plt.show()
		
		print("END")
	
if __name__ == '__main__':
	
	# test_trout_playback('stock_5.csv')
	#test_sinusoid_thread()
	#test_sinusoid()
	
	#test_constant_pd_thread()
	
	test_constant_pd()
	
#	all_position, target = test_sinusoid(run = True)
#	
#	import matplotlib.pyplot as plt
#	
#	plt.plot(target, color = 'r')
#	plt.plot(all_position, color ='b')
#	
#	plt.show()