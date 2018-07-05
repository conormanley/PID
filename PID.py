""" Basic module to simulate a PID controller."""

import matplotlib
import matplotlib.pyplot as plt
import time
import math
import copy
import random

SEED = random.seed(21)

def sigmoid_plot():
	val_arr = []
	for i in range(50, 130, 1):
		#temp = .1 * (1 / (1 + math.exp(10 * (0.5 - i / 100))))
		temp = .1 * 50 * (1 - 1 / ( 1 + math.exp(.3 * (70 - i))))
		val_arr.append(temp)
	
	plt.plot(range(50, 50 + len(val_arr)), val_arr)
	plt.show()

def measure(output, current, dt, ambient=50):
	new_temp = current + (ambient - current) * .1 * (1 / (1 + math.exp(10 * (0.5 - random.random())))) * dt + .07 * output * dt
	
	#reaction starting at T=70
	if current >= 70:
		new_temp += dt * 50 * (1 - 1 / ( 1 + math.exp(.3 * (70 - current))))
	return new_temp

def PID_plot(time_arr, temp_arr, output_arr, title=None):
	plt.subplot(121)
	plt.plot(time_arr, temp_arr)
	plt.ylabel("Measured Temperature")
	plt.xlabel("Time")
	
	if title != None:
		plt.suptitle(title)
	
	plt.subplot(122)
	plt.plot(time_arr, output_arr, linestyle='dashed')
	plt.ylabel("Controller Output")
	plt.xlabel("Time")
	
	plt.subplots_adjust(left=0.2, wspace=0.8, top=0.8)
	
	keep_open=True
	plt.show(block=keep_open)	#block=False to exit out of plots
	if keep_open==False:
		time.sleep(1)
		plt.close()
	
	
def controller(setpoint, initial_value, until=1000, dt=0.1, Kp=3, Ki=4, Kd=16, 
						output_min= -1000, output_max=1000, tol=.025):
	""" Basic PID controller that will step through time until the `measured_value` is 
		consistently within `tol` tolerance of the `setpoint`. 
	
	==========		Inputs:		==========
	setpoint 					:= target value for the controller to reach
	initial_value 			:= starting measurement
	until						:= maximum simulation time
	dt								:= step size in time
	Kp							:= proportional control constant
	Ki								:= integral control constant
	Kd							:= derivative control constant
	output_min			:= minimum allowable controller output
	output_max			:= maximum allowable controller output
	tol							:= fraction of setpoint allowed to deviate
	
	==========		Internal:		==========
	previous_error		:= value of error from previous step
	integral					:= value of integral error from step
	t								:=	time (s)
	measured_value	:= value from most recent step
	tol_count				:= counter for tolerance successions
	error						:= value of error from step
	derivative				:= value of derivative error from step
	output					:= controller response to error
	
	==========		Outputs:		==========
	time_arr					:= list of time intervals of recorded data
	temp_arr				:= list of temperatures at time intervals
	output_arr				:= list of output valuse at time intervals
	"""
		
	previous_error = 0
	integral = 0
	t = 0
	measured_value = initial_value
	temp_arr = [measured_value]
	output_arr = [0]
	time_arr = [t]
	tol_count = 0
	
	while True:
		error = setpoint - measured_value
		integral = integral + error * dt
		derivative = (error - previous_error) / dt
		output = max(min(Kp * error + Ki * integral + Kd * derivative, output_max), output_min)
		output_arr.append(output)
		previous_error = error	
		measured_value = measure(output, measured_value, dt)
		t += dt
		time_arr.append(t)
		temp_arr.append(measured_value)
		if abs(measured_value - setpoint) / setpoint <= tol:
			tol_count += 1
		else:
			tol_count = 0
		if tol_count >= int(10 / dt):
			break
		if t >= until:
			break
	
	return time_arr, temp_arr, output_arr
	
def autotune(Ki_range=100, Kp_range=100, Kd_range=100, inc=10):
	best = 10000
	best_set = [0, 0, 0]
	best_arr = []
	eq_arr = []
	for Ki in range(0, Ki_range+1, inc):
		for Kp in range(0, Kp_range+1, inc):
			for Kd in range(0, Kd_range+1, inc):
				time_arr, temp_arr, output_arr = controller(100, 50, Kp=Kp, Ki=Ki, Kd=Kd)
				eq_arr.append(time_arr[-1])
				if time_arr[-1] < best:
					print("New best: Kp={0}, Ki={1}, Kd={2}, Was={3} Now time={4}".format(Kp, Ki, Kd, best, time_arr[-1]))
					best = time_arr[-1]
					best_set = copy.deepcopy([Kp, Ki, Kd])
					best_arr = [time_arr, temp_arr, output_arr]
	
	print("Best time: {0}".format(best))	
	print("Best settings: Kp={0}, Ki={1}, Kd={2}".format(best_set[0], best_set[1], best_set[2]))
	plt.plot(range(len(eq_arr)), eq_arr)
	plt.show()
	PID_plot(best_arr[0], best_arr[1], best_arr[2])
	return best_set

def run(Kp, Ki, Kd, setpoint):
	time_arr, temp_arr, output_arr = controller(setpoint, 50, Kp=Kp, Ki=Ki, Kd=Kd)
	title = "Temperature: {0} Control Time: {1}".format(str(setpoint), str(time_arr[-1]))
	PID_plot(time_arr, temp_arr, output_arr, title=title)

if __name__ == "__main__":
	"""
	PID_settings = autotune(Ki_range=100, Kp_range=100, Kd_range=200, inc=10)
	
	run(PID_settings[0], PID_settings[1], PID_settings[2], 60)
	run(PID_settings[0], PID_settings[1], PID_settings[2], 70)
	run(PID_settings[0], PID_settings[1], PID_settings[2], 80)
	run(PID_settings[0], PID_settings[1], PID_settings[2], 90)
	run(PID_settings[0], PID_settings[1], PID_settings[2], 100)
	run(PID_settings[0], PID_settings[1], PID_settings[2], 110)
	run(PID_settings[0], PID_settings[1], PID_settings[2], 120)
	run(PID_settings[0], PID_settings[1], PID_settings[2], 130)

	"""	
	
	run(30, 20, 10, 120)


