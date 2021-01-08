
## http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter


from tkinter import *
#from tkinter.ttk import *
from protocol2 import sign
import time

class ram_frame(LabelFrame):

	def __init__(self,window,protocol,trace,id):
		super().__init__(text="RAM")
		self.protocol = protocol
		self.trace = trace
		self.id = id
		#self["width"]=200
		#self["height"]=1000
		self.labels = {}
		self.entries = {}
		self.variables = {}
		self.row = 0
		self.data_ready = 0
		self.counter = 0
		self.start_time = time.time()*1000.0

		self.gui_spacer("")
		self.gui_entry("Torque Enable", "torque_enable", 0, False, True, False, 0x40, 1, 1 )
		self.gui_entry("LED", "led", 0, True, True, True, 0x41, 1, 1 )
		self.gui_entry("Control Mode", "control_mode", 0, True, True, True, 0x42, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Goal Position", "goal_position", 0, True, True, True, 0x43, 10, 2 )
		self.gui_entry("Goal Velocity", "goal_velocity", 0, True, True, True, 0x45, 1, 2 )
		self.gui_entry("Goal Current", "goal_current", 0, True, True, True, 0x47, 1, 2 )
		self.gui_entry("Goal PWM", "goal_pwm", 0, True, True, True, 0x49, 1, 2 )
		self.gui_spacer("---")
		self.gui_entry("Present Position", "present_position", 0, False, True, False, 0x4D, 10, 2 )
		self.gui_entry("Present Velocity", "present_velocity", 0, False, True, False, 0x4F, 1, 2 )
		self.gui_entry("Present Current", "present_current", 0, False, True, False, 0x51, 1, 2 )
		self.gui_entry("Present Voltage", "present_voltage", 0, False, True, False, 0x53, 1, 1 )
		self.gui_entry("Present Temperature", "present_temperature", 0, False, True, False, 0x54, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Moving", "moving", 0, False, True, False, 0x55, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Setpoint Position", "setpoint_position", 0, False, True, False, 0x56, 10, 2 )
		self.gui_entry("Setpoint Velocity", "setpoint_velocity", 0, False, True, False, 0x58, 1, 2 )
		self.gui_entry("Setpoint Current", "setpoint_current", 0, False, True, False, 0x5A, 1, 2 )
		self.gui_entry("Setpoint PWM", "setpoint_pwm", 0, False, True, False, 0x5C, 1, 2 )
		self.gui_spacer("---")
		self.gui_entry("Motor Current ADC", "motor_current_input_adc", 0, False, True, False, 0x5E, 1, 2 )
		self.gui_entry("Current Offset ADC", "motor_current_input_adc_offset", 0, False, True, False, 0x60, 1, 2 )
		self.gui_entry("Position Input ADC", "position_input_adc", 0, False, True, False, 0x62, 1, 2 )
		self.gui_entry("Current Input ADC", "voltage_input_adc", 0, False, True, False, 0x64, 1, 2 )
		self.gui_spacer("---")
		self.gui_entry("Protocol CRC Fail", "protocol_crc_fail", 0, False, True, False, 0x80, 1, 1 )
		self.gui_entry("Hardware Error Status", "hardware_error_status", 0, False, True, False, 0x81, 1, 1 )

		# update button
		#button_update = Button(self,text="Update",command = self.read_all)
		#button_update.grid(column = 2, row = 0, sticky='we')

		self.read_all()

	def gui_entry(self,text_label,variable_name,variable_value,has_local,has_servo,has_callback,callback_reg_address,callback_reg_scale,callback_reg_size):
		self.labels[variable_name] = Label(self, text = text_label, anchor="w", justify=LEFT)
		self.labels[variable_name].grid(column = 0, row = self.row, sticky='w')
		if has_local:
			self.variables[variable_name+"_local"] = StringVar()
			self.variables[variable_name+"_local"].set(str(variable_value))
		if has_servo:
			self.variables[variable_name+"_servo"] = StringVar()
			self.variables[variable_name+"_servo"].set("empty")
		if has_local:
			self.entries[variable_name+"_local"] = Entry(self, width = 15, textvariable = self.variables[variable_name+"_local"]) 
			self.entries[variable_name+"_local"].grid(column = 1, row = self.row)
		if has_servo:
			self.entries[variable_name+"_servo"] = Entry(self, width = 15, state="readonly", textvariable = self.variables[variable_name+"_servo"]) 
			self.entries[variable_name+"_servo"].grid(column = 2, row = self.row)
		if has_local and has_callback:
			self.entries[variable_name+"_local"].bind('<Return>', (lambda _: self.callback_entry(variable_name,callback_reg_address,callback_reg_scale,callback_reg_size)))
		self.row += 1

	def callback_entry(self,variable_name,callback_reg_address,callback_reg_scale,callback_reg_size):
		print("set " + variable_name + ":"+self.variables[variable_name+"_local"].get())
		#self.protocol.write_command(1,callback_reg_address,callback_reg_scale*int(self.variables[variable_name+"_local"].get()),callback_reg_size)
		print("write RAM...")
		if callback_reg_size==1:
			self.protocol.write_byte_command(
				self.id.current_id,
				callback_reg_address,
				[
					callback_reg_scale*int(self.variables[variable_name+"_local"].get())
				],
				extra_timeout=50
			)
		if callback_reg_size==2:
			self.protocol.write_word_command(
				self.id.current_id,
				callback_reg_address,
				[
					callback_reg_scale*int(self.variables[variable_name+"_local"].get())
				],
				extra_timeout=50
			)

	def gui_spacer(self,text_label):
		label = Label(self, text = text_label, anchor="w", justify=LEFT)
		label.grid(column = 0, row = self.row, sticky='w')
		self.row += 1		

	def read_all(self):
		# write test
		if self.trace.variables["square_current"].get() == 1:
			value = self.trace.test_square_current()
			if  value != 0:
				print("write RAM...")
				self.protocol.write_word_command(self.id.current_id,0x47,[value],verbose=1)
		# write test
		elif self.trace.variables["square_position"].get() == 1:
			value = self.trace.test_square_position()
			if  value != 0:
				print("write RAM...")
				self.protocol.write_word_command(self.id.current_id,0x43,[value],verbose=1)
		elif self.trace.variables["triangle_position"].get() == 1:
			value = self.trace.test_triangle_position()
			if  value != 0:
				print("write RAM...")
				self.protocol.write_word_command(self.id.current_id,0x43,[value],verbose=1)
		elif self.trace.variables["round_position"].get() == 1:
			update, servo_angles = self.trace.test_round_position()
			if  update:
				#print("write RAM...")
				self.protocol.write_word_command(2,0x43,[int((servo_angles[0,0]+45.0)*10.0)],verbose=0)
				self.protocol.write_word_command(1,0x43,[int((135.0-servo_angles[1,0])*10.0)],verbose=0)


		# send read command
		if (self.counter%100)==0:
			verb = 1
			end_time = time.time()*1000.0
			print("delay for 100 iterations:" + str(end_time-self.start_time) + "ms")
			self.start_time = end_time
		else:
			verb = 0
		error, result = self.protocol.read_byte_command(
			self.id.current_id,		# ID
			0x40, # from EEPROM
			54,	# byte number to read
			verbose=verb
		) # TODO change ID through GUI


		if error != 0 :
			print("error:"+str(error))
		elif len(result)==54:
			goal_position 		= float((result[3] + (result[4]<<8))/10.0)
			setpoint_position 	= float((result[22] + (result[23]<<8))/10.0)
			present_position 	= float((result[13] + (result[14]<<8))/10.0)
			goal_velocity 		= float(sign(result[5] + (result[6]<<8)))
			setpoint_velocity 	= float(sign(result[24] + (result[25]<<8)))
			present_velocity 	= float(sign(result[15] + (result[16]<<8)))
			goal_current 		= float(sign(result[7] + (result[8]<<8)))
			setpoint_current 	= float(sign(result[26] + (result[27]<<8)))
			present_current 	= float(sign(result[17] + (result[18]<<8)))
			goal_pwm 			= float(sign( result[9] + (result[10]<<8)))
			setpoint_pwm 		= float(sign( result[28] + (result[29]<<8)))

			self.trace.update(
				goal_position,
				setpoint_position,
				present_position,
				goal_velocity,
				setpoint_velocity,
				present_velocity,
				goal_current,
				setpoint_current,
				present_current,
				goal_pwm,
				setpoint_pwm)

			if self.counter == 0:
				self.variables['led_local'].set(str(result[1]))
				self.variables['control_mode_local'].set(str(result[2]))
				self.variables['goal_position_local'].set(str( goal_position ))
				self.variables['goal_velocity_local'].set(str( goal_velocity ))
				self.variables['goal_current_local'].set(str( goal_current ))
				self.variables['goal_pwm_local'].set(str( goal_pwm ))

			self.variables['torque_enable_servo'].set(str(result[0]))
			self.variables['led_servo'].set(str(result[1]))
			self.variables['control_mode_servo'].set(str(result[2]))
			self.variables['goal_position_servo'].set(str( goal_position ))
			self.variables['goal_velocity_servo'].set(str( goal_velocity ))
			self.variables['goal_current_servo'].set(str( goal_current ))
			self.variables['goal_pwm_servo'].set(str( goal_pwm ))
			self.variables['present_position_servo'].set(str( present_position ))
			self.variables['present_velocity_servo'].set(str( present_velocity ))
			self.variables['present_current_servo'].set(str( present_current ))
			self.variables['present_voltage_servo'].set(str(result[19]))
			self.variables['present_temperature_servo'].set(str(result[20]))
			self.variables['moving_servo'].set(str(result[21]))
			self.variables['setpoint_position_servo'].set(str( setpoint_position ))
			self.variables['setpoint_velocity_servo'].set(str( setpoint_velocity ))
			self.variables['setpoint_current_servo'].set(str( setpoint_current ))
			self.variables['setpoint_pwm_servo'].set(str( setpoint_pwm ))
			self.variables['motor_current_input_adc_servo'].set(str(result[30] + (result[31]<<8)))
			self.variables['motor_current_input_adc_offset_servo'].set(str(sign(result[32] + (result[33]<<8))))
			self.variables['position_input_adc_servo'].set(str(result[34] + (result[35]<<8)))
			self.variables['voltage_input_adc_servo'].set(str(result[36] + (result[37]<<8)))

			self.variables['protocol_crc_fail_servo'].set(str(result[52]))
			self.variables['hardware_error_status_servo'].set(str(result[53]))
			self.data_ready = 1


		self.counter += 1
		self.after(1,self.read_all)