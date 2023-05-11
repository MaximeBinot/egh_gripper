#!/usr/bin/env python2
import time
import sys
import threading
import os 
import rospy
from std_msgs.msg import Int32

from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus.exceptions import ConnectionException 

from daemon.egh_help import *
from daemon.egh_iodd import EGH_INDEX as index
from daemon.egh_iodd import EGH_CODE as code
from daemon.egh_iodd import EGH_CMD as cmd
from daemon.egh_iodd import getDataLength as dataLength
from daemon.egh_iodd import getErrMsg as errMsg


# Initialize Variables
initialized = False
allow_run = False
title = ""
Connected=False
connID=0
IOL_IpAdress = os.getenv("IOL_IP", '192.168.1.253')
IOL_Port = int(os.getenv("IOL_PORT", 502))
DevicePort = 1 # ToDo get the port automatically
DesiredPosition = 0.0
DesiredForce = 0
GripDirection = 0
WorkpieceNumber = 0
Command = 0
ExecCmd = False
CurrentPosition = 0.0
HWSwitch1 = False
HWSwitch2 = False
Workpiece8 = False
Workpiece7 = False
Workpiece6 = False
Workpiece5 = False
Workpiece4 = False
Workpiece3 = False
Workpiece2 = False
Workpiece1 = False
wpTolerance = [0.5, -1, -1, -1, -1, -1, -1, -1, -1]
wpTargetPosition = [0.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
wpGripForce = [0, -1, -1, -1, -1, -1, -1, -1, -1]
gfPresetCurrent = [0.2, 0.15, 0.1, 0.05]
MaximalStroke = 40.0
ReferenceDirection = False
ActStatus = 0
Referenced = False
Success = False
EndStop = False
Blocked = False
ProcessingCmd = False
BlockingCmd = True
wasLastCommandReference = False

EventPort = [0, 0, 0, 0]
EventQualifier = [0, 0, 0, 0]
EventCode = [0, 0, 0, 0]

#prgState = 0
prgStateLast = 0
safetyModeLast = 0
isPrgBreak = False
countReset = 0
countModbus = 0
countThread = 0

lock = threading.Lock()

def init():
	global initialized
	print("Connecting to gripper")
	connect(True)
	time.sleep(1.0)
	print("Starting modbus")
	startModbusTCP()
	time.sleep(3.0)
	print("Acknowledge and reference")
	cmdAcknowledge()
	ack_fails = 0
	ack_fails_max = 2
	while not cmdAcknowledge():
		time.sleep(2)
		ack_fails += 1
		if ack_fails > ack_fails_max:
			cmdReferencing()
			time.sleep(1)
			ack_fails = 0
			ack_fails_max += 1
	time.sleep(1)
	while not cmdReferencing():
		print("Referencing failed")
	print("Gripper initialization successful.")
	initialized = True

def get_Ip():
	global IOL_IpAdress
	#print("IOL_IpAdress=" + str(IOL_IpAdress))
	return str(IOL_IpAdress)
def set_Ip(variable):
	global IOL_IpAdress
	IOL_IpAdress = variable
	return IOL_IpAdress

def connect(variable):
	global Connected
	global client
	global connID
	print("connect("+str(variable)+") called")
	if variable:
		if Connected: 
			global client
			client.close()
			print("connect() -- Disconnecting")
		client = ModbusClient(host = IOL_IpAdress, port=IOL_Port)
		Connected = client.connect()
		print("connect() -- Connecting"+ IOL_IpAdress + ", returns: " + str(Connected))
		if Connected:
			time.sleep(0.5)
			connID = acyclic_connection_request()	
	else:		
		print("connect() -- Disconnecting")	
		if Connected:
			acyclic_close_request()
			time.sleep(0.1)
			client.close()
			connID=0
			Connected = False	
	return connID

def is_connected():
	global Connected
	#print("is_connected: ",Connected)
	return Connected

def ModbusTCP():
	#print ("ModbusTCP() called")
	global allow_run
	global modbus_thread
	global EventPort
	global countModbus 
	global countThread

	if allow_run:
		modbus_thread = threading.Timer(0.25, ModbusTCP) # operating faster than 20Hz results in bugs
		modbus_thread.daemon = True
		modbus_thread.start()
		if is_connected():
			countThread = countThread + 1
			try:
				if countThread %2 == 1:
					countModbus = countModbus + 1
					diag_msg = client.read_holding_registers(70, 2*len(EventPort), unit=1)
					if isinstance(diag_msg, ReadHoldingRegistersResponse):
						#print_hex(diag_msg.registers, "Diagnose: ")
						decode_diag_msg(diag_msg)
					input_msg = client.read_holding_registers(1, 4, unit=1)
					if isinstance(input_msg, ReadHoldingRegistersResponse):
						#print_hex(input_msg.registers, "ModbusTCP: ")
						decode_input_msg(input_msg)
					else:
						#print(input_msg)
						pass
					output_msg = generate_output_msg()
					rq = client.write_registers(2049, output_msg, skip_encode=True, unit=1)
					#print(output_msg);
					assert(rq.function_code < 0x80), "Write register failed!!!"
					#printInputs()
			except	ConnectionException as err:
				global Connected
				Connected = False
				print(err)
	else:
		print("ModbusTCP() disable")

def checkPrgState(prgState):
	#global prgState
	global prgStateLast
	global isPrgBreak
	global countReset

	#prgState = robotStatusBits
	if (prgState != prgStateLast) and (prgStateLast == 3):
		#stopModbusTCP()
		#cmdStop()
		set_Command(cmd.Stop)
		set_ExecCmd(True)
		#tr(prgState)+" prgStateLast: "+str(prgStateLast))
		prgStateLast = prgState
		#startModbusTCP()
	if (prgState != 3):
		isPrgBreak = True
		countReset = 0
	if (prgState == 3) and (isPrgBreak == True):	
		countReset = countReset + 1
		if (countReset == 10):
			isPrgBreak = False
			countReset = 0
	prgStateLast = prgState

def checkSafetyMode(safetyMode):
	global safetyModeLast
	if (safetyMode != safetyModeLast) and ((safetyModeLast != 2) or (safetyModeLast != 1)):
		#stopModbusTCP()
		#cmdFastStop()
		set_Command(cmd.FastStop)
		set_ExecCmd(True)
		#print("safetyMode: "+str(safetyMode)+" safetyModeLast: "+str(safetyModeLast))
		safetyModeLast = safetyMode
		#startModbusTCP()
	safetyModeLast = safetyMode

def getPrgBreak():
	global isPrgBreak
	return isPrgBreak
def resetPrgBreak():
	global isPrgBreak
	isPrgBreak = False
	return isPrgBreak

def startModbusTCP():
	global allow_run
	if allow_run == False:
		allow_run = True
		ModbusTCP()
		print("ModbusTCP() called, allow_run: " + str(allow_run))
	return allow_run

def stopModbusTCP():
	global allow_run
	allow_run = False
	return allow_run


def isModbusRunning():
	global allow_run
	return allow_run

# decode diagnose messages -- events
def decode_diag_msg(msg):
	decoder = BinaryPayloadDecoder.fromRegisters(msg.registers, byteorder=Endian.Big, wordorder=Endian.Big)
	global EventPort
	for i in range(len(EventPort)):
		EventPort[i] = decoder.decode_8bit_uint()
		global EventQualifier
		EventQualifier[i] = decoder.decode_8bit_uint()
		global EventCode
		EventCode[i] = decoder.decode_16bit_uint()
	#print(EventPort)

# decode input message
def decode_input_msg(msg):
	decoder = BinaryPayloadDecoder.fromRegisters(msg.registers, byteorder=Endian.Big, wordorder=Endian.Big)
	#decoder.skip_bytes(2)
	BitList = decoder.decode_bits()
	set_ProcessingCmd(BitList[7])
	set_Blocked(BitList[6])
	set_EndStop(BitList[5])
	set_Success(BitList[4])
	set_Referenced(BitList[3])
	set_ActStatus(bitList2Int(BitList[0: 2]))
	BitList = decoder.decode_bits()
	set_Workpiece1(BitList[0])
	set_Workpiece2(BitList[1])
	set_Workpiece3(BitList[2])
	set_Workpiece4(BitList[3])
	set_Workpiece5(BitList[4])
	set_Workpiece6(BitList[5])
	set_Workpiece7(BitList[6])
	set_Workpiece8(BitList[7])
	BitList = decoder.decode_bits()
	set_HWSwitch2(BitList[1])
	set_HWSwitch1(BitList[0])
	BitList = decoder.decode_bits()
	set_CurrentPosition(decoder.decode_32bit_float())
	decoder.reset()
	#print('decode_input_msg called')
	return

# ProcessData Input Variable: ProcessingCmd
def get_ProcessingCmd():
	global ProcessingCmd
	#print("ProcessingCmd=" + str(ProcessingCmd))
	return ProcessingCmd
def isNotProcessingCmd():
	if get_ProcessingCmd():
		return False
	else:
		return True
def set_ProcessingCmd(variable): 
	global ProcessingCmd
	ProcessingCmd = variable
	return ProcessingCmd

# ProcessData Input Variable: Blocked
def get_Blocked():
	global Blocked
	#print("Blocked=" + str(Blocked))
	return Blocked
def set_Blocked(variable): 
	global Blocked
	Blocked = variable
	return Blocked

# ProcessData Input Variable: EndStop
def get_EndStop():
	global EndStop
	#print("EndStop=" + str(EndStop))
	return EndStop
def set_EndStop(variable): 
	global EndStop
	EndStop = variable
	return EndStop

# ProcessData Input Variable: Success
def get_Success():
	global Success
	#print("Success=" + str(Success))
	return Success
def set_Success(variable): 
	global Success
	Success = variable
	return Success

# ProcessData Input Variable: Referenced
def get_Referenced():
	global Referenced
	#print("Referenced=" + str(Referenced))
	return Referenced
def set_Referenced(variable): 
	global Referenced
	Referenced = variable
	return Referenced

# ProcessData Input Variable: ActStatus
# 0 (0b xxxx.x000): Error
# 1 (0b xxxx.x001): Out of specification
# 2 (0b xxxx.x010): Maintenance required
# 3 (0b xxxx.x011): Ready for operation
def getActStatus():
	global ActStatus
	#print("ActStatus=" + str(ActStatus))
	return ActStatus
def set_ActStatus(variable): 
	global ActStatus
	ActStatus = variable
	return ActStatus

# ProcessData Input Variable: Workpiece1
def get_Workpiece1():
	global Workpiece1
	#print("Workpiece1=" + str(Workpiece1))
	return Workpiece1
def set_Workpiece1(variable): 
	global Workpiece1
	Workpiece1 = variable
	return Workpiece1

# ProcessData Input Variable: Workpiece2
def get_Workpiece2():
	global Workpiece2
	#print("Workpiece2=" + str(Workpiece2))
	return Workpiece2
def set_Workpiece2(variable): 
	global Workpiece2
	Workpiece2 = variable
	return Workpiece2

# ProcessData Input Variable: Workpiece3
def get_Workpiece3():
	global Workpiece3
	#print("Workpiece3=" + str(Workpiece3))
	return Workpiece3
def set_Workpiece3(variable): 
	global Workpiece3
	Workpiece3 = variable
	return Workpiece3

# ProcessData Input Variable: Workpiece4
def get_Workpiece4():
	global Workpiece4
	#print("Workpiece4=" + str(Workpiece4))
	return Workpiece4
def set_Workpiece4(variable): 
	global Workpiece4
	Workpiece4 = variable
	return Workpiece4

# ProcessData Input Variable: Workpiece5
def get_Workpiece5():
	global Workpiece5
	#print("Workpiece5=" + str(Workpiece5))
	return Workpiece5
def set_Workpiece5(variable): 
	global Workpiece5
	Workpiece5 = variable
	return Workpiece5

# ProcessData Input Variable: Workpiece6
def get_Workpiece6():
	global Workpiece6
	#print("Workpiece6=" + str(Workpiece6))
	return Workpiece6
def set_Workpiece6(variable): 
	global Workpiece6
	Workpiece6 = variable
	return Workpiece6

# ProcessData Input Variable: Workpiece7
def get_Workpiece7():
	global Workpiece7
	#print("Workpiece7=" + str(Workpiece7))
	return Workpiece7
def set_Workpiece7(variable): 
	global Workpiece7
	Workpiece7 = variable
	return Workpiece7

# ProcessData Input Variable: Workpiece8
def get_Workpiece8():
	global Workpiece8
	#print("Workpiece8=" + str(Workpiece8))
	return Workpiece8
def set_Workpiece8(variable): 
	global Workpiece8
	Workpiece8 = variable
	return Workpiece8

# ProcessData Input Variable: HWSwitch2
def get_HWSwitch2():
	global HWSwitch2
	#print("HWSwitch2=" + str(HWSwitch2))
	return HWSwitch2
def set_HWSwitch2(variable): 
	global HWSwitch2
	HWSwitch2 = variable
	return HWSwitch2

# ProcessData Input Variable: HWSwitch1
def get_HWSwitch1():
	global HWSwitch1
	#print("HWSwitch1=" + str(HWSwitch1))
	return HWSwitch1
def set_HWSwitch1(variable): 
	global HWSwitch1
	HWSwitch1 = variable
	return HWSwitch1

# ProcessData Input Variable: CurrentPosition
def get_CurrentPosition():
	global CurrentPosition
	#print("CurrentPosition=" + str(CurrentPosition))
	return CurrentPosition
def set_CurrentPosition(variable): 
	global CurrentPosition
	CurrentPosition = variable
	return CurrentPosition
# ProcessData Output Variable: ExecCmd
def set_ExecCmd(variable): 
	global ExecCmd
	ExecCmd = variable
	return ExecCmd
def get_ExecCmd(): 
	global ExecCmd
	return ExecCmd
# ProcessData Output Variable: Command
def set_Command(variable): 
	global Command
	Command = variable
	return Command
def get_Command(): 
	global Command
	return Command
# ProcessData Output Variable: WorkpieceNumber
def set_WorkpieceNumber(variable): 
	global WorkpieceNumber
	WorkpieceNumber = variable
	#print("WorkpieceNumber = " + str(WorkpieceNumber))
	return WorkpieceNumber
def get_WorkpieceNumber(): 
	global WorkpieceNumber
	return WorkpieceNumber
# ProcessData Output Variable: GripDirection
def set_GripDirection(variable): 
	global GripDirection
	GripDirection = variable
	#print("GripDirection = " + str(GripDirection))
	return GripDirection
def get_GripDirection(): 
	global GripDirection
	return GripDirection
# ProcessData Output Variable: DesiredForce
def setGripForce(variable): 
	global DesiredForce
	DesiredForce = variable
	#print("DesiredForce = " + str(DesiredForce))
	return DesiredForce
def getGripForce(): 
	global DesiredForce
	return DesiredForce
# ProcessData Output Variable: DesiredPosition
def set_DesiredPosition(variable): 
	global DesiredPosition
	DesiredPosition = variable
	#print("DesiredPosition = " + str(DesiredPosition))
	return DesiredPosition
def get_DesiredPosition(): 
	global DesiredPosition
	return DesiredPosition

def getWorkPiece(wp):
	if wp == 1:
		return get_Workpiece1()
	elif wp == 2:
		return get_Workpiece2()
	elif wp == 3:
		return get_Workpiece3()
	elif wp == 4:
		return get_Workpiece4()	
	elif wp == 5:
		return get_Workpiece5()
	elif wp == 6:
		return get_Workpiece6()	
	elif wp == 7:
		return get_Workpiece7()	
	elif wp == 8:
		return get_Workpiece8()
	return 0

def activeWorkpiece():
	if get_Workpiece1():
		return 1
	elif get_Workpiece2():
		return 2
	elif get_Workpiece3():
		return 3
	elif get_Workpiece4():
		return 4
	elif get_Workpiece5():
		return 5
	elif get_Workpiece6():
		return 6
	elif get_Workpiece7():
		return 7
	elif get_Workpiece8():
		return 8
	else:
		return 0
		
def isGrippedWp(wp):
	if wp == 1:
		return get_Workpiece1()
	elif wp == 2:
		return get_Workpiece2()
	elif wp == 3:
		return get_Workpiece3()
	elif wp == 4:
		return get_Workpiece4()	
	elif wp == 5:
		return get_Workpiece5()
	elif wp == 6:
		return get_Workpiece6()	
	elif wp == 7:
		return get_Workpiece7()	
	elif wp == 8:
		return get_Workpiece8()
	return 0
	
def isInPosition():
	wp=get_WorkpieceNumber()
	global wpTargetPosition
	target = getWorkpieceParam(wp,1) if wpTargetPosition[wp] == -1 else wpTargetPosition[wp]
	global wpTolerance
	tolerance = getWorkpieceParam(wp,2) if wpTolerance[wp] == -1 else wpTolerance[wp]
	ret = (target - tolerance <= get_CurrentPosition() <= target + tolerance)
	print("isInPosition(): wp= "+str(wp)+" target: "+str(target)+" tol: "+str(tolerance)+" wpBit"+str(getWorkPiece(wp))+" -- returns " + str(ret))
	return ret
def isSuccess():
	return get_Success()

def isBlocked():
	return get_Blocked()

def isClosed():
	gd= get_GripDirection()
	global ReferenceDirection
	rd= ReferenceDirection
	global MaximalStroke
	ms= MaximalStroke
	cp = get_CurrentPosition()
	ret=False
	if gd: 
		#innergrip
		if rd:
			if cp >= ms - 0.5:
				ret = True 
		else:		
			if cp <= 0.5:
				ret = True 
	else: 
		#outergrip
		if rd:
			if cp <= 0.5:
				ret = True 
		else:	
			if cp >= ms - 0.5:
				ret = True 	
	
	#print("isClosed():" +str(ret) +"-- gd: "+str(gd)+" ms: "+str(ms)+" rd: "+str(rd)+" cp: "+str(cp))
	return ret

def isEndStop():
	return get_EndStop()

def isOpen():
	gd= get_GripDirection()
	global ReferenceDirection
	rd= ReferenceDirection
	global MaximalStroke
	ms= MaximalStroke
	cp = get_CurrentPosition()
	ret=False
	if gd: 	
		#innergrip	
		if rd:
			if cp <= 0.5:
				ret = True 
		else:		
			if cp >=  ms - 0.5:
				ret = True 
	else: 	
		#outergrip				
		if rd:
			if cp >= ms - 0.5:
				ret = True 
		else:	
			if cp <= 0.5:
				ret = True 	
	#print("isOpen():" +str(ret) +"-- gd: "+str(gd)+" ms: "+str(ms)+" rd: "+str(rd)+" cp: "+str(cp))
	return ret

def isGraspOk():
	wp=get_WorkpieceNumber()
	ac = getMeasuredValue(code.ActualCurrent)
	global wpGripForce
	gf = getWorkpieceParam(wp,3) if wpGripForce[wp] == -1 else wpGripForce[wp]
	global gfPresetCurrent
	pc= gfPresetCurrent[gf] 
	ret =  False
	if (get_Command() == 4) and (get_Blocked() == True) and (is_InPosition() == False) and (pc - 0.05 <= ac <= pc + 0.05):
		ret = True
	
	print("isGraspOk():" +str(ret) +"-- wp: "+str(wp)+" gf: "+str(gf)+" ac: "+str(ac)+" pc: "+str(pc))
	return ret 

def isCmdActive():
	global isPrgBreak
	ret = (get_Success() != True) and (get_Blocked() != True) and (isPrgBreak == False)
	return ret

# Commands
def setBlockingCmd(value):
	global BlockingCmd
	BlockingCmd = value
	return True

def cmdEnabled():
	if get_Referenced() == False:
		#print('Greifer not referenced')
		return False
	if getActStatus() == 0:
		#print('Status = Error')
		return False
	resetPrgBreak()
	return True

def isNotSuccessFlag():
	if get_Success():
		return False
	else:
		return True

def cmdHandshake(feedback=True):
	global countModbus
	global lastCommand
	global wasLastCommandReference
	if wasLastCommandReference :
		#print("LastCommandReference "+str(Command))
		set_ExecCmd(False)
		time.sleep(0.7)
		set_ExecCmd(True)
		time.sleep(0.7)
		set_ExecCmd(False)
		time.sleep(0.7)
		wasLastCommandReference = False
	countModbus = 0
	if get_ExecCmd():
		set_ExecCmd(False)
		while (countModbus < 2 and is_connected()):
			pass
	set_ExecCmd(True)
	if feedback == True:
		ret = wait_until(get_ProcessingCmd)
		if ret: ret = wait_until(isNotSuccessFlag) 
		set_ExecCmd(False)
		return ret
	else:
		return True

def cmdHandshakeFS():
	global countModbus
	countModbus = 0
	if get_ExecCmd():
		set_ExecCmd(False)
		while (countModbus < 2 and is_connected()):
			pass
	set_ExecCmd(True)
	return True

def cmdAcknowledge():
	set_Command(cmd.Acknowledge)
	ret = cmdHandshake(feedback=True)
	print('cmdAcknowledge() ret: ' + str(getActStatus()))
	if getActStatus():
		return True
	else:	
		return False

def isPositionZero():
	if (get_CurrentPosition() == 0.0):
		return True
	else:
		return False

def cmdReferencing():
	print("Referencing")
	global wasLastCommandReference
	set_Command(cmd.Referencing)
	ret = cmdHandshake(feedback=True)
	wasLastCommandReference = True
	ret = wait_until(get_Referenced, timeout=10)
	if ret: ret = wait_until(get_Success, timeout= 10)
	else: print("Referencing timed out")
	return ret

def cmdReferencingGui():	
	set_Command(cmd.Referencing)
	ret = cmdHandshakeFS()
	return ret

def cmdStrokeMeasurement():
	if not cmdEnabled():
		return False
	set_Command(cmd.MessStroke)
	ret = cmdHandshakeFS()
	return True

def cmdGrip(workpiece=0, grip_direction=0, grip_force=0, ):
	if not cmdEnabled():
		return False
	#print('cmdGrip: wp: ' + str(workpiece) + ' dir: ' + str(grip_direction) + ' force: ' + str(grip_force))# + ' pos: ' + str(position))
	set_WorkpieceNumber(workpiece)
	set_GripDirection(grip_direction)
	setGripForce(grip_force)
	#set_DesiredPosition(position)
	set_Command(cmd.Grip)
	ret = cmdHandshake()
	return True

def cmdRelease(grip_direction=0):
	if not cmdEnabled():
		return False
	set_GripDirection(grip_direction)
	set_Command(cmd.Release)
	ret = cmdHandshake()
	return True

def cmdPositioning(target_pos):
	print("Positioning: {}".format(target_pos))
	if not cmdEnabled():
		return False
	set_DesiredPosition(target_pos)
	set_Command(cmd.Positioning)
	ret = cmdHandshake()
	return True

def cmdRelative(rel_pos):
	print("Relative: {}".format(rel_pos))
	if not cmdEnabled():
		return False
	set_DesiredPosition(rel_pos)
	set_Command(cmd.Relative)
	ret = cmdHandshake(feedback=True)
	return ret

def cmdStop():
	if not cmdEnabled():
		return False
	set_Command(cmd.Stop)
	ret = cmdHandshake()
	if ret: ret = wait_until(isNotProcessingCmd, timeout=5)
	#print('cmdStop successed? :', ret)
	return ret

def cmdFastStop():
	set_Command(cmd.FastStop)
	ret = cmdHandshakeFS()
	return ret

# Create output message
def generate_output_msg(cmd_count=None):
	exec_cmd = True
	if cmd_count is not None:
		exec_cmd = cmd_count %2 == 0
	builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
	builder.reset()
	tmp = []
	tmp.append(int(get_ExecCmd() and exec_cmd))
	for bit in (bin(get_Command())[2:].zfill(7)): tmp.insert(0, int(bit))
	builder.add_bits(tmp)
	tmp = []
	for bit in (bin(get_WorkpieceNumber())[2:].zfill(4)): tmp.insert(0, int(bit))
	for bit in (bin(0)[2:].zfill(3)): tmp.insert(0, int(bit))
	tmp.insert(0, int(get_GripDirection()))
	builder.add_bits(tmp)
	tmp = []
	for bit in (bin(0)[2:].zfill(5)): tmp.insert(0, int(bit))
	for bit in (bin(getGripForce())[2:].zfill(3)): tmp.insert(0, int(bit))
	builder.add_bits(tmp)
	tmp = []
	for bit in (bin(0)[2:].zfill(8)): tmp.insert(0, int(bit))
	builder.add_bits(tmp)
	tmp = []
	builder.add_32bit_float(get_DesiredPosition())
	global debug_msg 
	debug_msg = builder.to_registers()
	msg = builder.build()
	return msg

# Acyclic Access
def acyclic_access_success():
	global acyclicSuccess
	return acyclicSuccess

def generate_acyclic_write_header(data_length, index, subindex=0, cmd=0x0200):
	builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
	builder.reset()
	if (data_length != 0) and (data_length%2 != 0):
		data_length + 1
	builder.add_16bit_uint(0x0010 + data_length)	# twiche the number of Modbus registers to be sent
	builder.add_16bit_uint(DevicePort) 		# port number
	builder.add_16bit_uint(0x11FE) 			# INDEX_CAP?
	builder.add_16bit_uint(0x0000) 			# reserved
	builder.add_16bit_uint(0x0800 + DevicePort) 	# Function number - CALL = 8
	builder.add_16bit_uint(0xFE4A) 			# Fixed
	builder.add_16bit_uint(cmd) 			# Command - default for write (0x0200)
	builder.add_16bit_uint(index*256 + subindex) 	# index (MSB) + subindex (LSB)	
	#msg = builder.to_registers()
	#print_hex(msg, "generate_acyclic_write_header: ")
	return builder

def generate_acyclic_read_header(data_length, index, subindex=0, cmd=0x0300):
	builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
	builder.reset()
	
	if (data_length != 0) and (data_length%2 != 0):
		data_length + 1
	builder.add_16bit_uint(0x0010 + data_length*2)	# twiche the number of Modbus registers to be sent
	builder.add_16bit_uint(DevicePort) 		# port number
	builder.add_16bit_uint(0x11FE) 			# INDEX_CAP?
	builder.add_16bit_uint(0x0000) 			# reserved
	builder.add_16bit_uint(0x0800 + DevicePort) 	# Function number - CALL = 8
	builder.add_16bit_uint(0xFE4A) 			# Fixed
	builder.add_16bit_uint(cmd) 			# Command - default for read (0x0300)
	builder.add_16bit_uint(index*256 + subindex) 	# index (MSB) + subindex (LSB)	
	#msg = builder.to_registers()
	#print_hex(msg, "generate_acyclic_read_header: ")
	return builder
	
def generate_acyclic_read_request():
	global DevicePort
	builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
	builder.reset()
	builder.add_16bit_uint(0x0008)			# twiche the number of Modbus registers to be sent
	builder.add_16bit_uint(DevicePort)		# port number
	builder.add_16bit_uint(0x11FF)			# INDEX_CAP?
	builder.add_16bit_uint(0x0000)			# reserved
	msg = builder.to_registers()
	#print_hex(msg, "generate_acyclic_read_request: ")	
	return msg

def acyclic_close_request():
	global connID
	builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
	builder.reset()
	builder.add_16bit_uint(0xFFFF)
	builder.add_16bit_uint(0xFFFF)
	msg = builder.to_registers()
	#print_hex(msg,"acyclic_close_request: ")	
	client.write_registers(connID, msg, unit=1)

def acyclic_connection_request():
	builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
	builder.reset()
	builder.add_16bit_uint(0xFFFF) 	
	builder.add_16bit_uint(0xFFFF) 		
	reset = builder.to_registers()
	client.write_registers(0x1202, reset, unit=1)
	acyclic_call = client.read_holding_registers(4608, 2, unit=1)
	if isinstance(acyclic_call, ReadHoldingRegistersResponse):
			#print_hex(acyclic_call.registers, "acyclic_connection_request")
		return acyclic_call.registers[1]
	else:
		print(acyclic_call)
		return 0
	
def acyclic_read_data(index, subindex=0):
	if is_connected():
		global connID
		#print("connID=",connID)
		#write_request = generate_acyclic_write_header(0, index, subindex, cmd=0x0300)

		#builder = generate_acyclic_write_header(0, index, subindex, cmd=0x0300)
		builder = generate_acyclic_read_header(0, index, subindex, cmd=0x0300)

		write_request = builder.to_registers()
		#print_hex(write_request, "write_request: ")

		client.write_registers(connID, write_request, unit=1)
		read_request = generate_acyclic_read_request()
		client.write_registers(connID, read_request, unit=1)
		#time.sleep(0.05)
		time.sleep(0.2)
		data_length = dataLength(index,subindex)
		if data_length%2 != 0:
			data_length = data_length + 1
			acyclic_response = client.read_holding_registers(connID, 8 +  0.5*data_length, unit=1)
		#acyclic_response = client.read_holding_registers(connID, 8 +  data_length, unit=1)
		
		if isinstance(acyclic_response, ReadHoldingRegistersResponse):
			#print_hex(acyclic_response.registers, "acyclic_read: Response")
			decoder = BinaryPayloadDecoder.fromRegisters(acyclic_response.registers, byteorder=Endian.Big, wordorder=Endian.Big)
			decoder.skip_bytes(12)
			status = decoder.decode_8bit_uint()
			if status == 0x00:
				decoder.skip_bytes(3)
				return True, decoder
			ret = decoder
		else:
			print(acyclic_response)
			ret = -1 #"IllegalAddress"
		return False, ret
	else:	
		ret = -1
		return False, ret

def acyclic_read(index, subindex=0):
	counter = 0
	while counter <= 5:
		success,decoder = acyclic_read_data(index, subindex)
		counter = counter + 1
		#time.sleep(0.5)
		time.sleep(0.3)
		if success: return success,decoder
	print('acyclic_read: failed after ' +  str(counter) + ' attemps')
	return success,decoder

def acyclic_write_data(builder):
	if is_connected():
		global connID
		write_request = builder.to_registers()
		#print_hex(write_request, "acyclic_write_data: ")
		client.write_registers(connID, write_request, unit=1)
		read_request = generate_acyclic_read_request()
		client.write_registers(connID, read_request, unit=1)
		time.sleep(0.04)#RCC
		acyclic_response = client.read_holding_registers(connID, 8, unit=1)
		decoder = BinaryPayloadDecoder.fromRegisters(acyclic_response.registers, byteorder=Endian.Big, wordorder=Endian.Big)
		decoder.skip_bytes(12)
		status = decoder.decode_8bit_uint()
		#print_hex(acyclic_response.registers, "acyclic_write_data: Response")
		if status == 0x00: return True
		return False	

def acyclic_write(builder):
	counter = 0
	while counter <= 5:
		ret = acyclic_write_data(builder)
		#ret = acyclic_write_data(builder, index, subindex)
		counter = counter + 1
		time.sleep(0.5)
		if ret: return True
	print('acyclic_write: failed after ' +  str(counter) + ' attemps')
	return False

def getManufactureName():
	idx=index.ManufactureName
	success,decoder = acyclic_read(idx)
	if success:
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def getManufactureText():
	idx=index.ManufactureText
	success,decoder = acyclic_read(idx)
	if success:	
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def getProductName():
	idx=index.ProductName
	success,decoder = acyclic_read(idx)
	if success:
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def getProductID():
	idx=index.ProductID
	success,decoder = acyclic_read(idx)
	if success:
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def getProductText():
	idx=index.ProductText
	success,decoder = acyclic_read(idx)
	if success:
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def getSerialNumber():
	idx=index.SerialNumber
	success,decoder = acyclic_read(idx)
	if success:
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def getHWVersion():
	idx=index.HWVersion
	success,decoder = acyclic_read(idx)
	if success:
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def getFWVersion():
	idx=index.FWVersion
	success,decoder = acyclic_read(idx)
	if success:
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def getApplicationMark():
	idx=index.ApplicationMark
	success,decoder = acyclic_read(idx)
	if success:
		ret = decoder.decode_string(dataLength(idx, 0))
		ret = strip_control_characters(ret)
		print(ret)
	else:
		ret = "-1"
	return ret

def setApplicationMark(text):
	idx=index.ApplicationMark
	builder = generate_acyclic_write_header(len(text), idx)
	if len(text) != 0:
		builder.add_string(text)
	return acyclic_write(builder)
	
def getDeviceAccessBlock():
	success,decoder = acyclic_read(index.DeviceAccessBlock)
	if success:
		BitList = decoder.decode_bits()
		ret = BitList[0] + 2*BitList[1]
		print(BitList)
		print("Parameter: ",BitList[0])
		print("Data Storage: ",BitList[1])
	else:
		ret = -1
	return ret

def getReferenceDirection():
	success,decoder = acyclic_read(index.ReferenceDirection)
	if success:
		BitList = decoder.decode_bits()
		global ReferenceDirection
		ReferenceDirection = BitList[0]
		#print('Reference Direction:', ReferenceDirection)
		if BitList[0] == True:
			return 1
		else:
			return 0
	else:
		return -1

def setReferenceDirection(val):
	global ReferenceDirection
	idx=index.ReferenceDirection
	builder = generate_acyclic_write_header(1, idx)
	if val != 0: 
		ReferenceDirection = True
		builder.add_8bit_uint(1)
	else:
		ReferenceDirection = False
		builder.add_8bit_uint(0)
	return acyclic_write(builder)
	
def getMaintenanceInterval():
	success,decoder = acyclic_read(index.MaintenanceInterval)
	if success:
		ret = decoder.decode_32bit_int()
		#print("MaintenanceInterval: ",ret)
	else:
		ret = -1
	return ret 

def setMaintenanceInterval(val):
	idx=index.MaintenanceInterval
	builder = generate_acyclic_write_header(2, idx)
	builder.add_32bit_int(val)
	return acyclic_write(builder)

def getMaximalStroke():
	success,decoder = acyclic_read(index.MaximalStroke)
	if success:
		global MaximalStroke		
		MaximalStroke = decoder.decode_32bit_float()
		#print("MaximalStroke: ",MaximalStroke)
		return MaximalStroke
	else:
		return -1.0

def getWorkpieceParam(workpiece, subindex):
	if 1 <= workpiece <= 8:
		success,decoder = acyclic_read(index.Workpiece_1 + (workpiece - 1), subindex)
		if success:
			if subindex == 1:
				ret = decoder.decode_32bit_float()
				global wpTargetPosition
				wpTargetPosition[workpiece] = ret
				#print("wpTargetPosition["+str(workpiece)+"] = "+ str(wpTargetPosition[workpiece]))
			elif subindex == 2:
				ret = decoder.decode_32bit_float()
				global wpTolerance
				wpTolerance[workpiece] = abs(ret)
				#print("wpTolerance["+str(workpiece)+"] = "+ str(wpTolerance[workpiece]))
			elif subindex == 3:
				ret = decoder.decode_8bit_int()
				global wpGripForce
				wpGripForce[workpiece] = ret
			elif subindex == 4:
				ret = decoder.decode_8bit_int()
			else:
				#print("getWorkpieceParam: Wrong Workpiece subindex detected...")
				ret = -1
			print ("getWorkpieceParam ("+ str(workpiece)+","+str(subindex)+"): " +  str(ret))
		else:
			print ("getWorkpieceParam () failed")
			ret = -1
	else:
		if subindex == 1:
			ret = get_DesiredPosition()
		elif subindex == 2:
			ret = 0
		elif subindex == 3:
			ret = getGripForce()
		elif subindex == 4:
			ret = get_GripDirection()
		else:
			ret = -1
			#print("getWorkpieceParam: Wrong Workpiece subindex detected...")
		#print("getWorkpieceParam: Unknown Workpiece index detected, returning parameter in process data...")
	return ret

def setWorkpieceParam(workpiece, subindex, value):
	idx=index.Workpiece_1
	wp = int(workpiece)
	subIdx = int(subindex)
	if 1 <= wp <= 8:
		builder = generate_acyclic_write_header(dataLength(idx, subIdx), idx + (wp - 1), subIdx)
		if subIdx == 1:		
			builder.add_32bit_float(value)
		elif subIdx == 2:
			builder.add_32bit_float(value)
		elif subIdx == 3:
			if 0 <= value <= 3:
				builder.add_8bit_uint(int(value))
			else:
				#print("setWorkpieceParam: Wrong gripping force value detected...")
				pass
		elif subIdx == 4:
			if value != 0: 
				builder.add_8bit_uint(1)
			else:
				builder.add_8bit_uint(0)
		ret = acyclic_write(builder)
		#ret = acyclic_write(builder, idx + (workpiece - 1), subIdx)
		#print("setWorkpieceParam ("+ str(wp)+","+str(subIdx)+"," +str(value)+"): " + str(ret))
		return ret
	else:
		print("setWorkpieceParam: Wrong Workpiece index detected...")
	return False

def cmdSystem(iodd_code):
	idx= index.SystemCommand
	builder = generate_acyclic_write_header(dataLength(idx), idx)
	builder.add_8bit_uint(iodd_code)
	return acyclic_write(builder)

def getMeasuredValue(iodd_code):
	idx= iodd_code
	success,decoder = acyclic_read(idx)
	if success: 	
		if idx == code.Position:		
			ret = decoder.decode_32bit_float()
		elif idx == code.ActualCurrent:
			ret = decoder.decode_32bit_float()
		elif idx == code.ActualVoltage:
			ret = decoder.decode_32bit_float()
		elif idx == code.ActualTemp:
			ret = decoder.decode_32bit_float()
		elif idx == code.SensorSwitch1:
			ret = decoder.decode_8bit_int()
		elif idx == code.SensorSwitch2:
			ret = decoder.decode_8bit_int()
		elif idx == code.ErrorCounter:
			ret = decoder.decode_16bit_uint()
		elif idx == code.CycleCounter:
			ret = decoder.decode_32bit_uint()
		elif idx == code.ImpulseCounter:
			ret = decoder.decode_32bit_uint()
		elif idx == code.MaintenanceCounter:
			ret = decoder.decode_32bit_int()
		else:
			ret = -1
		#print("getMeasuredValue: ",ret)
	else:
		#print("getMeasuredValue: Failed.")
		ret = -1
	return ret

def getDeviceStatus():
	success,decoder = acyclic_read(index.DeviceStatus)
	if success:
		ret = decoder.decode_8bit_uint()
		#print("getDeviceStatus: ",ret)
	else:
		ret = -1
	return ret

def getDetailDeviceStatus(subindex):
	success,decoder = acyclic_read(index.DetailDeviceStatus, subindex)	
	if success:
		BitList = decoder.decode_bits()
		msg_type=bitList2Int(BitList[0: 4])
		msg_code = decoder.decode_16bit_uint()
		#print("getDetailDeviceStatus: type "+ str(msg_type) + ", msg code: " + str(msg_code))
	else:
		msg_code = 0
	return msg_code
	
def getLastError():
	success,decoder = acyclic_read(index.DetailLastError)
	if success:
		ret = decoder.decode_8bit_uint()
		#print("getLastError: ",ret)
	else:
		ret = -1
	return ret

def isEvent(index):
	global EventPort
	#print("EventPort[0]: ",EventPort[0])
	return EventPort[index]

def getEventQualifier(index):
	byte = EventQualifier[index]
	return byte & 0x07, byte & 0x08, (byte >> 4) & 0x03, (byte >> 6) 

def getEventCode(index):
	return EventCode[index]

def exit_handler():
	#print("Server will be closed")
	#server.server_close() 
	#print("ModbusTCP stopped")
	global modbus_thread
	modbus_thread.cancel()
	#print("Client will be closed")
	if is_connected():
		client.close()
		global Connected
		Connected = False
	return 0

def getErrorMsg(code):
	return errMsg(code)

