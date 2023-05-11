def enum(**enums):
	return type('Enum', (), enums)
EGH_INDEX = enum(
	# Identification data
	ManufactureName = 0x0010,
	ManufactureText = 0x0011,
	ProductName	= 0x0012,
	ProductID	= 0x0013,
	ProductText 	= 0x0014,
	SerialNumber	= 0x0015,
	HWVersion	= 0x0016,
	FWVersion	= 0x0017,
	ApplicationMark = 0x0018,

	# Parameter
	DeviceAccessBlock 	= 0x000C,
	ReferenceDirection 	= 0x0054,
	MaintenanceInterval	= 0x00CC,
	MaximalStroke		= 0x00CD,

	# Parameter Workpiece
	Workpiece_1 		= 0x0065,
	Workpiece_2 		= 0x0066,
	Workpiece_3 		= 0x0067,
	Workpiece_4 		= 0x0068,
	Workpiece_5 		= 0x0069,
	Workpiece_6 		= 0x006A,
	Workpiece_7 		= 0x006B,
	Workpiece_8 		= 0x006C,
	
	TargetPosition		= 1,
	Toleranz		= 2,
	GripForce		= 3,
	GripDirection		= 4,

	# System Commands
	SystemCommand		= 0x0002,

	# Device Status
	DeviceStatus		= 0x0024,
	DetailDeviceStatus	= 0x0025,
	DetailLastError		= 0x00C8)


EGH_CODE = enum(
	RestoreFactoryDefaults	= 0x0082,
	MaintenanceCounterReset = 0x00A1,
	TeachWorkpiece		= 0x00A0,
	
	# Measurement
	Position 		= 0x0046,
	ActualCurrent		= 0x0048,
	ActualVoltage 		= 0x0051,
	ActualTemp		= 0x0050,
	SensorSwitch1		= 0x00AE,
	SensorSwitch2		= 0x00AF,
	ErrorCounter		= 0x0020,
	CycleCounter		= 0x00C9,
	ImpulseCounter		= 0x00CA,
	MaintenanceCounter 	= 0x00CB,

	Notification1		= 0,
	Notification2		= 1,
	Warning1		= 2,
	Warning2		= 3,
	Error			= 4,

	DetailNotification1_4	= 0,
	DetailNotification1	= 1,
	DetailNotification2	= 2,
	DetailNotification3	= 3,	
	DetailNotification4	= 4,

	# System commands
	RestoreDeliveryStatus	= 0x0082,
	ResetMaintenanceCounter = 0x00A1,
	TeachInWorkpiece	= 0x00A0)

EGH_CMD = enum(
	FastStop	= 0,
	Acknowledge 	= 1,
	Referencing 	= 2,
	Release		= 3,
	Grip		= 4,
	Positioning	= 5,
	Relative	= 6,
	MessStroke	= 7, 
	Stop		= 8, 
	Calibrate	= 9)

def getErrMsg(code):
	if code == 0x0000: 
		ret = "COMMUNICATION FAILED"
	elif code == 0x1801: 
		ret = "WARNING_BOOT MODE"
	elif code == 0x1806: 
		ret = "NOT REFERENCED"
	elif code == 0x1856: 
		ret = "GRIPPING FORCE OUTSIDE OF SPECIFICATION"
	elif code == 0x1857: 
		ret = "WORKPIECE SELECTION OUTSIDE OF SPECIFICATION"
	elif code == 0x1858: 
		ret = "POSITION MOVEMENT BLOCKED"
	elif code == 0x18D2: 
		ret = "ERROR CONFIG MEMORY"
	elif code == 0x18D9: 
		ret = "ERROR FAST STOP"
	elif code == 0x18DE: 
		ret = "ERROR CURRENT"
	elif code == 0x187A: 
		ret = "ERROR_Life_Sign"
	elif code == 0x187E: 
		ret = "ERROR VALUE"
	elif code == 0x18DA: 
		ret = "ERROR PRESSURE"
	elif code == 0x18E5: 
		ret = "ERROR POS SYSTEM"
	elif code == 0x1822: 
		ret = "POSITION NOT REACHABLE"
	elif code == 0x1000: 
		ret = "UNKNOWN ERROR (0x1000)"
	elif code == 0x4000: 
		ret = "TEMPERATUR FAIL"
	elif code == 0x4210: 
		ret = "DEVICE TEMPERATUR TOO HIGH"
	elif code == 0x5110: 
		ret = "SUPPLY VOLTAGE TOO HIGH"
	elif code == 0x5111: 
		ret = "SUPPLY VOLTAGE TOO LOW"
	elif code == 0x8C42: 
		ret = "MAINTENANCE REQUIRED - CHANGE WARING PARTS"
	elif code == 0xFF99: 
		ret = "REQUEST UPLOAD"
	else: 
		ret = "UNKNOWN ERROR (0X"+format(code,"02x")+")"
	return ret


def getDataLength(index, subindex=0):
	if index >= EGH_INDEX.Workpiece_1 and index <= EGH_INDEX.Workpiece_8:
		if subindex == EGH_INDEX.TargetPosition:
			return 4
		elif subindex == EGH_INDEX.Toleranz:
			return 4
		elif subindex == EGH_INDEX.GripForce:
			return 1
		elif subindex == EGH_INDEX.GripDirection:
			return 1
		else:
			return 0 
	else:
		switcher = {
			EGH_INDEX.ManufactureName: 63, # Manufacture name
			EGH_INDEX.ManufactureText: 63, # Manufacture text
			EGH_INDEX.ProductName: 63, # Product name
			EGH_INDEX.ProductID: 63, # Product id
			EGH_INDEX.ProductText: 63, # Product text
			EGH_INDEX.SerialNumber: 15, # serial number
			EGH_INDEX.HWVersion: 63, # HW version
			EGH_INDEX.FWVersion: 63, # FW version
			EGH_INDEX.ApplicationMark: 31, # Application specific Tag
			EGH_INDEX.DeviceAccessBlock: 1 , # Device access blocked
			EGH_INDEX.ReferenceDirection: 1 , # Reference direction (1: inwards, 0: outtwards)
			EGH_INDEX.MaintenanceInterval: 4 , # Maintenance interval
			EGH_INDEX.MaximalStroke: 4,   # maximal stroke
			EGH_INDEX.SystemCommand: 1,
			EGH_INDEX.DeviceStatus: 1,
			EGH_INDEX.DetailDeviceStatus: 3,
			EGH_INDEX.DetailLastError: 1,
			EGH_CODE.Position: 4,
			EGH_CODE.ActualCurrent: 4,
			EGH_CODE.ActualVoltage: 4,
			EGH_CODE.ActualTemp: 4,
			EGH_CODE.SensorSwitch1: 1,
			EGH_CODE.SensorSwitch2: 1,
			EGH_CODE.ErrorCounter: 2,
			EGH_CODE.CycleCounter: 4,
			EGH_CODE.ImpulseCounter: 4,
			EGH_CODE.MaintenanceCounter: 4
		}
		#return switcher.get(index*256 + subindex,0)
		return switcher.get(index)

