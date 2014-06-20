import serial,bluetooth,time,sys,tty,termios

_KheperaAddr = '00:07:80:85:A1:FF'
_command  = 'D,l100000,l100000'
_exit = False
_speed = 10000

def serialConnect():
	gauges = serial.Serial()
	gauges.port = '/dev/rfcomm0'
	gauges.baudrate = 9600
	gauges.parity = 'N'
	gauges.writeTimeout = 0
	gauges.open()
	print("Connected to " + gauges.portstr)
	gauges.write("D,l10000,l10000")

def bluetoothConnect():
	global _KheperaAddr
	while(True):
		try:
			socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
			socket.connect(( _KheperaAddr, 1))
			break;
		except bluetooth.btcommon.BluetoothError as error:
			socket.close()
			print "Could not connect: ", error, "; Retrying in 10s..."
			time.sleep(10)
	print "Connected to Khepera"
	return socket;

class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
		global _command
		global _exit
		global _speed
		inkey = _Getch()
		while(1):
			k=inkey()
			if k!='':break
		if k=='\x1b[A':
			_command  = 'D,l' + str(_speed) + ',l' + str(_speed)
			print "up"
		elif k=='\x1b[B':
			_command  = 'D,l' + str(_speed*-1) + ',l' + str(_speed*-1)
			print "down"
		elif k=='\x1b[C':
			_command  = 'D,l' + str(_speed) + ',l' + str(_speed*-1)
			print "right"
		elif k=='\x1b[D':
			_command  = 'D,l' + str(_speed*-1) + ',l' + str(_speed)
			print "left"
		else:
			_command  = 'D,l0,l0'
			_exit = True
			print "not an arrow key!"

def main():
	global _command
	global _exit
	global _KheperaAddr
	socket = bluetoothConnect()
	while(True):
		try:
			print "press a key to stop or use arrow keys ..."
			get()
			print "sending command: ", _command, " ..." 
			socket.send( _command + "\r\n")
			print "receiving ..."
			print socket.recv(1024)
			if(_exit):
				exit()
		except bluetooth.btcommon.BluetoothError as error:
			print "Caught BluetoothError: ", error
			time.sleep(5)
			socket = bluetoothConnect()
			pass
	socket.close()

if __name__=='__main__':
        main()
