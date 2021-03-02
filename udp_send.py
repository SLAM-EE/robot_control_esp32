import socket
import sys
from pynput import keyboard

def on_press(key):
    try:
    	data_in = key.char
    	print('alphanumeric key {0} pressed'.format(data_in))
    	MESSAGE = bytes(data_in, 'utf-8')
    	sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
	
    except AttributeError:
        print('special key {0} pressed'.format(key))

if len(sys.argv) != 3 :
	sys.stdout.write("Error : The correct format is python udp_send.py server_ip server_port \n")
	sys.exit(1)
else:
	UDP_IP = sys.argv[1]
	UDP_PORT = int(sys.argv[2])
	
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()

sock.close()
