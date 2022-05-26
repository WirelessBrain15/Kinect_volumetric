import socket
import sys

HEADER = 1024
FORMAT = 'utf-8'
PORT = 8080
IP = '192.168.2.4'
ADDR = (IP, PORT)

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket successfully created . . . ")
except socket.error as err:
    print("Socket creation failed with error . . . ")


s.connect(ADDR)
# s.listen(5)
print("successfully connected . . . ") 
connected = True

while connected is True:
    # conn,addr = s.accept()
    data = s.recv(HEADER).decode()
    print("banana")
    if not data:
        connected = False
    print("recieved data ...", data.decode(FORMAT))

s.close()