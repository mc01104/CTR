import serial
import socket
from time import sleep

# initialize serial communication
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM3'
ser.open()

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('localhost', 10000)
sock.bind(server_address)
print 'starting up on %s port %s' % sock.getsockname()
sock.listen(1)


def run(socket, serial):
    import sys
    print >> sys.stderr, 'waiting for a connection'
    connection, client_address = socket.accept()
    counter = 0
    while True:
        data_send = serial.readline()
        if not data_send:
            continue
        #data_send = "%d\n" % counter
        print counter
        print data_send
        data_recv = connection.recv(16)
        counter = counter + 1
        if data_recv:
            connection.sendall(data_send)
        sleep(0.5)

    #return run(socket, serial)

if __name__ == '__main__':
    run(sock, ser)

