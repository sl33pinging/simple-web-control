import socket as skt
import dance_template
from pymavlink import mavutil

#creates and returns a socket
def createSocket():
    #Protocol = UDB, addressFamily = ipv4
    protocol = skt.SOCK_STREAM
    addressFamily = skt.AF_INET

    #socket creation
    socket = skt.socket(addressFamily, protocol)

    #Bind information
    serverIp = "10.29.63.52"
    serverPrt = 6666

    #Binding port and ip
    socket.bind((serverIp, serverPrt))

    return socket

#logs request
def requestProcessor(request):
    #executing request
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    dance_template.arm_rov(mav_connection)
    # Arm the ROV and wait for confirmation
    dance_template.run_motors_timed(mav_connection, seconds = request, motor_settings=[-100, -100, 100, 100, 0, 0])

#create socket
socket = createSocket()

#receiving and processing data
while True:
    socket.listen()
    conn, addr = socket.accept() 
    #receiving data
    request = conn.recv(2048)
    #executes request
    requestProcessor(request)
    conn.close()
    
