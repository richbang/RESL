import socket
import re
localIP = "192.168.12.9"

localPort = 5000

bufferSize = 1024

cnt = 0

msgFromServer = "Hello UDP Client"

bytesToSend = str.encode(msgFromServer)

# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))

print("UDP server up and listening")

# Listen for incoming datagrams
total_count=0
while (total_count<=10000):

    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

    message = bytesAddressPair[0]
    if (cnt == 0):
        list1 = str(message)
        list1 = str(list1[4:])
        list1 = list1.replace("'",'')
        list1 = float(list1)
        address1 = bytesAddressPair[1]

        cnt += 1

        file = open("C:/Users/OWNER/Desktop/rtls_log/log.txt", 'a')
        file.write(str(list1))
        file.write(' ')
        file.close()

    else:
        list2 = str(message)
        list2 = str(list2[4:])
        list2 = re.sub(r'[^0-9.]', '', list2)
        #list2 = list2.replace("'",'')
        list2 = float(list2)
        list2 = f'{list2:.3f}'
        address2 = bytesAddressPair[1]

       # clientIP = "Client IP Address:{}".format(address2)

        cnt = 0

        file = open("C:/Users/OWNER/Desktop/rtls_log/log.txt", 'a')
        file.write(str(list2))
        file.write('\n')
        clientMsg = "Message from Client: X : {}, Y : {}".format(list1, list2)
        print(clientMsg)
        file.close()

        total_count+=1