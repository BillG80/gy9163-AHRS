import bluetooth

def start_bluetooth_server():
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server_sock.bind(("", bluetooth.PORT_ANY))
    server_sock.listen(1)

    port = server_sock.getsockname()[1]
    bluetooth.advertise_service(server_sock, "RPiServer",
                                service_id="00001101-0000-1000-8000-00805F9B34FB",
                                service_classes=["00001101-0000-1000-8000-00805F9B34FB", bluetooth.SERIAL_PORT_CLASS],
                                profiles=[bluetooth.SERIAL_PORT_PROFILE])

    print(f"Waiting for connection on RFCOMM channel {port}")

    client_sock, client_info = server_sock.accept()
    print(f"Accepted connection from {client_info}")

    try:
        while True:
            data = client_sock.recv(1024)
            if not data:
                break
            print(f"Received: {data.decode()}")
            client_sock.send(data)  # Echo back the received data
    except OSError:
        pass

    print("Disconnected.")
    client_sock.close()
    server_sock.close()

if __name__ == "__main__":
    start_bluetooth_server() 