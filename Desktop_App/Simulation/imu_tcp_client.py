import socket

HOST = "0.0.0.0"
PORT = 3333

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.bind((HOST, PORT))
srv.listen(1)

print("Waiting for ESP...")
conn, addr = srv.accept()
print("Connected:", addr)

while True:
    data = conn.recv(1024)
    if not data:
        break
    print(data.decode(errors="ignore"))
