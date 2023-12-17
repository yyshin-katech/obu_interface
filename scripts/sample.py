import socket
import selectors
import threading
import json

# JSON 데이터 생성
data = {
    'name': 'John',
    'age': 30,
    'city': 'New York'
}
json_data = json.dumps(data)  # JSON 문자열로 변환

# TCP 클라이언트 설정
host = '서버 IP 주소'  # 실제 서버의 IP 주소로 변경해야 합니다.
port = 12345  # 실제 서버의 포트 번호로 변경해야 합니다.

# 송신 쓰레드 클래스
class SendThread(threading.Thread):
    def __init__(self, client_socket):
        super().__init__()
        self.client_socket = client_socket

    def run(self):
        self.client_socket.sendall(json_data.encode())

# 수신 쓰레드 클래스
class RecvThread(threading.Thread):
    def __init__(self, client_socket):
        super().__init__()
        self.client_socket = client_socket

    def run(self):
        selector = selectors.DefaultSelector()
        selector.register(self.client_socket, selectors.EVENT_READ)

        while True:
            events = selector.select()
            for key, _ in events:
                if key.fileobj == self.client_socket:
                    response = self.client_socket.recv(1024)
                    if response:
                        print('서버로부터 받은 응답:', response.decode())
                    else:
                        selector.unregister(self.client_socket)
                        return

# 메인 쓰레드
def main():
    # TCP 클라이언트 연결
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))

    # 송신 쓰레드 시작
    send_thread = SendThread(client_socket)
    send_thread.start()

    # 수신 쓰레드 시작
    recv_thread = RecvThread(client_socket)
    recv_thread.start()

    # 메인 쓰레드 종료 대기
    send_thread.join()
    recv_thread.join()

    # TCP 클라이언트 종료
    client_socket.close()

if __name__ == '__main__':
    main()