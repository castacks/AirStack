import socket
import threading
import time
import struct

sim_start_time = time.time()


def get_sim_time():
    return (time.time() - sim_start_time)/2.

def handle_client(conn, addr):
    print(f'Connected by {addr}')

    initial_sitl_time = -1.
    initial_sim_time = -1.
    
    with conn:
        while True:
            data = conn.recv(8)
            print('data', data)
            if not data:
                break

            t = struct.unpack('Q', data)[0]
            s = get_sim_time()

            if initial_sitl_time < 0:
                print('initial')
                initial_sitl_time = t
                initial_sim_time = s

            sitl_time = t - initial_sitl_time
            sim_time = (s - initial_sim_time)*1000000
            time_to_sleep = int(sitl_time - sim_time)
            print('time to sleep', time_to_sleep, sitl_time, sim_time)
            
            #print(f'Received {data.decode()} from {addr}')
            #time.sleep(0.01)
            #conn.sendall(b'Hello from server')
            conn.sendall(struct.pack('i', time_to_sleep))

def start_server(host='127.0.0.1', port=65432):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f'Server listening on {host}:{port}')
        while True:
            conn, addr = s.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.start()
            print(f'Active connections: {threading.active_count() - 1}')

if __name__ == "__main__":
    start_server()
