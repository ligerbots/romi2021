
from multiprocessing import Process, Queue
from .aruco import process_image, process_image_2d
from queue import Empty
import time
import socket
import sys
import traceback

task_queue = Queue()
result_queue = Queue()

def task_proc(task_q,result_q):
    while True:
        timestamp, image = task_q.get()
        output_image, pose = process_image_2d(image)
        result_q.put((timestamp,pose,output_image))

def process_frame(timestamp, image):
    task_queue.put((timestamp, image))

for i in range(3):
    p = Process(target=task_proc, args=(task_queue, result_queue))
    p.daemon=True
    p.start()

HOST = '10.0.0.120' #'10.0.0.125'
PORT = 7000

print("HOST =",HOST,"PORT =",PORT)
def loop(sink, output_stream):
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST, PORT))

                time_offset_millis = 0
                def dosync():
                    nonlocal time_offset_millis, s
                    start_time = time.time()

                    s.sendall(b'sync\n')
                    remote_time = s.recv(1024)

                    end_time = time.time()
                    local_time_millis = round((start_time+end_time)/2*1000)

                    remote_time_int = int(remote_time.decode("ascii"))

                    time_offset_millis = remote_time_int-local_time_millis
                def send(timestamp, pose):
                    nonlocal time_offset_millis, s

                    s.sendall(b'data\n')
                    s.sendall(str(round(timestamp/1000+time_offset_millis)).encode("ascii", "ignore")+b"\n")
                    s.sendall(str(len(pose)).encode("ascii", "ignore")+b"\n")
                    for x in pose:
                        s.sendall(str(x).encode("ascii", "ignore")+b"\n")
                input_img = None

                framecount = 0
                lastfpslog = time.time()
                dosync()
                while True:
                    timestamp, input_img = sink.grabFrame(input_img)

                    if input_img is not None:
                        if(task_queue.qsize()==0):
                            process_frame(timestamp, input_img)
                        while True:
                            try:
                                timestamp, pose, output_img = result_queue.get_nowait()
                                if pose is not None:
                                    send(timestamp, pose)
                                    framecount+=1
                                    output_stream.putFrame(output_img)
                            except Empty:
                                break
                    if(lastfpslog+1<time.time()):
                        dosync()
                        print("fps:", framecount)
                        framecount=0
                        lastfpslog=time.time()

        except Exception:
            traceback.print_exc()
            time.sleep(1)
