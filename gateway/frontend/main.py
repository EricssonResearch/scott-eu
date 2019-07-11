import time
import zmq
import json

BIND_ADDR = "tcp://127.0.0.1:5557"

def producer(addr):
    """Send JSON messages via ZMQ.PUSH

    Bind on the given addr and send 20000 messages.
    """
    context = zmq.Context()
    zmq_socket = context.socket(zmq.DEALER)
    zmq_socket.bind(addr)
    # Start your result manager and workers before you start your producers
    for num in range(20000):
        work_message = { 'num' : num }
        zmq_socket.send_json(work_message)
        print("Sent message: " + to_json(work_message))
        try:
            reply = zmq_socket.recv_json(flags=zmq.NOBLOCK)
            print("Message received:", reply)
        except zmq.Again as e:
            #print("No message received yet")
            pass


def to_json(msg):
    """Pretty-print a dict as a JSON string

    Use Unicode and 2-space indents.
    """
    return json.dumps(msg, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    # the script was invoked directly from the command line
    print("Pushing to " + BIND_ADDR)
    producer(BIND_ADDR)
