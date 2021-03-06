import json
import serial
import time

port = serial.Serial(port='/dev/ttyUSB0', baudrate=9600)

if __name__ == "__main__":
    time.sleep(1)
    port.flushOutput()
    send_dict = {
        "power": 0.0,
        "turn": 0.0
    }
    K = 1
    runs = 0
    while True:
        # Get test output
        output = json.dumps(send_dict).encode()
        print(output)

        # Serial write
        port.write(output)

        # Serial Read
        data = port.read_until()
        decoded_data = data.decode('utf-8','ignore')
        # print(data.decode('utf-8','ignore').encode("utf-8"))
        print(decoded_data)

        # sensor_json = json.loads(decoded_data)
        # yaw = sensor_json['yaw']
        # print(f"Yaw {runs}: {yaw}")
        # runs += 1

        # time.sleep(1)

        # Update the payload
        # send_dict["power"] += 0.01 * K
        # if send_dict["power"] > 0.5:
        #     K = -1
        # elif send_dict["power"] < -0.5:
        #     K = 1

        # port.write("{\"power\":0.0, \"turn\":0.0}".encode())
        # data = port.read_until()
        # print(data)

    port.close()
