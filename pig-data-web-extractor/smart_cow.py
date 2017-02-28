from datetime import datetime
import requests
import serial

arr = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F"]
s = "@"
e = "*"
separator = ","

array = []


def check_value(arr, val):
    if val in arr:
        # print  "INSERT"
        return "INSERT"
    elif val == s:
        # print "START"
        return "START"
    elif val == e:
        # print "END"
        return "END"
    elif val == separator:
        return "SEP"
    else:
        # print "ZERO"
        return "ZERO"


j = 0
start_f = 0
stop_f = 0
zero = 0
insert_value = ""
array_msg = []
ser = serial.Serial("/dev/ttyAMA0")
ser.baudrate = 115200
jsonResp = dict()
from messure_converter import convert_tab_of_messures, messure_converter, get_messures_from_array


convert = messure_converter()
while ser.isOpen():
    data = ser.read(1)
    # print "received data: " + data
    # print datetime.utcnow().isoformat(), data


    parse = check_value(arr, data)
    if parse == "START":
        start_f += 1
        if start_f == 1:
            stop_f = 0
            array_msg = []
            insert_value = ""
            j = 0

    if parse == "END":
        stop_f += 1

    if stop_f == 1:
        if (len(array_msg) == 18):
            print "poprawna paczka"
            # print array_msg
            array_to_send = ",".join(array_msg)
            print array_to_send
            r = requests.post("http://207.154.204.63/api/v1/measurements/add_measurement_v3", "" + array_to_send)

            mock_tab = convert_tab_of_messures(array_to_send, convert)
            mock_json = get_messures_from_array(mock_tab)
            print ("mock json" + str(mock_json));

            r = requests.post("http://207.154.204.63//api/v1/float_measurements", "" + mock_json)


            print(r.status_code, r.reason)
        insert_value = ""
        array_msg = []
        start_f = 0
        j = 0

    if parse == "SEP":
        j += 1
        insert_value = ""

    if parse == "INSERT":
        if j == 0:
            insert_value += str(data)
            # print("value: " + insert_value)
        elif j > 0:
            j = 0
            insert_value = ""
            insert_value += str(data)
            # print("value j>0: " + insert_value)

    if len(insert_value) == 2:
        if ((start_f == 1) and (stop_f == 0)):
            # print("insert: " + insert_value)
            array_msg.append(insert_value)

    if ((len(array_msg) == 18) and (stop_f == 1)):
        print "poprawna paczka"
        print array_msg
        array_to_send = "".join(array_msg)
        print array_to_send
        array_msg = []
        stop_f = 0
        start_f = 0
        j = 0