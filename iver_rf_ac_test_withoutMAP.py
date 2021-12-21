import threading
import datetime
import serial
from queue import Queue
import MRC_Iver

filepath = r"C:\Log_files"
time_now = datetime.datetime.now()
fileName = filepath + '\\' + "log-RF-AC-test" + str(time_now.year) + str(time_now.month) + str(time_now.day) + \
           str(time_now.hour) + str(time_now.minute) + ".txt"
log_file = open(fileName, "a")

TIMEOUT_RF = 1
TIMEOUT_AC = 1

rf_port = str(input('RF COMPort: '))  # 'COM2'
ac_port = str(input('AC COMPort: '))  # 'COM5'
ser_rf = serial.Serial(rf_port, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=TIMEOUT_RF, xonxoff=0)
ser_ac = serial.Serial(ac_port, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=TIMEOUT_AC, xonxoff=0)

# iver = '3072'
iver = str(input('Which IVER: 3072/3089: '))

q_log = Queue()
ac_i, rf_i = 1, 1
send_through_ac_every = int(input('How often send OSD through AC in sec: '))
send_through_RF_every = int(input('How often send OSD through RF in sec: '))

event_iver = threading.Event()


def read_ac():
    """ Reading AC port """
    # print(datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S"), ": Start")
    ser_ac.reset_input_buffer()
    send_through_ac()
    osi_rec, osd_ak = 0, 0
    while True:
        try:
            frm_iver = ser_ac.readline().decode()
            if len(frm_iver) > 1:
                q_log.put([datetime.datetime.now().strftime("%H:%M:%S.%f"), ': AC: ', frm_iver])
                if MRC_Iver.received_stream(frm_iver) == 'osi':
                    osi_return = MRC_Iver.osi(frm_iver)
                    if MRC_Iver.osi(frm_iver) is not None:
                        print(datetime.datetime.now(), ': AC: lat:', osi_return['Latitude'],
                              'lng: ', osi_return['Longitude'], 'speed:', osi_return['Speed'],
                              'Battery: ', osi_return['Battery'])
                        q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', osi_return])
                        print(datetime.datetime.now(), f': OSI received AC: {osi_rec} / requested: {ac_i}')
                        osi_rec += 1
                elif MRC_Iver.received_stream(frm_iver) == 'osdAck':
                    if MRC_Iver.osd_ack(frm_iver) == 0:
                        q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', 'OSD Ack', osd_ak])
                        print(datetime.datetime.now(), ': OSI Ack received AC', osd_ak)
                        osd_ak += 1
        except Exception as e:
            q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', e])
            ser_ac.reset_input_buffer()
            continue


def read_rf():
    """Read RF port"""
    # print(datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S"), ": Start")
    ser_rf.reset_input_buffer()
    # inst_snd = '$AC;Iver3-' + iver + ';' + '$' + MRC_Iver.osd() + '\r\n'
    # ser_rf.write(inst_snd.encode())
    send_through_rf()
    osi_rec, osd_ak = 0, 0
    while True:
        try:
            frm_iver = ser_rf.readline().decode()
            if len(frm_iver) > 1:
                q_log.put([datetime.datetime.now().strftime("%H:%M:%S.%f"), ': RF: ' + frm_iver])
                if MRC_Iver.received_stream(frm_iver) == 'osi':
                    osi_return = MRC_Iver.osi(frm_iver)
                    if MRC_Iver.osi(frm_iver) is not None:
                        print(datetime.datetime.now(), ': RF: lat:', osi_return['Latitude'],
                              'lng: ', osi_return['Longitude'], 'speed:', osi_return['Speed'],
                              'Battery: ', osi_return['Battery'])
                        q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', osi_return])
                        print(datetime.datetime.now(), f': OSI received RF: {osi_rec} / requested: {rf_i}')
                        osi_rec += 1
                elif MRC_Iver.received_stream(frm_iver) == 'osdAck':
                    if MRC_Iver.osd_ack(frm_iver) == 0:
                        q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', 'OSD Ack RF', osd_ak])
                        print(datetime.datetime.now(), ': OSI Ack received RF ', osd_ak)
                        osd_ak += 1
        except Exception as e:
            q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', e])
            ser_ac.reset_input_buffer()
            continue


def send_through_rf():
    # send_through_RF_every = int(input('How often send OSD through RF in sec: '))
    threading.Timer(send_through_RF_every, send_through_rf).start()
    inst_snd = '$AC;Iver3-' + iver + ';' + '$' + MRC_Iver.osd() + '\r\n'
    ser_rf.reset_output_buffer()
    ser_rf.write(inst_snd.encode())
    global rf_i
    print(datetime.datetime.now(), ': Sending through RF: ', rf_i)
    q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ': send trough RF: ', rf_i])
    rf_i += 1


def send_through_ac():
    # send_through_ac_every = int(input('How often send OSD through AC in sec: '))
    threading.Timer(send_through_ac_every, send_through_ac).start()
    inst_snd = '$AC;Iver3-' + iver + ';' + '$' + MRC_Iver.osd() + '\r\n'
    ser_ac.reset_output_buffer()
    ser_ac.write(inst_snd.encode())
    global ac_i
    print(datetime.datetime.now(), ': Sending through AC: ', ac_i)
    q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ': send trough AC: ', ac_i])
    ac_i += 1


def log_data():
    while True:
        log = q_log.get()
        log_file.write(str(log) + '\n')
        log_file.flush()


t_rf = threading.Thread(target=read_rf, daemon=True)
t_ac = threading.Thread(target=read_ac, daemon=True)
t_log = threading.Thread(target=log_data, daemon=True)
t_rf.start()
t_ac.start()
t_log.start()
t_rf.join()
t_ac.join()
t_log.join()



