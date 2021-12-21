from time import monotonic
import time
# import geopy.distance
import serial
import pandas as pd
import numpy as np
# import checkSum
import re
import warnings
import tkinter as tk
from geographiclib.geodesic import Geodesic
import numpy.polynomial.polynomial as poly
import pymap3d as pm
from scipy import stats

# import matplotlib.pyplot as plt
# import rasterio
# from rasterio.plot import show
# from tkinter import filedialog

root = tk.Tk()
root.withdraw()


def check_sum(instruction):
    """ Remove any newlines and $ and calculate checksum """
    if re.search("\n$", instruction):
        instruction = instruction[:-1]
    if re.search("\r$", instruction):
        instruction = instruction[:-1]
    if re.search("\$", instruction):
        instruction = instruction[1:]
    nmeadata, cksum = re.split('\*', instruction)
    calc_cksum = 0
    for s in nmeadata:
        calc_cksum ^= ord(s)
    """ Return the calculated checksum """
    return '{:02X}'.format(calc_cksum)


def received_stream(stream):
    if stream == '':
        # print(" Waiting")
        return 'None'
    else:
        if re.search("ACK", stream):
            acknowledgement = {'8': 'osdAck', '16': 'omwAck'}
            return acknowledgement[stream.split(';')[-1].split(',')[1]]
        elif re.search("OSI", stream):
            return 'osi'
        elif re.search("OSD", stream):
            return 'osd'
        elif re.search("OMW", stream):
            return 'omw'
        else:
            return 'not known keyword'


def omw_ack(stream):
    # print("omw_ack section: ", stream)
    rec_chksm = stream.split('*')[-1][0:3]
    cal_chksm = check_sum(stream.split(';')[-1][1:-2])
    if int(rec_chksm, 16) == int(cal_chksm, 16):
        # print('Right checksum')
        if int(stream.split(',')[2]) == 0:
            # print('The IVER has acknowledged the OMW command without an error')
            return 0
        else:
            # print(stream)
            print('The IVER has raised an error to execute the OMW command')
            return 1
    
    else:
        print('wrong checksum')
        print('Received checkSum: ' + rec_chksm + 'Calculated checkSum: ' + cal_chksm)


def osi(stream):
    try:
        # print(stream)
        if int(stream.split('*')[-1], 16) == int(check_sum(stream.split(';')[-1][1:-2]), 16):
            # print('Right checkSum')
            stream = stream.split(',')
            mode = {'N': 'Normal_UVC', 'S': 'Stopped', 'P': 'Parking',
                    'M': 'Manual_Override', 'mP': 'Manual_parking',
                    'A': 'Servo', 'W': 'Waypoint'}
            # print('Mode : {}'.format(mode[stream[2]]))
            # print('NextWp: ', stream[3])
            # print('Latitude: ', stream[4])
            # print('Longitude: ', stream[5])
            # print('Speed: {} Knots'.format(stream[6]))
            # print("Distance to next WP: {} meters".format(stream[7]))
            # print('Battery percent: ', stream[16])
            # osi_return = (stream[3], stream[4], stream[5], stream[6], stream[7], stream[16])
            osi_return = {'NextWp': stream[3], 'Latitude': float(stream[4]), 'Longitude': float(stream[5]),
                          'Speed': float(stream[6]), 'DistanceToNxtWP': float(stream[7]), 'Battery': float(stream[16])}
            # return mode[stream[2]], stream[3]
            return osi_return
        else:
            print('Wrong checkSum')
            print("Received checkSum: " + str(stream.split('*')[-1]) + 'Calculated checksum is : ' + str(
                check_sum(stream.split(';')[-1][1:-2])))
            print(" Wrong CheckSum: ", stream)
            # osi_return = {'NextWp': 0, 'Latitude': 00.00000, 'Longitude': 00.00000,
            #               'Speed': 0.00, 'DistanceToNxtWP': 0.00, 'Battery': 0.00}
            return None
    except Exception as osi_exception:
        print("Error: ", osi_exception)
        # osi_return = {'NextWp': 0, 'Latitude': 00.00000, 'Longitude': 00.00000,
        #               'Speed': 0.00, 'DistanceToNxtWP': 0.00, 'Battery': 0.00}
        return None


def osd():
    ins_osd = 'OSD,,,S,,,,,*'
    instruction = ins_osd + check_sum(ins_osd)
    return instruction


def osd_req_recvd(stream):
    stream = stream.strip()
    # print(stream)
    if int(stream.split('*')[-1], 16) == int(check_sum(stream.split(';')[-1][1:-2]), 16):
        # print(" right Check Sum")
        return 0
    else:
        print("wrong CheckSum")
        return 1


# def osd_3089():
#     ins_osd_3089 = 'OSD,,,S,,,,,*'
#     instruction = ins_osd_3089 + check_sum(ins_osd_3089)
#     return instruction


# def osd_3072():
#     ins_osd_3072 = 'OSD,,,S,,,,,*'
#     instruction = ins_osd_3072 + check_sum(ins_osd_3072)
#     return instruction


def osd_ack(stream):
    # print("osd_ack section: ", stream)
    rec_chksm = stream.split('*')[-1][0:3]
    cal_chksm = check_sum(stream.split(';')[-1][1:-2])
    if int(rec_chksm, 16) == int(cal_chksm, 16):
        # print('Right checksum')
        if int(stream.split(',')[2]) == 0:
            # print('The IVER has acknowledged the OSD command without an error.')
            return 0
        else:
            print('The IVER has raised an error to execute the OSD command')
            return 1
    
    else:
        print('wrong checksum')
        print('Received checkSum: ' + rec_chksm + 'Calculated checkSum: ' + cal_chksm)


def omw_stop():
    ins_omw_stop = 'OMW,STOP*'
    instruction = ins_omw_stop + check_sum(ins_omw_stop)
    return instruction


def omw_req_recvd(stream):
    # assert f"{stream} is not string"
    # $AC;Iver3-3089;$OMW,30.35197,-89.62897,0.0,,10,4.0,0, *64
    stream = stream.strip()
    # print(stream)
    if int(stream.split('*')[-1], 16) == int(check_sum(stream.split(';')[-1][1:-2]), 16):
        # print(" right Check Sum")
        return 0
    else:
        print("OMW Request Received: wrong CheckSum")
        return 1


def wamv_gpgll(stream):
    if int((stream.split('*')[-1]), 16) == int(check_sum(stream.split('*')[0] + '*'), 16):
        # print("right CheckSum")
        return 0
    else:
        print("Wamv_gpgll: Wrong CheckSum: received checkSum{}, calculated checkSum{}".format(
            stream.split('*')[-1], check_sum(stream.split('*')[0] + '*')))
        return 1


'''
def dd_ddm_nmea_lat(coordinates):
    # coordinates = float
    coordinates = str(coordinates)
    if re.search('-', coordinates):
        coordinates = coordinates.strip('-')
    coordinates = coordinates.split('.')
    co_return = ''.join((coordinates[0] + '.' + str(int(coordinates[-1]) * 60)).split('.'))
    co_return = co_return[:4] + '.' + co_return[4:]
    return co_return


def dd_ddm_nmea_lng(coordinates):
    # coordinates = float
    coordinates = str(coordinates)
    coordinates = '{}'.format(coordinates[1:] if coordinates.startswith('-') else coordinates)
    # if re.search('-', coordinates):
    #     coordinates = coordinates.strip('-')
    coordinates = coordinates.split('.')
    co_return = ''.join((coordinates[0] + '.' + str(int(coordinates[-1]) * 60)).split('.'))
    co_return = co_return.zfill(len(co_return) + 1)
    co_return = (co_return[:4] + '.' + co_return[4:])
    # print(co_return)
    return co_return


def ddm_dd_nmea_lat(coordinates):
    coordinates = ''.join(str(coordinates).split('.'))
    co_return = ''.join(coordinates[:2] + '.' + str(int(int(coordinates[2:]) / 60)))
    return co_return


def ddm_dd_nmea_lng(coordinates):
    coordinates = str(coordinates)
    coordinates = '{}'.format(coordinates[1:] if coordinates.startswith('0') else coordinates)
    # print((coordinates))
    coordinates = ''.join(coordinates.split('.'))
    co_return = coordinates[:2] + '.' + ''.join(str(float(coordinates[2:]) / 60).split('.'))
    # print(co_return)
    return co_return
'''


# ddm = degree, decimal minutes, dd = degree decimal
def ddm2dd(coordinates):
    """ Convert degree, decimal minutes to degree decimal; return 'Lat_dd': float(lat_dd), 'Lng_dd': float(lng_dd)}
    Input Ex.:  ['3020.1186383580', 'N', '0894.5222887340', 'W'],
    return: {'Lat_dd': float(lat_dd), 'Lng_dd': float(lng_dd)} """
    lat, lat_direction, lng, lng_direction = coordinates[0], coordinates[1], coordinates[2], coordinates[3]
    lat = ''.join(lat.split('.'))
    lat_ddm = lat[:2] + '.' + str(int(int(lat[2:]) / 60))
    lat_dd = '{}'.format('-' + lat_ddm if lat_direction == 'S' else lat_ddm)
    lng_ddm = ''.join('{}'.format(lng[1:] if lng.startswith('0') else lng).split('.'))
    is_zero = lng_ddm[2:].startswith('0')
    lng_ddm = lng_ddm[:2] + '.' + (str(int(int(lng_ddm[2:]) / 60)) if not is_zero else ('0' + str(int(int(lng_ddm[2:]) / 60))))
    lng_dd = '{}'.format('-' + lng_ddm if lng_direction == 'W' else lng_ddm)
    dd = {'Lat_dd': float(lat_dd), 'Lng_dd': float(lng_dd)}
    return dd


def dd2ddm(coordinates):
    """ Convert degree decimal to degree decimal minute;"""
    # lat, lng = ''.join(str(coordinates[0]).split('.')), ''.join(str(coordinates[1]).split('.'))
    lat, lng = ''.join(str(coordinates[0]).split('.')), str(coordinates[1])
    lat = lat[:2] + ''.join(str(int(int(lat[2:]) * 60)).split('.'))
    lat = lat[:4] + '.' + lat[4:]
    lat_ddm = lat
    lng_ddm = '{}'.format(lng[1:] if lng.startswith('-') else lng)
    lng_ddm = lng_ddm.split('.')
    is_zero = lng_ddm[1].startswith('0')
    # lng_ddm = lng_ddm[:2] + ''.join(str((int(int(lng_ddm[2:]) * 60))).split('.'))
    lng_ddm = lng_ddm[0] + '{}'.format('0' + str(int(lng_ddm[1])*60) if is_zero else str(int(lng_ddm[1])*60))
    lng_ddm = lng_ddm.zfill(len(lng_ddm) + 1)
    lng_ddm = lng_ddm[:4] + '.' + lng_ddm[4:]
    ddm = {'Lat_ddm': lat_ddm, 'N_S': 'S' if lat.startswith('-') else 'N',
           'Lng_ddm': lng_ddm, 'E_W': 'W' if lng.startswith('-') else 'E'}
    return ddm


def speed_ha_coordinates(coordinate1_withtimestamp, coordinate2_withtimestamp):
    # ['30.35059', '-89.62995', '104139']  # example coordinate_withtimestamp
    geod = Geodesic(6378388, 1 / 297.0)
    co1 = coordinate1_withtimestamp[0:2]
    co1_time = coordinate1_withtimestamp[-1]
    co2 = coordinate2_withtimestamp[0:2]
    co2_time = coordinate2_withtimestamp[-1]
    # l = geod.InverseLine(lat1, lng1, lat2, lng2)
    d = geod.Inverse(float(co1[0]), float(co1[1]), float(co2[0]), float(co2[1]))
    # print(d)
    distance = d['s12']
    ha = d['azi2']
    time_diff = (int(co2_time[0:2]) * 3600 + int(co2_time[2:4]) * 60 + int(co2_time[4:])) - \
                (int(co1_time[0:2]) * 3600 + int(co1_time[2:4]) * 60 + int(co1_time[4:]))
    # print(time_diff)
    # speed = distance / time_diff
    # result = {'speed': speed, 'ha': ha, 'dis12': distance}
    try:
        speed = distance / time_diff
        result = {'speed': speed, 'ha': ha, 'dis12': distance}
    except ZeroDivisionError:
        result = {'speed': 0, 'ha': ha, 'dis12': distance}
    return result


def distance_in_m(coordinate_1, coordinate_2):
    return str(round(geopy.distance.GeodesicDistance(coordinate_1, coordinate_2).m, 1))


def haSpeed_ply(df):
    """ fitting ha & speed, example input: [[30.35158, -89.6296, '104511'], [30.3516, -89.62957, '104513'],
     [30.35162, -89.62954, '104515']]  """
    df = pd.DataFrame(df, columns=['lat', 'lon', 't'])
    # df['dt'] = df.t.diff()
    df['latlng'] = df[['lat', 'lon', 't']].values.tolist()
    ha, speed = [], []
    for i in range(len(df.latlng) - 1):
        h = speed_ha_coordinates(df.latlng[i], df.latlng[i + 1])
        # ha.append(h['ha'] + 360 if h['ha'] < 0 else np.nan if h['ha'] == 0 else h['ha'])
        ha.append(np.absolute(h['ha']) if h['ha'] < 0 else np.nan if h['ha'] == 0 else h['ha'])
        speed.append(h['speed'])
    ha.append(0)
    speed.append(0)
    df['speed'] = speed
    df['ha'] = ha
    # print(df)
    df = df[df['speed'].notna()]
    df = df[df['ha'].notna()]
    df = df.drop(df.index[len(df) - 1])
    # df['speed_'] = df['speed'].apply(lambda x: np.abs(x - df['speed'].mean()) / df['speed'].std())
    # print(df)
    deg = 5
    np_speed = df['speed'].to_numpy(dtype=np.float32)
    np_ha = df['ha'].to_numpy(dtype=np.float32)
    np_t = df['t'].to_numpy(dtype=np.float32)
    x = np.linspace(0, len(np_ha), len(np_ha))
    warnings.simplefilter('ignore', np.RankWarning)
    model_ha = np.poly1d(np.polyfit(x, np_ha, deg=deg))
    line_speed = np.linspace(x[0], x[-1], num=len(x) * 10)
    predict_ha = model_ha(len(np_ha))
    
    x = np.linspace(0, len(np_speed), len(np_speed))
    model_speed = np.poly1d(np.polyfit(x, np_speed, deg=deg))
    line_speed = np.linspace(x[0], x[-1], num=len(x) * 10)
    p_speed = model_speed(len(np_speed))
    
    coefs = poly.polyfit(x, np_speed, 4)
    x_new = np.linspace(x[0], x[-1], num=len(x) * 10)
    ffit = poly.polyval(x_new, coefs)
    result = {'speed': p_speed, 'ha': predict_ha}
    return result


def point_on_line(a, b, p):
    ap = p - a
    ab = b - a
    result = a + np.dot(ap, ab) / np.dot(ab, ab) * ab
    return result


def coordinate_fit(df, deg=1):
    """ Fitting coordinates; takes coordinates and return 2 coordinates with timestamp. Return
    [[lat1, lng1, t1], [lat2, lng2, t2]"""
    df = pd.DataFrame(df, columns=['lat', 'lon', 't'])
    df['h'] = df.apply(lambda h: 0, axis=1)
    ' Convert to cartesian coordinate'
    x_c, y_c, Az = [], [], []
    for i in range(len(df.lat)):
        l, n, a = pm.geodetic2enu(df.lat[i], df.lon[i], df.h[i], df.lat[0], df.lon[0], df.h[0], ell=None, deg=True)
        x_c.append(l)
        y_c.append(n)
    df['x_c'], df['y_c'] = x_c, y_c
    # df['Azm'] = Az
    m, n = df['x_c'], df['y_c']
    p1, p2 = np.array([df.x_c[0], df.y_c[0]]), np.array([df.x_c[len(df.x_c) - 1], df.y_c[len(df.x_c) - 1]])
    
    ' Apply linear regression'
    slope, intercept, r_value, p_value, std_err = stats.linregress(m, n)
    
    # def linefitline(x):
    # 	return intercept + slope * x
    #
    # line = linefitline(m)
    line = list(map(lambda b: intercept + slope * b, m))
    df['line'] = line
    'perpendicular on the '
    a = np.array([df['x_c'][0], df['line'][0]])
    b = np.array([df.x_c[len(df['x_c']) - 1], df.line[len(df['line']) - 1]])
    x_1, y_1 = point_on_line(a, b, p1)
    x_2, y_2 = point_on_line(a, b, p2)
    'Back to lat lng'
    lat1, lng1, _h1 = pm.enu2geodetic(x_1, y_1, df.h[0], df.lat[0], df.lon[0], df.h[0], ell=None, deg=True)
    lat2, lng2, _h2 = pm.enu2geodetic(x_2, y_2, df.h[0], df.lat[0], df.lon[0], df.h[0], ell=None, deg=True)
    result = [[lat1, lng1, df.t[0]], [lat2, lng2, df.t[len(df.t) - 1]]]
    
    # x = np.linspace(df.lon[0], df.lon[len(df['lon']) - 1], num=len(df['lon']) * 15)
    # ffit = poly.polyval(x, poly.polyfit(df['lon'], df['lat'], deg=deg))
    # lat, lng = ffit[-1], df.lon[len(df['lon']) - 1]
    # # result = {'lat': lat, 'lng': lng, 't': df.t[len(df['t']) - 1]}
    # result = [lat, lng, df.t[len(df['t']) - 1]]
    return result


def iver_status(iver='3089', port_rf='com7', port_ac='com4', time_out=1, time_wait_ac=14):
    try:
        osi_return = {'NextWp': 0, 'Latitude': 00.00000, 'Longitude': 00.00000,
                      'Speed': 0.00, 'DistanceToNxtWP': 0.00, 'Battery': 0.00}
        # time_wait = 14  # fetch this data from iverVariables.txt, we get responses after 12 sec.
        t_start = monotonic()
        count = 0
        try:
            ser_rf = serial.Serial(port_rf, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=time_out,
                                   xonxoff=0)
            ser_rf.reset_output_buffer()
        except Exception as e_rf:
            print("I am in the RF com port exception block.", e_rf)
            print("Will send through ACOMM...... ")
            ser_rf_open = 'notOpen'
        else:
            ser_rf_open = 'open'
        try:
            ser_ac = serial.Serial(port_ac, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=time_out,
                                   xonxoff=0)
            ser_ac.reset_output_buffer()
        except Exception as e_ac:
            print("I am in the AC com port exception block. ", e_ac)
            ser_ac_open = 'notOpen'
        else:
            ser_ac_open = 'open'
        break_innerloop = 'no'
        while ser_rf_open == 'open' or ser_ac_open == 'open':
            try:
                time.sleep(1)
                if ser_rf_open == 'open':
                    inst_snd = '$AC;Iver3-' + iver + ';' + '$' + osd() + '\r\n'
                    ser_rf.reset_output_buffer()
                    ser_rf.write(inst_snd.encode())
                    frm_iver = ser_rf.readline().decode()
                    frm_iver_OSDAck = ser_rf.readline().decode()
                if ser_rf_open == 'open' and len(frm_iver) >= 1:  # if rf com port is open and responded through rf comm
                    if received_stream(frm_iver_OSDAck) == 'osdAck' and osd_ack(frm_iver_OSDAck) == 0:
                        if received_stream(frm_iver) == 'osi':
                            osi_return = osi(frm_iver)
                            break
                    else:
                        count += 1
                # something other than osdAck and osi
                elif ser_ac_open == 'open' and (ser_rf_open == 'notOpen' or len(frm_iver) < 1) and \
                        monotonic() - t_start >= time_wait_ac:  # send osd through ac comm
                    # print("Sent")
                    inst_snd = '$AC;Iver3-' + iver + ';' + '$' + osd() + '\r\n'
                    ser_ac.write(inst_snd.encode())
                    print("Sent")
                    i = 0
                    print("waiting for a response from the Iver...")
                    while True:
                        frm_iver = ser_ac.readline().decode()
                        frm_iver_OSDAck = ser_ac.readline().decode()
                        # print(frm_iver, frm_iver_OSDAck)
                        if received_stream(frm_iver_OSDAck) == 'osdAck' and osd_ack(frm_iver_OSDAck) == 0:
                            if received_stream(frm_iver) == 'osi':
                                osi_return = osi(frm_iver)
                                # print(osi_return)
                                t_start = monotonic()
                                i = 0
                                break_innerloop = 'yes'
                                break
                        else:
                            print(i)
                            i += 1
                            if i == 15:  # wait 15 sec to get the response from the iver through AC;can fetch data from file
                                break
                            else:
                                continue
                else:
                    if break_innerloop == 'yes':
                        break
                    else:
                        continue
            except Exception as loop:
                print("I am in the exception loop block.", loop)
                ser_rf.reset_input_buffer()
                ser_ac.reset_input_buffer()
                continue
        
        return osi_return
    except Exception as iverStatus:
        return osi_return

# def listen_iver(iver='3089', read_port='b', port_rf='com7', port_ac='com6', time_out=1):
#     try:
#         if read_port == 'b':
#             try:
#                 ser_rf = serial.Serial(port_rf, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=time_out,
#                                        xonxoff=0)
#                 ser_rf.reset_output_buffer()
#             except Exception as e_rf:
#                 print("I am in the RF com port exception block.", e_rf)
#                 ser_rf_open = 'notOpen'
#             else:
#                 ser_rf_open = 'open'
#             try:
#                 ser_ac = serial.Serial(port_ac, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=time_out,
#                                        xonxoff=0)
#                 ser_ac.reset_output_buffer()
#             except Exception as e_ac:
#                 print("I am in the AC com port exception block. ", e_ac)
#                 ser_ac_open = 'notOpen'
#             else:
#                 ser_ac_open = 'open'
#         elif read_port == 'r':
#             print("Rread RF only: ")
#         elif read_port == 'a':
#             print(" Reading ACom port")
#         else:
#             print("Wrong request")
#     except Exception as e_listen:
#         print("Error: ", e_listen)
#         return "None"
