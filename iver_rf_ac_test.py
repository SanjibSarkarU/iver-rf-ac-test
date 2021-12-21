import threading
import datetime
import tkinter as tk
import re
from time import monotonic

from rasterio.plot import show
import rasterio
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib import animation
import matplotlib.pyplot as plt
from queue import Queue
from geographiclib.geodesic import Geodesic
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

iver = '3072'

send_through_rf_every = int(input('How often send OSD through RF in sec: '))
send_through_ac_every = int(input('How often send OSD through AC in sec: '))


class App(tk.Frame):
    def __init__(self, master=None, **kwargs):
        tk.Frame.__init__(self, master, **kwargs)
        
        self.running = False
        self.ani = None
        self.q_plot, self.q_log = Queue(), Queue()
        self.ac_i, self.rf_i = 1, 1

        self.xdata_iver, self.ydata_iver = [], []
        
        self.event_iver = threading.Event()
        self.latlng_iver = []
        self.inst_snd = None
        btns = tk.Frame(self)
        btns.pack()
        
        lbl = tk.Label(btns, text="update interval (ms)")
        lbl.pack(side=tk.LEFT)
        
        self.interval = tk.Entry(btns, width=5)
        self.interval.insert(0, '50')
        self.interval.pack(side=tk.LEFT)
        
        self.btn = tk.Button(btns, text='Start', command=self.on_click)
        self.btn.pack(side=tk.LEFT)
        
        self.btn_exit = tk.Button(btns, text='Exit', command=quit)
        self.btn_exit.pack(side=tk.LEFT)
        
        self.fig = plt.Figure()
        self.ax1 = self.fig.add_subplot(111)
        line_width = 0.9
        
        self.line_iver, = self.ax1.plot([], [], 'r-', lw=line_width)
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        
        img = rasterio.open('Stennis_QW.tif')  # 'Cat_Island_Low.tif', 'Stennis_QW.tif'
        x = show(img, ax=self.ax1)
        self.canvas.get_tk_widget().pack(expand=True)
        self.canvas.figure.tight_layout()
        self.geod = Geodesic(6378388, 1 / 297.0)
    
    def on_click(self):
        if self.ani is None:
            return self.start()
        if self.running:
            self.ani.event_source.stop()
            self.btn.config(text='following')
        else:
            self.ani.event_source.start()
            self.btn.config(text='Pause')
        self.running = not self.running
    
    def start(self):
        threading.Thread(target=self.read_ac, daemon=True).start()
        threading.Thread(target=self.read_rf, daemon=True).start()
        threading.Thread(target=self.log_data, daemon=True).start()
        
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update_graph,
            # frames=self.points,
            interval=int(self.interval.get()),
            repeat=False,
            blit=True)
        self.running = True
        self.btn.config(text='Pause')
        # self.ani._start()
    
    def read_ac(self):
        """ Reading AC port """
        # print(datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S"), ": Start")
        ser_ac.reset_input_buffer()
        # inst_snd = '$AC;Iver3-' + iver + ';' + '$' + MRC_Iver.osd() + '\r\n'
        # ser_ac.write(inst_snd.encode())
        self.send_through_ac()
        osi_rec, osd_ak = 0, 0
        while True:
            try:
                frm_iver = ser_ac.readline().decode()
                if len(frm_iver) > 1:
                    self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S.%f"), ': AC: ', frm_iver])
                    if MRC_Iver.received_stream(frm_iver) == 'osi':
                        osi_return = MRC_Iver.osi(frm_iver)
                        if MRC_Iver.osi(frm_iver) is not None:
                            print(datetime.datetime.now(), ': AC: lat:', osi_return['Latitude'],
                                  'lng: ', osi_return['Longitude'], 'speed:', osi_return['Speed'],
                                  'Battery: ', osi_return['Battery'])
                            self.q_plot.put({'lat': osi_return['Latitude'], 'lon': osi_return['Longitude'],
                                             'key': 'iver'})
                            self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', osi_return])
                            print(datetime.datetime.now(), f': OSI received AC: {osi_rec} / requested: {self.ac_i}')
                            osi_rec += 1
                    elif MRC_Iver.received_stream(frm_iver) == 'osdAck':
                        if MRC_Iver.osd_ack(frm_iver) == 0:
                            self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', 'OSD Ack', osd_ak])
                            print(datetime.datetime.now(), ': OSI Ack received AC', osd_ak)
                            osd_ak += 1
            except Exception as e:
                self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', e])
                ser_ac.reset_input_buffer()
                continue
    
    def read_rf(self):
        """Read RF port"""
        # print(datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S"), ": Start")
        ser_rf.reset_input_buffer()
        # inst_snd = '$AC;Iver3-' + iver + ';' + '$' + MRC_Iver.osd() + '\r\n'
        # ser_rf.write(inst_snd.encode())
        self.send_through_rf()
        osi_rec, osd_ak = 0, 0
        while True:
            try:
                frm_iver = ser_rf.readline().decode()
                if len(frm_iver) > 1:
                    self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S.%f"), ': RF: ', frm_iver])
                    if MRC_Iver.received_stream(frm_iver) == 'osi':
                        osi_return = MRC_Iver.osi(frm_iver)
                        if MRC_Iver.osi(frm_iver) is not None:
                            print(datetime.datetime.now(), ': RF: lat:', osi_return['Latitude'],
                                  'lng: ', osi_return['Longitude'], 'speed:', osi_return['Speed'],
                                  'Battery: ', osi_return['Battery'])
                            self.q_plot.put({'lat': osi_return['Latitude'], 'lon': osi_return['Longitude'],
                                             'key': 'iver'})
                            self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', osi_return])
                            print(datetime.datetime.now(), f': OSI received RF: {osi_rec} / requested: {self.rf_i}')
                            osi_rec += 1
                    elif MRC_Iver.received_stream(frm_iver) == 'osdAck':
                        if MRC_Iver.osd_ack(frm_iver) == 0:
                            self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', 'OSD Ack RF', osd_ak])
                            print(datetime.datetime.now(), ': OSI Ack received RF ', osd_ak)
                            osd_ak += 1
            except Exception as e:
                self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ':', e])
                ser_ac.reset_input_buffer()
                continue
    
    def update_graph(self, i):
        plot_inbox = self.q_plot.get()
        print(plot_inbox)
        if plot_inbox['key'] == 'iver':
            self.xdata_iver.append(plot_inbox['lon'])
            self.ydata_iver.append(plot_inbox['lat'])
            self.line_iver.set_data(self.xdata_iver, self.ydata_iver)
        return self.line_iver,
    
    def send_through_rf(self):
        # send_through_ac_every = 15
        threading.Timer(send_through_rf_every, self.send_through_rf).start()
        inst_snd = '$AC;Iver3-' + iver + ';' + '$' + MRC_Iver.osd() + '\r\n'
        ser_rf.reset_output_buffer()
        ser_rf.write(inst_snd.encode())
        print(datetime.datetime.now(), ': Sending through RF: ', self.rf_i)
        self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ': send trough RF: ', self.rf_i])
        self.rf_i += 1
      
    def send_through_ac(self):
        # send_through_ac_every = 25
        threading.Timer(send_through_ac_every, self.send_through_ac).start()
        inst_snd = '$AC;Iver3-' + iver + ';' + '$' + MRC_Iver.osd() + '\r\n'
        ser_ac.reset_output_buffer()
        ser_ac.write(inst_snd.encode())
        print(datetime.datetime.now(), ': Sending through AC: ', self.ac_i)
        self.q_log.put([datetime.datetime.now().strftime("%H:%M:%S:%f"), ': send trough AC: ', self.ac_i])
        self.ac_i += 1
    
    def log_data(self):
        while True:
            log = self.q_log.get()
            log_file.write(str(log) + '\n')
            log_file.flush()


def main():
    root = tk.Tk()
    root.title('RF-AC test')
    root.iconbitmap('usm.ico')
    app = App(root)
    app.pack()
    root.mainloop()


if __name__ == '__main__':
    main()
