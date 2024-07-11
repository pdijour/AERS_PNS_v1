import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QEvent, QTimer, QTime, QThread, pyqtSignal, QMutex, QMutexLocker
from PyQt5.QtGui import QFont, QIcon, QPixmap
from serial.tools import list_ports
import pyqtgraph as pg
import qdarktheme
import numpy as np
from functions import *
import datetime
import time
from fastlogging import LogInit
import qdarkstyle
import os
import time
import math

pathFormat = "%d_%m_%Y__%H_%M_%S"
nowTime = datetime.datetime.now()
time1 = nowTime.strftime(pathFormat)

pathFile = "ConsoleLogs/Data_" + time1 + ".log"
print('Path to Logfile is: */', pathFile)
logger = LogInit(pathName=pathFile, console=False, colors=True)

import serial

class ParameterStorage (QThread):
    def __init__(self):
        super(ParameterStorage, self).__init__()
        
        self.com_port = ""
        self.baudrate = 921600
        self.wave_type = "Trapezoid"
        self.stim_rise = 0.05
        self.stim_fall = 0.05
        self.resp_rate = 15
        self.i_time = 2.5
        self.e_time = 7.5
        self.pulse_train_freq = 60
        self.OnOff = True
        self.peak_current = 100
        self.current_high_calc = 50
        self.current_low_calc = 0
        self.voltage_high_calc = 5
        self.voltage_low_calc = 0
        self.OnOff2 = False
        self.peak_current2 = 100
        self.current_high_calc2 = 50
        self.current_low_calc2 = 0
        self.voltage_high_calc2 = 5
        self.voltage_low_calc2 = 0
        self.symmetric = False
        self.same_settings = False
        self.asymmetric = 1

        self.predictive_delay = 0
        self.triggering_type = "Mandatory"

        self.elapsed_time_label = ""
        self.number_cycles_label = ""

        self.MICRO = None
        self.micro_n_breaths = 0
        self.micro_insp_period = self.i_time
        self.micro_exp_period = self.e_time
        self.micro_frequency = self.pulse_train_freq
        self.micro_wave_type = 1
        self.micro_amplitude1 = self.voltage_high_calc
        self.micro_base1 = self.voltage_low_calc
        self.micro_allow_stim1 = 0
        self.micro_amplitude2 = self.voltage_high_calc2
        self.micro_base2 = self.voltage_low_calc2
        self.micro_allow_stim2 = 0
        self.micro_rise_time = self.stim_rise
        self.micro_fall_time = self.stim_fall
        self.micro_trigger_type = self.triggering_type

        self.sentZero = False
        self.sentOnce = False

        self.connected = False
        self.flow = None
        self.pressure = None
        self.switchstate = None
        self.values_esp = np.zeros(14)
        self.iestate = 0

        self.mutex = QMutex()

        self.read_thread = None

        self.thread_that_reads = None
        self.thread_that_writes = None

        self.stim1_val = None
        self.stim2_val = None
        self.pres_status = None

        self.I_trigger = True
        self.E_trigger = False
        self.E_trigger_I = False
    
        self.last_i_time = 0
        self.last_e_time = 0

        self.hfov_base = 0
        self.num_breaths_avg = 3

        self.burst_flag1 = 0
        self.burst_flag_ext1 = 0
        self.burst_flag2 = 0
        self.burst_flag_ext2 = 0

        self.bilevel_t = 50
        self.bilevel_v = 50

        self.flow_attached = False
        self.com_connected = False
        
        self.bias1 = False
        self.bias_num = 0
        self.bias2 = False
        self.bias_num2 = 0
        self.flag = False

        self.i_threshold = 15
        self.e_threshold = -10

        self.zeroing = False

    def update_from_main_window(self, main_window):
        self.com_port = main_window.com_port_list.currentText()
        self.wave_type = main_window.wave_type.currentText()
        self.stim_rise = main_window.stim_rise_spinbox.value()
        self.stim_fall = main_window.stim_fall_spinbox.value()
        self.resp_rate = main_window.resp_rate_spinbox.text()
        self.i_time = main_window.i_time
        self.e_time = main_window.e_time
        self.I_ratio = main_window.I_ratio_spinbox.text()
        self.E_ratio = main_window.E_ratio_spinbox.text()
        self.pulse_train_freq = main_window.pulse_train_freq_spinbox.value()
        self.OnOff = main_window.OnOff.isChecked()
        self.peak_current = main_window.peak_current_spinbox.value()
        self.current_high_calc = main_window.current_high_calc
        self.current_low_calc = main_window.current_low_calc
        self.voltage_high_calc = main_window.voltage_high_calc
        self.voltage_low_calc = main_window.voltage_low_calc
        self.rise = main_window.rise_spinbox.value()
        self.fall = main_window.fall_spinbox.value()
        self.OnOff2 = main_window.OnOff2.isChecked()
        self.peak_current2 = main_window.peak_current_spinbox2.value()
        self.current_high_calc2 = main_window.current_high_calc2
        self.current_low_calc2 = main_window.current_low_calc2
        self.voltage_high_calc2 = main_window.voltage_high_calc2
        self.voltage_low_calc2 = main_window.voltage_low_calc2
        self.rise2 = main_window.rise_spinbox2.value()
        self.fall2 = main_window.fall_spinbox2.value()
        self.symmetric = main_window.symmetric_checkbox.isChecked()
        self.same_settings = main_window.same_settings_checkbox.isChecked()
        self.asymmetric = main_window.asymmetric_spinbox.value()
        self.current_text = main_window.start_button.text()

        self.predictive_delay = main_window.predictive_delay_spinbox.value()
        self.triggering_type = main_window.triggering_combobox.currentText()

        self.I_trigger = main_window.radio_button1.isChecked()
        self.E_trigger = main_window.radio_button2.isChecked()
        self.E_trigger_I = main_window.radio_button3.isChecked()

        self.hfov_base = main_window.hfov_base_spinbox.value()
        self.bilevel_t = main_window.bilevel_t_spinbox.value()
        self.bilevel_v = main_window.bilevel_v_spinbox.value()

        self.num_breaths_avg = main_window.number_breaths_avg.value()

        self.i_threshold = main_window.I_threshold.value()
        self.e_threshold = main_window.E_threshold.value()

        self.bias1 = main_window.current_bias_checkbox.isChecked()
        self.bias2 = main_window.current_bias_checkbox2.isChecked()

class ReadThread(QThread):
    dataReceived = pyqtSignal(str)  # Emit the complete line as a signal

    def __init__(self, parameters, main_window_instance, mutex):
        super().__init__()
        self.running = True
        self.parameters = parameters
        self.main_window_instance = main_window_instance
        self.new_data = None
        self.mutex = mutex
        self.buffer = ""  # Initialize a buffer to accumulate incoming data
        self.error_logged = False

    def run(self):
        time.sleep(1)
        while self.running:
            if self.parameters.read_thread is None:
                try:
                    with QMutexLocker(self.mutex):
                        self.parameters.read_thread = serial.Serial(self.parameters.com_port, self.parameters.baudrate, timeout=0)
                        self.parameters.read_thread.dtr = False
                        self.parameters.read_thread.rts = False
                        print("Port connected")
                        self.error_logged = False  # Reset the error flag upon successful connection
                except serial.SerialException as e:
                    if not self.error_logged:
                        print(f"During reading, error opening serial port: {e}")
                        self.error_logged = True  # Set the flag to prevent further logging
                        # print(4)
                    self.parameters.read_thread = None
                    # print(3)
                    time.sleep(0.005)
                    continue
            else:
                try:
                    with QMutexLocker(self.mutex):
                        incoming_data = self.parameters.read_thread.read(2048).decode()
                        self.buffer += incoming_data
                except UnicodeDecodeError:
                    pass
                except serial.SerialException:    
                    self.parameters.read_thread = None
                    # self.close_serial_port()
                    continue
                except AttributeError:
                    print("Serial port closed")
                    # self.close_serial_port()
                    continue

            # Check if there is at least one complete line in the buffer
            while '\r\n' in self.buffer:
                line, self.buffer = self.buffer.split('\r\n', 1)  # Split on the first occurrence of \r\n
                if line:
                    self.process_data(line)

            # time.sleep(0.05)  # Sleep for 50 milliseconds

    def process_data(self, line):
        self.new_data = line.split(',')
        try:
            if len(self.new_data) == 24 and all(len(val) > 0 for val in self.new_data):
                self.parameters.switchstate = self.new_data[0]
                self.parameters.flow = float(self.new_data[1])
                self.parameters.pressure = float(self.new_data[2])
                self.parameters.iestate = float(self.new_data[3])
                # Phoebe change this cheat below
                if float(self.new_data[4]) <= self.parameters.voltage_high_calc:
                    self.parameters.stim1_val = float(self.new_data[4])
                if float(self.new_data[5]) <= self.parameters.voltage_high_calc2:
                    self.parameters.stim2_val = float(self.new_data[5])
                self.parameters.values_esp[2] = float(self.new_data[6])
                self.parameters.values_esp[3] = float(self.new_data[7])
                self.parameters.values_esp[4] = float(self.new_data[8])
                self.parameters.values_esp[5] = float(self.new_data[9])
                self.parameters.values_esp[6] = float(self.new_data[10])
                self.parameters.values_esp[7] = float(self.new_data[11])
                self.parameters.values_esp[8] = round(float(self.new_data[12]))
                self.parameters.values_esp[9] = float(self.new_data[13])
                self.parameters.values_esp[10] = float(self.new_data[14])
                self.parameters.values_esp[11] = float(self.new_data[15])
                self.parameters.pres_status = int(self.new_data[16])
                self.parameters.number_cycles_label = '{:.0f} stims'.format(float(self.new_data[17]))
                self.parameters.last_i_time = float(self.new_data[18])/1000
                self.parameters.last_e_time = float(self.new_data[19])/1000
                self.parameters.burst_flag1 = int(self.new_data[20])
                self.parameters.burst_flag_ext1 = int(self.new_data[21])
                self.parameters.burst_flag2 = int(self.new_data[22])
                self.parameters.burst_flag_ext2 = int(self.new_data[23])
                self.parameters.flow_attached = True
                if self.parameters.micro_allow_stim1 == 1 or self.parameters.micro_allow_stim2 == 1:
                    self.parameters.burst_flag1 = 0
                    self.parameters.burst_flag_ext1 = 0
                    self.parameters.burst_flag2 = 0
                    self.parameters.burst_flag_ext2 = 0
            elif len(self.new_data) == 8 and all(len(val) > 0 for val in self.new_data):
                self.parameters.switchstate = self.new_data[0]
                self.parameters.stim1_val = float(self.new_data[1])
                self.parameters.stim2_val = float(self.new_data[2])
                self.parameters.number_cycles_label = '{:.0f} stims'.format(float(self.new_data[3]))
                self.parameters.burst_flag1 = int(self.new_data[4])
                self.parameters.burst_flag_ext1 = int(self.new_data[5])
                self.parameters.burst_flag2 = int(self.new_data[6])
                self.parameters.burst_flag_ext2 = int(self.new_data[7])
                self.parameters.flow = 0
                self.parameters.pressure = 0
                self.parameters.iestate = 0
                self.parameters.flow_attached = False
                if self.parameters.micro_allow_stim1 == 1 or self.parameters.micro_allow_stim2 == 1:
                    self.parameters.burst_flag1 = 0
                    self.parameters.burst_flag_ext1 = 0
                    self.parameters.burst_flag2 = 0
                    self.parameters.burst_flag_ext2 = 0
            
            self.main_window_instance.processSerialData()
            self.parameters.values_esp[0] = self.parameters.flow
            self.parameters.values_esp[1] = self.parameters.pressure

        except ValueError as e:
            print(f"Error processing line: {e}")

    def close_serial_port(self):
        with QMutexLocker(self.mutex):
            if self.parameters.read_thread and self.parameters.read_thread.is_open:
                self.parameters.read_thread.close()
            self.parameters.read_thread = None

    def close_thread(self):
        if self.parameters.read_thread:
           self.parameters.read_thread.close()
        self.running = False  # Signal the thread to stop


class WriteThread(QThread):
    update_signal = pyqtSignal(str)  # Add more parameters if needed
    
    def __init__(self, parameters, mutex):
        QThread.__init__(self)
        self.running = True
        self.parameters = parameters
        self.mutex = mutex
        self.trigger_send = False
    
    def trigger_send_to_serial(self, allow_stim):
        try:
            self.send_to_serial(allow_stim)  # Move this line here
        except serial.SerialException as e:
            print(f"During trigger send to serial, error opening serial port: {e}")

    def run(self):
        pass

    def send_to_serial(self, allow_stim):
        if self.parameters.symmetric or (self.parameters.OnOff and not self.parameters.OnOff2) or (self.parameters.OnOff2 and not self.parameters.OnOff):
            self.parameters.micro_n_breaths = 0
        else:
            self.parameters.micro_n_breaths = self.parameters.asymmetric

        self.parameters.micro_insp_period = self.parameters.i_time

        self.parameters.micro_exp_period = self.parameters.e_time

        self.parameters.micro_frequency = self.parameters.pulse_train_freq

        # if allow_stim == 0 and not self.parameters.bias1 and not self.parameters.bias2:
        #     self.parameters.micro_wave_type = 0
        # elif allow_stim == 0 and self.parameters.bias1 and self.parameters.bias2:
        #     self.parameters.micro_wave_type = 6
        # elif allow_stim == 0 and self.parameters.bias1:
        #     self.parameters.micro_wave_type = 7
        # elif allow_stim == 0 and self.parameters.bias2:
        #     self.parameters.micro_wave_type = 8
        if self.parameters.wave_type == "Triangle":
            self.parameters.micro_wave_type = 1
        elif self.parameters.wave_type == "Trapezoid":
            self.parameters.micro_wave_type = 2
        elif self.parameters.wave_type == "Sine":
            self.parameters.micro_wave_type = 3
        elif self.parameters.wave_type == "HFOV":
            self.parameters.micro_wave_type = 4
        elif self.parameters.wave_type == "Bilevel":
            self.parameters.micro_wave_type = 5
        
        self.parameters.micro_rise_time = self.parameters.stim_rise
        self.parameters.micro_fall_time = self.parameters.stim_fall

        if self.parameters.triggering_type == "Mandatory":
            self.parameters.micro_trigger_type = 1
        elif self.parameters.triggering_type == "Synchronous":
            if self.parameters.I_trigger:
                self.parameters.micro_trigger_type = 2
            if self.parameters.E_trigger:
                self.parameters.micro_trigger_type = 7
            elif self.parameters.E_trigger_I:
                self.parameters.micro_trigger_type = 5
        elif self.parameters.triggering_type == "Synchronous Triggered":
            if self.parameters.I_trigger:
                self.parameters.micro_trigger_type = 3
            elif self.parameters.E_trigger:
                self.parameters.micro_trigger_type = 6
        elif self.parameters.triggering_type == "Predictive":
            self.parameters.micro_trigger_type = 8
        else:
            self.parameters.micro_trigger_type = 4


        if self.parameters.OnOff and allow_stim == 1:
            self.parameters.micro_allow_stim1 = 1
        elif allow_stim == 2:
            self.parameters.micro_allow_stim1 = 2
            # self.parameters.micro_allow_stim2 = 3
            self.parameters.micro_trigger_type = 1
        else:
            self.parameters.micro_allow_stim1 = 0

        self.parameters.micro_amplitude1 = self.parameters.voltage_high_calc
        if self.parameters.OnOff:
            self.parameters.micro_base1 = self.parameters.voltage_low_calc
        else:
            self.parameters.micro_base1 = 0

        if self.parameters.OnOff2 and allow_stim == 1:
            self.parameters.micro_allow_stim2 = 1
        elif allow_stim == 3:
            self.parameters.micro_allow_stim2 = 3
            # self.parameters.micro_allow_stim1 = 2
            self.parameters.micro_trigger_type = 1
        elif allow_stim == 5:
            self.parameters.micro_allow_stim2 = 3
            self.parameters.micro_allow_stim1 = 2
            self.parameters.micro_trigger_type = 1
        else:
            self.parameters.micro_allow_stim2 = 0
        
        if allow_stim == 4:
            self.parameters.micro_allow_stim1 = 4
            self.parameters.micro_allow_stim2 = 4
        
        self.parameters.micro_amplitude2 = self.parameters.voltage_high_calc2
        if self.parameters.OnOff2:
            self.parameters.micro_base2 = self.parameters.voltage_low_calc2
        else:
            self.parameters.micro_base2 = 0
                
        data = pack_data_to_transfer(
            self.parameters.micro_n_breaths,
            self.parameters.micro_insp_period,
            self.parameters.micro_exp_period,
            self.parameters.micro_frequency,
            self.parameters.micro_wave_type,
            self.parameters.micro_rise_time,
            self.parameters.micro_fall_time,
            self.parameters.micro_allow_stim1,
            self.parameters.micro_amplitude1,
            self.parameters.micro_base1,
            self.parameters.micro_allow_stim2,
            self.parameters.micro_amplitude2,
            self.parameters.micro_base2,
            self.parameters.micro_trigger_type,
            self.parameters.predictive_delay,
            self.parameters.hfov_base,
            self.parameters.num_breaths_avg,
            self.parameters.i_threshold,
            self.parameters.e_threshold,
            self.parameters.bilevel_t,
            self.parameters.bilevel_v,
            self.parameters.bias_num,
            self.parameters.bias_num2,
            self.parameters.zeroing)

        with QMutexLocker(self.mutex):
            print(data) 
            send_data(data, self.parameters.read_thread)


class StimPlot(QThread):
    def __init__(self, parameters):
        QThread.__init__(self)

        self.parameters = parameters
        self.init_plot()

        self.elapsed_time = QTime(0, 0)
        self.elapsed_time_timer = QTimer(self)
        self.elapsed_time_timer.timeout.connect(self.update_elapsed_time)
        self.elapsed_time_timer.start(1000)  # Update elapsed time every 1000 ms (1 second)

        self.parameters.elapsed_time_label = "00:00:00"
        self.parameters.number_cycles_label = "0 stims"

        self.next_stim_idx = 0

        self.counted_stim = False

    def toggle_elapsed_time_timer(self, is_running):
        if is_running:
            self.elapsed_time_timer.start(1000)
        else:
            self.elapsed_time_timer.stop()

    def init_plot(self):
        self.graphbox = pg.PlotWidget()
        self.graphbox.setMouseEnabled(x=False, y=False)
        self.graphbox.setMenuEnabled(False)
        self.graphbox.clear()
        # self.graphbox.setMaximumSize(5000, 200)

        self.graphbox.setBackground('k')
        styles = {"font-size": "18px"}

        self.graphbox.setLabel("bottom", "Time (s)")
        
        p1 = self.graphbox.plotItem
        p1.setLabel("left", "StimGate (V)") 
        p1.setYRange(0,10)

        # Create a new Viewbox, link the right axis to its coordinate system
        self.p2 = pg.ViewBox()
        p1.showAxis('right')
        p1.scene().addItem(self.p2)
        p1.getAxis('right').linkToView(self.p2)
        self.p2.setXLink(p1)
        p1.getAxis('right').setLabel('Flow (L/min)', color='#00FF00') #, **styles
        self.autoscale_on = False

        # Handle view resizing
        def updateViews():
            self.p2.setGeometry(p1.vb.sceneBoundingRect())

        updateViews()
        p1.vb.sigResized.connect(updateViews)

        # Set up plotting
        self.curve_purple = p1.plot(pen=pg.mkPen('#800080', width=3), name='Symmetric AWG')
        self.curve_red = p1.plot(pen=pg.mkPen('b', width=3), name='Stim1 AWG')
        self.curve_blue = p1.plot(pen=pg.mkPen('r', width=3), name='Stim2 AWG')
        self.curve_ie = p1.plot(pen=pg.mkPen('#FFFF00', width=1, name='IE'))

        # Set up arrays for plotting
        self.total = 30
        self.delay = 25
        self.freq = 1/self.delay
        self.x = np.linspace(0, self.total, self.total * int(1000 / self.delay))
        self.data_red = np.zeros(len(self.x))
        self.data_blue = np.zeros(len(self.x))
        self.data_purple = np.zeros(len(self.x))
        self.data = np.zeros(len(self.x))
        self.data_ie = np.zeros(len(self.x))

        self.graphbox.setXRange(0, self.total)

        self.currentIndex = 0

        self.period = 60 / self.parameters.resp_rate

        self.plot_running = False

        self.curve = pg.PlotCurveItem(pen=pg.mkPen('#00FF00', width=1, name='Flow'))
        # self.curve_ie = pg.PlotCurveItem(pen=pg.mkPen('#FFFF00', width=1, name='IE'))
        self.p2.addItem(self.curve)
        # p2.addItem(self.curve_ie)

        self.timer_flow = pg.QtCore.QTimer(self)
        self.timer_flow.timeout.connect(self.update_plot_flow)
        self.timer_flow.start(self.delay)

        self.IE_plot_on = True

    def clear_plot(self):
        self.plot_running = False
        self.elapsed_time_timer.stop()

        # self.parameters.thread_that_writes.trigger_send_to_serial(0)
        self.graphbox.clear()
        self.graphbox.setXRange(0, self.total)
        self.currentIndex = 0
        self.x = np.linspace(0, self.total, self.total*int(1000/self.delay))
        
        self.elapsed_time = QTime(0, 0)
        self.num_cycles = 0
        self.parameters.elapsed_time_label = self.elapsed_time.toString('hh:mm:ss')

        # Setup for AWG
        self.curve_purple = self.graphbox.plot(pen = pg.mkPen('#800080', width=3), name = 'Symmetric') #, symbol=large_dot
        self.curve_red = self.graphbox.plot(pen=pg.mkPen('b', width=3), name='Stim1')
        self.curve_blue = self.graphbox.plot(pen=pg.mkPen('r', width=3), name='Stim2')
        self.curve_ie = self.graphbox.plot(pen=pg.mkPen('#FFFF00', width=1, name='IE'))

        self.data_red = np.zeros(len(self.x))
        self.data_blue = np.zeros(len(self.x))
        self.data_purple = np.zeros(len(self.x))
        self.data_ie = np.zeros(len(self.x))

        self.data = np.zeros(len(self.x))

    def plot_IE_line(self):
        if self.IE_plot_on:
            self.IE_plot_on = False
        else:
            self.IE_plot_on = True
        
    def autoscale(self):
        if self.autoscale_on:
            self.autoscale_on = False
        else:
            self.autoscale_on = True
    
    def toggle_plot_state(self):
        if self.plot_running:
            self.plot_running = False
        else:
            self.plot_running = True
            self.counted_stim = False
        self.toggle_elapsed_time_timer(self.plot_running)
    
    def pause_plot(self):
        self.parameters.thread_that_writes.trigger_send_to_serial(0)
        self.plot_running = False

    def update_elapsed_time(self):
        if self.plot_running:
            self.elapsed_time = self.elapsed_time.addSecs(1)
            self.parameters.elapsed_time_label = self.elapsed_time.toString('hh:mm:ss')
    
    def update_plot_flow(self):
        # print(self.currentIndex)
        if self.autoscale_on:
            self.p2.enableAutoRange(axis=pg.ViewBox.YAxis)
        else:
             self.p2.setYRange(-250, 250) #(-250, 250) #(-1500, 2000)
        
        try:
            self.data[self.currentIndex] = self.parameters.flow
            self.data_red[self.currentIndex] = self.parameters.stim1_val
            self.data_blue[self.currentIndex] = self.parameters.stim2_val
            if self.IE_plot_on:
                if self.parameters.iestate == 1:
                    self.data_ie[self.currentIndex] = 10
                elif self.parameters.iestate == 2:
                    self.data_ie[self.currentIndex] = 0
                elif self.parameters.iestate == 0:
                    self.data_ie[self.currentIndex] = 5
            else:
                self.data_ie[self.currentIndex] = None
        except:
            self.data[self.currentIndex] = self.data[self.currentIndex - 1]
            self.data_red[self.currentIndex] = self.data_red[self.currentIndex - 1]
            self.data_blue[self.currentIndex] = self.data_blue[self.currentIndex - 1]
            self.data_ie[self.currentIndex] = self.data_ie[self.currentIndex - 1]

        next_index = (self.currentIndex + 1) % len(self.data)
        nextnext_index = (self.currentIndex + 2) % len(self.data)

        # Make the point right after the one plotted black to make plotting more obvious)
        self.data[next_index] = None
        self.data_red[next_index] = None
        self.data_blue[next_index] = None
        self.data_ie[next_index] = None

        self.data[nextnext_index] = None
        self.data_red[nextnext_index] = None
        self.data_blue[nextnext_index] = None
        self.data_ie[nextnext_index] = None

        # self.data[nextnextnext_index] = None
        # self.data_red[nextnextnext_index] = None
        # self.data_blue[nextnextnext_index] = None

        if self.currentIndex == len(self.data_red) - 1:
            self.x += self.total  # Increment the X-axis range
            self.graphbox.setXRange(self.x[0], self.x[-1])  # Update the X-axis range

        self.curve.setData(self.x, self.data)
        self.curve_red.setData(self.x, self.data_red)
        self.curve_blue.setData(self.x, self.data_blue)
        self.curve_ie.setData(self.x, self.data_ie)

        self.currentIndex = next_index


class ExamplePlot(QWidget):
    def __init__(self, parameters):
        super().__init__()

        self.parameters = parameters
        self.init_plot()

    def init_plot(self):
        self.graphbox = pg.PlotWidget()
        self.graphbox.setMouseEnabled(x=False, y=False)
        self.graphbox.setMenuEnabled(False)
        self.graphbox.setFixedHeight(QApplication.desktop().screenGeometry().height() // 5)
        self.graphbox.setFixedWidth(QApplication.desktop().screenGeometry().width() // 10)
        
        self.graphbox.setBackground('k')
        styles = {"font-size": "16px"} #"color": "#f00", 
        self.graphbox.setLabel("left", "StimGate (V)")
        self.graphbox.setLabel("bottom", "Time (s)")

        self.curve_red = self.graphbox.plot(pen=pg.mkPen('b', width=3))
        self.curve_blue = self.graphbox.plot(pen=pg.mkPen('r', width=3))
        self.curve_purple = self.graphbox.plot(pen=pg.mkPen('#800080', width=3))
        self.period = 60.0/self.parameters.resp_rate
        self.delay = 1
        self.x = np.linspace(0, self.period, int(self.period*1000.0/self.delay))
        self.data_red = np.zeros(len(self.x))
        self.data_blue = np.zeros(len(self.x))
        self.data_purple = np.zeros(len(self.x))
        self.graphbox.setXRange(0, self.period)
        self.graphbox.setYRange(0, 10)

        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.setInterval(self.delay)  # Set the timer interval here

        self.start_plot()

    def start_plot(self):
        self.timer.start(self.delay)  # Start or restart the timer when the "Start" button is clicked

    def update_plot(self):
        self.period = self.parameters.i_time + self.parameters.e_time
        self.x = np.linspace(0, self.period, int(self.period*1000.0/self.delay))
        self.data_red = np.zeros(len(self.x))
        self.data_blue = np.zeros(len(self.x))
        self.data_purple = np.zeros(len(self.x))
        self.graphbox.setXRange(0, self.period)

        v_high = self.parameters.voltage_high_calc
        v_high2 = self.parameters.voltage_high_calc2

        if self.parameters.wave_type == "HFOV":
            v_low = (self.parameters.hfov_base/100) * (self.parameters.voltage_high_calc - self.parameters.voltage_low_calc) + self.parameters.voltage_low_calc
            v_low2 = (self.parameters.hfov_base/100) * (self.parameters.voltage_high_calc2 - self.parameters.voltage_low_calc2) + self.parameters.voltage_low_calc2
        else:
            v_low = self.parameters.voltage_low_calc
            v_low2 = self.parameters.voltage_low_calc2

        if self.parameters.stim_rise != 0:
            self.m_rise_red = (v_high - v_low) / (self.parameters.stim_rise)
            self.m_rise_blue = (v_high2 - v_low2) / (self.parameters.stim_rise)
            self.m_rise_bilevel1_red = ((v_high * self.parameters.bilevel_v / 100) - (v_low)) / (self.parameters.stim_rise)
            self.m_rise_bilevel2_red = (v_high - (v_high * self.parameters.bilevel_v / 100)) / (self.parameters.stim_rise)
            self.m_rise_bilevel1_blue = ((v_high2 * self.parameters.bilevel_v / 100) - (v_low2)) / (self.parameters.stim_rise)
            self.m_rise_bilevel2_blue = (v_high2 - (v_high2 * self.parameters.bilevel_v / 100)) / (self.parameters.stim_rise)
        else:
            self.m_rise_red = 0
            self.m_rise_blue = 0
            self.m_rise_bilevel1_red = 0
            self.m_rise_bilevel2_red = 0
            self.m_rise_bilevel1_blue = 0
            self.m_rise_bilevel2_blue = 0
        
        if self.parameters.stim_fall != 0:
            self.m_fall_red = (v_low - v_high) / (self.parameters.stim_fall)
            self.m_fall_blue = (v_low2 - v_high2) / (self.parameters.stim_fall)
        else:
            self.m_fall_red = 0
            self.m_fall_blue = 0

        self.b_fall_red = v_high - (self.m_fall_red * (self.parameters.i_time - self.parameters.stim_fall))
        self.b_fall_blue = v_high2 - (self.m_fall_blue * (self.parameters.i_time - self.parameters.stim_fall))

        for i,v in enumerate(self.x):
            if v < self.parameters.i_time: #used to be v % self.period

                # Symmetric and same settings
                if self.parameters.symmetric and self.parameters.same_settings:
                    self.data_blue[i] = None
                    self.data_red[i] = None
                    if self.parameters.wave_type == "Sine":
                        self.data_purple[i] = (self.parameters.voltage_high_calc - self.parameters.voltage_low_calc) * np.sin(np.pi * v/self.parameters.i_time) + self.parameters.voltage_low_calc
                    elif self.parameters.wave_type == "Triangle":
                        if v < self.parameters.stim_rise:
                            self.data_purple[i] = self.m_rise_red * v + self.parameters.voltage_low_calc
                        else:
                            self.data_purple[i] = self.m_fall_red * v + self.b_fall_red
                    elif self.parameters.wave_type == "Trapezoid":
                        if self.parameters.stim_rise != 0 and v < self.parameters.stim_rise:
                            self.data_purple[i] = self.m_rise_red * v + self.parameters.voltage_low_calc
                        elif self.parameters.stim_rise == 0 and v == self.parameters.stim_rise:
                            self.data_purple[i] = self.parameters.voltage_low_calc
                        elif (v > self.parameters.stim_rise and v < (self.parameters.i_time - self.parameters.stim_fall)):
                            self.data_purple[i] = self.parameters.voltage_high_calc
                        else:
                            self.data_purple[i] = self.m_fall_red * v + self.b_fall_red
                    elif self.parameters.wave_type == "HFOV":
                        t_ = v % (self.parameters.stim_rise + self.parameters.stim_fall)  # Reset time within each period
                        if v < self.parameters.i_time:
                            if t_ < self.parameters.stim_rise:
                                self.data_purple[i] = self.m_rise_red * t_ + v_low2
                            else:
                                self.data_purple[i] = self.parameters.hfov_base
                    elif self.parameters.wave_type == "Bilevel":
                        # first rise
                        if self.parameters.stim_rise != 0 and v < self.parameters.stim_rise:
                            self.data_purple[i] = self.m_rise_bilevel1_red * v + self.parameters.voltage_low_calc
                        elif self.parameters.stim_rise == 0 and v == self.parameters.stim_rise:
                            self.data_purple[i] = self.parameters.voltage_low_calc
                        # first level
                        elif v > self.parameters.stim_rise and v < self.parameters.i_time * (self.parameters.bilevel_t / 100):
                            self.data_purple[i] = self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100)
                        # second rise
                        elif self.parameters.stim_rise != 0 and v >= self.parameters.i_time * (self.parameters.bilevel_t / 100) and v < self.parameters.i_time * (self.parameters.bilevel_t / 100) + self.parameters.stim_rise:
                            self.data_purple[i] = self.m_rise_bilevel2_red * (v - (self.parameters.i_time * (self.parameters.bilevel_t / 100))) + (self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100))
                        elif self.parameters.stim_rise == 0 and v == self.parameters.i_time * (self.parameters.bilevel_t / 100) + self.parameters.stim_rise:
                            self.data_purple[i] = self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100)
                        # second level
                        elif v >= self.parameters.i_time * (self.parameters.bilevel_t / 100) and v < (self.parameters.i_time - self.parameters.stim_fall):
                            self.data_purple[i] = self.parameters.voltage_high_calc
                        # fall
                        else:
                            self.data_purple[i] = self.m_fall_red * v + self.b_fall_red
                
                # Both on but dif settings or asymmetric
                elif self.parameters.OnOff2:
                    self.data_purple[i] = None
                    
                    if self.parameters.wave_type == "Sine":
                        self.data_red[i] = (self.parameters.voltage_high_calc - self.parameters.voltage_low_calc) * np.sin(np.pi * v/self.parameters.i_time) + self.parameters.voltage_low_calc
                        self.data_blue[i] = (self.parameters.voltage_high_calc2 - self.parameters.voltage_low_calc2) * np.sin(np.pi * v/self.parameters.i_time) + self.parameters.voltage_low_calc2
                    elif self.parameters.wave_type == "Triangle":
                        if v < self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_red * v + self.parameters.voltage_low_calc
                            self.data_blue[i] = self.m_rise_blue * v + self.parameters.voltage_low_calc2
                        else:
                            self.data_red[i] = self.m_fall_red * v + self.b_fall_red
                            self.data_blue[i] = self.m_fall_blue * v + self.b_fall_blue
                    elif self.parameters.wave_type == "Trapezoid":
                        if self.parameters.stim_rise != 0 and v < self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_red * v + self.parameters.voltage_low_calc
                            self.data_blue[i] = self.m_rise_blue * v + self.parameters.voltage_low_calc2
                        elif self.parameters.stim_rise == 0 and v == self.parameters.stim_rise:
                            self.data_red[i] = self.parameters.voltage_low_calc
                            self.data_blue[i] = self.parameters.voltage_low_calc2
                        elif (v > self.parameters.stim_rise and v < (self.parameters.i_time - self.parameters.stim_fall)):
                            self.data_red[i] = self.parameters.voltage_high_calc
                            self.data_blue[i] = self.parameters.voltage_high_calc2
                        else:
                            self.data_red[i] = self.m_fall_red * v + self.b_fall_red
                            self.data_blue[i] = self.m_fall_blue * v + self.b_fall_blue
                    elif self.parameters.wave_type == "HFOV":
                        t_ = v % (self.parameters.stim_rise + self.parameters.stim_fall)  # Reset time within each period
                        if v < self.parameters.i_time:
                            if t_ < self.parameters.stim_rise:
                                self.data_red[i] = self.m_rise_red * t_ + v_low
                                self.data_blue[i] = self.m_rise_blue * t_ + v_low
                            else:
                                self.data_red[i] = self.parameters.hfov_base
                                self.data_blue[i] = self.parameters.hfov_base
                    elif self.parameters.wave_type == "Bilevel":
                        # first rise
                        if self.parameters.stim_rise != 0 and v < self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_bilevel1_red * v + self.parameters.voltage_low_calc
                            self.data_blue[i] = self.m_rise_bilevel1_blue * v + self.parameters.voltage_low_calc2
                        elif self.parameters.stim_rise == 0 and v == self.parameters.stim_rise:
                            self.data_red[i] = self.parameters.voltage_low_calc
                            self.data_blue[i] = self.parameters.voltage_low_calc2
                        # first level
                        elif v > self.parameters.stim_rise and v < self.parameters.i_time * (self.parameters.bilevel_t / 100):
                            self.data_red[i] = self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100)
                            self.data_blue[i] = self.parameters.voltage_high_calc2 * (self.parameters.bilevel_v / 100)
                        # second rise
                        elif self.parameters.stim_rise != 0 and v >= self.parameters.i_time * (self.parameters.bilevel_t / 100) and v < self.parameters.i_time * (self.parameters.bilevel_t / 100) + self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_bilevel2_red * (v - (self.parameters.i_time * (self.parameters.bilevel_t / 100))) + (self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100))
                            self.data_blue[i] = self.m_rise_bilevel2_blue * (v - (self.parameters.i_time * (self.parameters.bilevel_t / 100))) + (self.parameters.voltage_high_calc2 * (self.parameters.bilevel_v / 100))
                        elif self.parameters.stim_rise == 0 and v == self.parameters.i_time * (self.parameters.bilevel_t / 100) + self.parameters.stim_rise:
                            self.data_red[i] = self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100)
                            self.data_blue[i] = self.parameters.voltage_high_calc2 * (self.parameters.bilevel_v / 100)
                        # second level
                        elif v >= self.parameters.i_time * (self.parameters.bilevel_t / 100) and v < (self.parameters.i_time - self.parameters.stim_fall):
                            self.data_red[i] = self.parameters.voltage_high_calc
                            self.data_blue[i] = self.parameters.voltage_high_calc2
                        # fall
                        else:
                            self.data_red[i] = self.m_fall_red * v + self.b_fall_red
                            self.data_blue[i] = self.m_fall_blue * v + self.b_fall_blue
                    
                    if not self.parameters.OnOff:
                        self.data_red[i] = None

                # Just stim1
                else:
                    self.data_blue[i] = None
                    self.data_purple[i] = None

                    if self.parameters.wave_type == "Sine":
                        self.data_red[i] = (self.parameters.voltage_high_calc - self.parameters.voltage_low_calc) * np.sin(np.pi * v/self.parameters.i_time) + self.parameters.voltage_low_calc
                    elif self.parameters.wave_type == "Triangle":
                        if v < self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_red * v + self.parameters.voltage_low_calc
                        else:
                            self.data_red[i] = self.m_fall_red * v + self.b_fall_red
                    elif self.parameters.wave_type == "Trapezoid":
                        if self.parameters.stim_rise != 0 and v < self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_red * v + self.parameters.voltage_low_calc
                        elif self.parameters.stim_rise == 0 and v == self.parameters.stim_rise:
                            self.data_red[i] = self.parameters.voltage_low_calc
                        elif (v > self.parameters.stim_rise and v < (self.parameters.i_time - self.parameters.stim_fall)):
                            self.data_red[i] = self.parameters.voltage_high_calc
                        else:
                            self.data_red[i] = self.m_fall_red * v + self.b_fall_red
                    elif self.parameters.wave_type == "HFOV":
                        t_ = v % (self.parameters.stim_rise + self.parameters.stim_fall)  # Reset time within each period
                        if t_ < self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_red * t_ + v_low
                        else:
                            self.data_red[i] = self.parameters.hfov_base
                    elif self.parameters.wave_type == "Bilevel":
                        # first rise
                        if self.parameters.stim_rise != 0 and v < self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_bilevel1_red * v + self.parameters.voltage_low_calc
                        elif self.parameters.stim_rise == 0 and v == self.parameters.stim_rise:
                            self.data_red[i] = self.parameters.voltage_low_calc
                        # first level
                        elif v > self.parameters.stim_rise and v < self.parameters.i_time * (self.parameters.bilevel_t / 100):
                            self.data_red[i] = self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100)
                        # second rise
                        elif self.parameters.stim_rise != 0 and v >= self.parameters.i_time * (self.parameters.bilevel_t / 100) and v < self.parameters.i_time * (self.parameters.bilevel_t / 100) + self.parameters.stim_rise:
                            self.data_red[i] = self.m_rise_bilevel2_red * (v - (self.parameters.i_time * (self.parameters.bilevel_t / 100))) + (self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100))
                        elif self.parameters.stim_rise == 0 and v == self.parameters.i_time * (self.parameters.bilevel_t / 100) + self.parameters.stim_rise:
                            self.data_red[i] = self.parameters.voltage_high_calc * (self.parameters.bilevel_v / 100)
                        # second level
                        elif v >= self.parameters.i_time * (self.parameters.bilevel_t / 100) and v < (self.parameters.i_time - self.parameters.stim_fall):
                            self.data_red[i] = self.parameters.voltage_high_calc
                        # fall
                        else:
                            self.data_red[i] = self.m_fall_red * v + self.b_fall_red
            else:
                if self.parameters.symmetric and self.parameters.same_settings:
                    self.data_purple[i] = self.parameters.voltage_low_calc
                    self.data_blue[i] = None
                    self.data_red[i] = None
                elif self.parameters.symmetric:
                    self.data_purple[i] = None
                    self.data_blue[i] = self.parameters.voltage_low_calc2
                    self.data_red[i] = self.parameters.voltage_low_calc
                elif self.parameters.OnOff2:
                    self.data_red[i] = self.parameters.voltage_low_calc
                    self.data_blue[i] = self.parameters.voltage_low_calc2
                    self.data_purple[i] = None
                else:
                    self.data_red[i] = self.parameters.voltage_low_calc
                    self.data_blue[i] = None
                    self.data_purple[i] = None

        self.curve_red.setData(self.x, self.data_red)
        self.curve_blue.setData(self.x, self.data_blue)
        self.curve_purple.setData(self.x, self.data_purple)

        self.graphbox.show()

class PresPlot(QWidget):
    def __init__(self, parameters):
        super().__init__()

        self.parameters = parameters
        self.init_plot()

    def init_plot(self):
        self.graphbox = pg.PlotWidget()
        self.graphbox.setMouseEnabled(x=False, y=False)
        self.graphbox.setMenuEnabled(False)

        self.graphbox.setBackground('k')
        styles = {"font-size": "30px"} #"color": "#f00", 
        stylesg = {'color': '#00FF00'} #"color": "#f00", 
        self.graphbox.setLabel("left", " ", **stylesg)
        self.graphbox.setLabel("right", "Pressure (cmH2O)", **stylesg)
        self.graphbox.setLabel("bottom", "Time (s)")
        
        # self.graphbox.getAxis("bottom").setStyle(**styles)

        self.curve = self.graphbox.plot(pen=pg.mkPen('#00FF00', width=1))
        self.total = 30
        self.delay = 25
        self.x = np.linspace(0, self.total, int(self.total*1000/self.delay))
        self.data = np.zeros(len(self.x))
        self.currentIndex = 0
        self.graphbox.setXRange(0, self.total)
        self.graphbox.setYRange(-5, 30)
        yticks = [(-5, '-5'), (0, '0'), (5, '5'), (10, '10'), (15, '15'), (20, '20'), (25, '25'), (30, '30')]
        self.graphbox.getAxis('left').setTicks([yticks])
        self.graphbox.getAxis('right').setTicks([yticks])

        self.start_plot()

    def start_plot(self):
        self.timer = pg.QtCore.QTimer(self)
        self.timer.start(self.delay)  # Start or restart the timer when the "Start" button is clicked
        self.timer.timeout.connect(self.update_plot)

    def update_plot(self):
        self.data[self.currentIndex] = self.parameters.pressure
        self.curve.setData(self.x, self.data)

        # Make the point right after the one plotted black to make plotting more obvious
        next_index = (self.currentIndex + 1) % len(self.data)
        self.data[next_index] = None

        if self.currentIndex == len(self.data) - 1:
            self.x += self.total  # Increment the X-axis range
            self.graphbox.setXRange(self.x[0], self.x[-1])  # Update the X-axis range

        self.currentIndex = next_index

        self.graphbox.show()
    
    def clear_plot(self):
        self.plot_running = False
        # self.parameters.thread_that_writes.trigger_send_to_serial(0)
        self.graphbox.clear()
        self.graphbox.setXRange(0, self.total)
        self.currentIndex = 0
        self.x = np.linspace(0, self.total, self.total*int(1000/self.delay))
        
        self.elapsed_time = QTime(0, 0)
        self.num_cycles = 0
        self.parameters.elapsed_time_label = self.elapsed_time.toString('hh:mm:ss')

        # Setup for AWG
        self.curve = self.graphbox.plot(pen=pg.mkPen('#00FF00', width=1))
        self.data = np.zeros(len(self.x))

#%% MAIN WINDOW CLASS
class MainWindow(QDialog):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        icon_path = os.path.join(getattr(sys, '_MEIPASS', os.path.abspath(os.path.dirname(__file__))), 'images/logo_vector_edge2.png')
        self.setWindowIcon(QIcon(icon_path))

        self.setWindowTitle('Lunair Obstructive Sleep Apnea Research Stimulator')
    
        self.parameters = ParameterStorage()
        self.mutex = QMutex()

        self.setup_timer()

        # Create a tab widget
        self.tabs = QTabWidget(self)

        self.tab1 = QWidget()
        self.tab2 = QWidget()
        self.tab3 = QWidget()
        self.tab4 = QWidget()

        self.tabs.addTab(self.tab1, 'Controls')
        self.tabs.addTab(self.tab2, 'Values')
        self.tabs.addTab(self.tab3, 'Plots')
        self.tabs.addTab(self.tab4, 'About')

        # Create Widgets -----------
        self.figure = StimPlot(self.parameters)
        self.flow_figure = StimPlot(self.parameters)
        self.example_figure = ExamplePlot(self.parameters)
        self.create_wave_settings()
        self.create_trigger_settings()
        self.create_stim1_settings()
        self.create_stim2_settings()
        self.create_figure()
        self.select_com_port()
        # self.create_example_figure()

        self.pres_figure = PresPlot(self.parameters)
        self.create_plots()
        self.create_values()
        self.create_about()

        # Start a timer to periodically update the parameters
        self.sync_timer_IE = QTimer()

        self.current_high_spinbox.valueChanged.connect(self.checkAmplitudes)
        self.current_low_spinbox.valueChanged.connect(self.checkAmplitudes)
        self.current_high_spinbox2.valueChanged.connect(self.checkAmplitudes)
        self.current_low_spinbox2.valueChanged.connect(self.checkAmplitudes)

        self.I_time.valueChanged.connect(self.checkItime)
        self.stim_rise_spinbox.valueChanged.connect(self.checkItime)
        self.stim_fall_spinbox.valueChanged.connect(self.checkItime)
        self.wave_type.currentIndexChanged.connect(self.checkItime)
        self.hfov_freq_spinbox.valueChanged.connect(self.checkItime)

        self.sync_timer_IE.timeout.connect(self.Itime_checker)
    
        self.sync_timer_IE.start(100)  # Adjust the interval as needed

        self.GUI_setup.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.GUI_wave_settings.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.GUI_trigger_settings.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.example_groupbox.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        self.GUI_stim1_settings.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.GUI_stim2_settings.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.graphgroupbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)


        self.importantvalues = QGroupBox("")
        # self.importantvalues.setStyleSheet("font-size: 15px")

        self.elapsed_time_label = QLabel("00:00:00")
        self.number_cycles_label = QLabel('0 stims')

        self.insp_label = QLabel("INSP")
        self.exp_label = QLabel("EXP")

        self.insp_label.setStyleSheet("color: gray;")
        self.exp_label.setStyleSheet("color: gray;")
        self.insp_label.setStyleSheet("border: 1px solid black;") 
        self.exp_label.setStyleSheet("border: 1px solid black;") 

        self.elapsed_timer = QTimer()
        self.elapsed_timer.timeout.connect(self.update_elapsed_time)
        self.elapsed_timer.start(1000)

        self.ie_timer = QTimer()
        self.ie_timer.timeout.connect(self.update_ie_labels)
        self.ie_timer.start(500)

        toplayout = QGridLayout()
        toplayout.addWidget(self.elapsed_time_label,         0, 0, 1, 1)
        toplayout.addWidget(self.number_cycles_label,        0, 1, 1, 1)
        toplayout.addWidget(self.insp_label,                 0, 2, 1, 1)
        toplayout.addWidget(self.exp_label,                  0, 3, 1, 1)
        self.importantvalues.setLayout(toplayout)
        toplayout.setContentsMargins(0, 0, 0, 0)

        # Set entire layout
        entireLayout = QGridLayout()
        entireLayout.addWidget(self.tabs,                       0, 0, 50, 3)
        entireLayout.addWidget(self.importantvalues,            0, 2, 1, 1)
        self.setLayout(entireLayout)
        entireLayout.setContentsMargins(0, 0, 0, 0)
        

        # Set layouts
        mainLayout = QGridLayout()
        mainLayout.addWidget(self.GUI_setup,                  0, 0, 1, 4)
        mainLayout.addWidget(self.GUI_trigger_settings,       1, 0, 3, 2)
        trig_settings_height = self.GUI_trigger_settings.sizeHint().height()
        mainLayout.addWidget(self.GUI_wave_settings,          1, 2, 3, 2)
        # mainLayout.addWidget(self.example_groupbox,           2, 2, 1, 2, alignment=Qt.AlignCenter)
        # mainLayout.addWidget(self.GUI_wave_settings,          1, 2, 4, 1)
        # mainLayout.addWidget(self.example_groupbox,           1, 3, 4, 1)
        # self.example_groupbox.setFixedHeight(trig_settings_height)
        # self.example_groupbox.setFixedWidth(self.GUI_wave_settings.sizeHint().width())
        mainLayout.addWidget(self.GUI_stim1_settings,         5, 0, 1, 2)
        mainLayout.addWidget(self.GUI_stim2_settings,         5, 2, 1, 2)
        mainLayout.addWidget(self.graphgroupbox,              6, 0, 1, 4)
        mainLayout.addWidget(self.figure.graphbox,            7, 0, 1, 4)
        self.tab1.setLayout(mainLayout)

        valuesLayout = QGridLayout()
        valuesLayout.addWidget(self.valuesboxes)
        self.tab2.setLayout(valuesLayout)

        plotLayout = QGridLayout()
        plotLayout.addWidget(self.flowpresgraphs)
        self.tab3.setLayout(plotLayout)

        aboutLayout = QGridLayout()
        aboutLayout.addWidget(self.aboutboxes)
        self.tab4.setLayout(aboutLayout)

        # __________________________________
    
        # Connect signals to slots
        # self.resp_rate_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        # self.I_ratio_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        # self.E_ratio_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.com_port_list.currentIndexChanged.connect(self.update_parameters_from_gui)
        self.wave_type.currentIndexChanged.connect(self.update_parameters_from_gui)
        self.stim_rise_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.stim_fall_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.pulse_train_freq_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.OnOff.stateChanged.connect(self.update_parameters_from_gui)
        self.peak_current_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.current_high_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.current_low_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.rise_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.fall_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.OnOff2.stateChanged.connect(self.update_parameters_from_gui)
        self.peak_current_spinbox2.valueChanged.connect(self.update_parameters_from_gui)
        self.current_high_spinbox2.valueChanged.connect(self.update_parameters_from_gui)
        self.current_low_spinbox2.valueChanged.connect(self.update_parameters_from_gui)
        self.rise_spinbox2.valueChanged.connect(self.update_parameters_from_gui)
        self.fall_spinbox2.valueChanged.connect(self.update_parameters_from_gui)
        self.symmetric_checkbox.stateChanged.connect(self.update_parameters_from_gui)
        self.asymmetric_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        
        self.hfov_base_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.hfov_freq_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.I_time.valueChanged.connect(self.update_parameters_from_gui)
        self.resp_rate_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.bilevel_t_spinbox.valueChanged.connect(self.update_parameters_from_gui)
        self.bilevel_v_spinbox.valueChanged.connect(self.update_parameters_from_gui)

        self.prev_values = (self.wave_type.currentText(),
            self.stim_rise_spinbox.value(),
            self.stim_fall_spinbox.value(),
            self.pulse_train_freq_spinbox.value(),
            self.OnOff.isChecked(),
            self.peak_current_spinbox.value(),
            self.OnOff2.isChecked(),
            self.symmetric_checkbox.isChecked(),
            self.asymmetric_spinbox.value(),
            self.voltage_high_calc,
            self.voltage_low_calc,
            self.voltage_high_calc2,
            self.voltage_low_calc2,
            self.predictive_delay_spinbox.value(),
            self.triggering_combobox.currentIndex(),
            self.radio_button1.isChecked(),
            self.radio_button2.isChecked(),
            self.radio_button3.isChecked(),
            self.I_threshold.value(),
            self.E_threshold.value(),
            self.bilevel_t_spinbox.value(),
            self.bilevel_t_spinbox.value(),
            self.current_bias_checkbox.isChecked(),
            self.current_bias_checkbox2.isChecked())

        self.prev_values_not_synch = (self.resp_rate_spinbox.value(),
                                      self.I_time.value())
    
        self.prev_values_not_synchtrig = (self.I_time.value())

        self.prev_values_pred = (self.number_breaths_avg.value())
        
        self.create_threads()

    def closeEvent(self, event):
        print("GUI closed")
        # Add your cleanup or additional actions here
        # event.accept()
        self.parameters.voltage_low_calc = 0
        self.parameters.voltage_low_calc2 = 0
        self.parameters.bias_num = 0
        self.parameters.bias_num2 = 0
        self.parameters.thread_that_writes.trigger_send_to_serial(0)
        # serial.Serial(self.parameters.com_port, self.parameters.baudrate, timeout=0)
        # self.parameters.read_thread.close()

    def create_threads(self):
        if not hasattr(self.parameters, 'thread_that_reads') or self.parameters.thread_that_reads is None:
            self.parameters.thread_that_reads = ReadThread(self.parameters, self, self.mutex)
            self.parameters.thread_that_reads.start()
        if not hasattr(self.parameters, 'thread_that_writes') or self.parameters.thread_that_writes is None:
            self.parameters.thread_that_writes = WriteThread(self.parameters, self.mutex)
            self.parameters.thread_that_writes.start()
        # print("Threads created")

    def update_ie_labels(self):
        if self.parameters.iestate == 0:
            self.insp_label.setStyleSheet("color: gray;")
            self.exp_label.setStyleSheet("color: gray;")
        elif self.parameters.iestate == 1:
            self.insp_label.setStyleSheet("color: green;")
            self.exp_label.setStyleSheet("color: gray;")
        elif self.parameters.iestate == 2:
            self.insp_label.setStyleSheet("color: gray;")
            self.exp_label.setStyleSheet("color: red;")

    def Itime_checker(self):
        if self.parameters.wave_type == "Triangle":
            self.stim_rise_spinbox.setValue(self.parameters.i_time - self.parameters.stim_fall)
            self.stim_fall_spinbox.setValue(self.parameters.i_time - self.parameters.stim_rise)
    
    def update_parameters_from_gui(self):
        self.parameters.update_from_main_window(self)
        self.example_figure.update_plot()

    def check_value_changes(self):
        current_values = (self.wave_type.currentText(),
            self.stim_rise_spinbox.value(),
            self.stim_fall_spinbox.value(),
            self.pulse_train_freq_spinbox.value(),
            self.OnOff.isChecked(),
            self.peak_current_spinbox.value(),
            self.OnOff2.isChecked(),
            self.symmetric_checkbox.isChecked(),
            self.asymmetric_spinbox.value(),
            self.voltage_high_calc,
            self.voltage_low_calc,
            self.voltage_high_calc2,
            self.voltage_low_calc2,
            self.predictive_delay_spinbox.value(),
            self.triggering_combobox.currentIndex(),
            self.radio_button1.isChecked(),
            self.radio_button2.isChecked(),
            self.radio_button3.isChecked(),
            self.I_threshold.value(),
            self.E_threshold.value(),
            self.bilevel_t_spinbox.value(),
            self.bilevel_t_spinbox.value(),
            self.current_bias_checkbox.isChecked(),
            self.current_bias_checkbox2.isChecked())

        current_values_not_synch = (self.resp_rate_spinbox.value(),
                                    self.I_time.value())
        
        current_values_not_synchtrig = (self.I_time.value())

        current_values_pred = (self.number_breaths_avg.value())
        
        if current_values != self.prev_values:
            self.serial_send()

        if (self.triggering_combobox.currentText() == "Synchronous Triggered" or self.triggering_combobox.currentText() == "Predictive Triggered") and current_values_not_synchtrig != self.prev_values_not_synchtrig:
            self.serial_send()
        
        if self.triggering_combobox.currentText() == "Mandatory" and current_values_not_synch != self.prev_values_not_synch:
            self.serial_send()
        
        if (self.triggering_combobox.currentText() == "Predictive" or self.triggering_combobox.currentText() == "Predictive Triggered") and current_values_pred != self.prev_values_pred:
            self.serial_send()

        self.prev_values = current_values
        self.prev_values_not_synch = current_values_not_synch
        self.prev_values_not_synchtrig = current_values_not_synchtrig
        self.prev_values_pred = current_values_pred

    def serial_send(self):
        if self.parameters.current_text == "Stimulating. Press here to Pause":
            self.parameters.update_from_main_window(self)
            
            logger.info("Stimulation Pattern Changed (no Pause): " + str(datetime.datetime.now()))
            print("Stimulation Pattern Changed (no Pause): " + str(datetime.datetime.now()))
            self.parameters.thread_that_writes.trigger_send_to_serial(1)
            logger.info("Wave Type: {}; Rise: {}; Fall: {}; I ratio: {}; E ratio: {}; \
                        Stim 1 On/Off: {}; Stim 1 Peak current: {}; Stim 1 Voltage base: {}; Stim 1 Voltage amplitude: {}; \
                        Stim 2 On/Off: {}; Stim 2 Peak current: {}; Stim 2 Voltage base: {}; Stim 2 Voltage amplitude: {}; \
                        Symmetric: {}; Asymmetric # of breaths each side: {}".format(self.wave_type.currentText(), 
                                                                                     self.stim_rise_spinbox.value(), 
                                                                                     self.stim_fall_spinbox.value(), 
                                                                                     self.I_time.value(), 
                                                                                     self.E_time.value(), 
                                                                                     self.OnOff.isChecked(),
                                                                                     self.peak_current_spinbox.value(),
                                                                                     self.voltage_low_calc,
                                                                                     self.voltage_high_calc,
                                                                                     self.OnOff2.isChecked(),
                                                                                     self.peak_current_spinbox2.value(),
                                                                                     self.voltage_low_calc2,
                                                                                     self.voltage_high_calc2,
                                                                                     self.symmetric_checkbox.isChecked(),
                                                                                     self.asymmetric_spinbox.value(),
                                                                                     self.triggering_combobox.currentText(),
                                                                                     self.predictive_delay_spinbox.value()))

    def setup_timer(self):
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_parameters_from_gui)
        self.update_timer.timeout.connect(self.check_value_changes)
        self.update_timer.start(1000)  # Update every 10 milliseconds


    # -------- About tab -------------------
    def create_about(self):
        self.aboutboxes = QGroupBox("")
        self.version_num = QLabel("Product: FlowPres Research Stimulator v1.0")
        self.description = QLabel("Description: This is a test device that can create arbitrary waveforms, stimulation gates, and pulse trains for two different constant current stimulators. A variety of parameters can be manipulated in the Controls tab, various flow and pressure metrics are displayed in the Values tab, and flow/pressure and waveforms are visualized in the Plots tab.")
        self.description.setWordWrap(True)
        self.date_revision = QLabel("Last updated: 06-28-2024")
        self.creator = QLabel("Developed by Phoebe Dijour, R&D Engineer, Deerfield Catalyst")
        self.contact = QLabel("Contact us: info@dfcatalyst.com")
        self.release_notes = QLabel("Release Notes:\n- Double stim, predictive, basic features")
        self.requirements = QLabel("System Requirements:\n- Windows 10 or later\n- 2 GHz processor\n- 4 GB RAM\n- 500 MB free disk space")
        self.safety = QLabel("For investigational use only, not available for commercial use")

        self.lunair_logo = QLabel()
        icon_path = os.path.join(getattr(sys, '_MEIPASS', os.path.abspath(os.path.dirname(__file__))), 'images/logo_rect.png')
        self.lunair_logo.setPixmap(QPixmap(icon_path).scaledToHeight(200)) #.scaledToWidth(200)

        self.dfc_logo = QLabel()
        icon_path = os.path.join(getattr(sys, '_MEIPASS', os.path.abspath(os.path.dirname(__file__))), 'images/logo_dfc2.png')
        self.dfc_logo.setPixmap(QPixmap(icon_path).scaledToHeight(200)) #.scaledToWidth(200)

        about_layout = QGridLayout()
        about_layout.addWidget(self.version_num,   0, 0, 1, 3)
        about_layout.addWidget(self.description,   1, 0, 1, 3)
        about_layout.addWidget(self.date_revision, 2, 0, 1, 3)
        about_layout.addWidget(self.creator,       3, 0, 1, 3)
        about_layout.addWidget(self.contact,       4, 0, 1, 3)
        about_layout.addWidget(self.release_notes, 5, 0, 1, 3)
        about_layout.addWidget(self.requirements,  6, 0, 1, 3)
        about_layout.addWidget(self.safety,        7, 0, 1, 3)
        about_layout.addWidget(self.lunair_logo,   8, 0, 1, 1)
        about_layout.addWidget(self.dfc_logo,      8, 1, 1, 1)
        self.aboutboxes.setLayout(about_layout)
    
    # -------- Values in second tab -------------
    def create_values(self):
        self.valuesboxes = QGroupBox("")
        self.valuesboxes.setStyleSheet("font-size: 25px")

        self.value1 = QLabel(str(self.parameters.values_esp[0]))
        self.value1.setStyleSheet("border: 1px solid black;") 
        self.label1 = QLabel("Flow (lpm)")
        self.label1.setBuddy(self.value1)

        self.value2 = QLabel(str(self.parameters.values_esp[1]))
        self.value2.setStyleSheet("border: 1px solid black;") 
        self.label2 = QLabel("Pressure (cmH2O)")
        self.label2.setBuddy(self.value2)

        self.value3 = QLabel(str(self.parameters.values_esp[2]))
        self.value3.setStyleSheet("border: 1px solid black;") 
        self.label3 = QLabel("Ti(s)")
        self.label3.setBuddy(self.value3)

        self.value4 = QLabel(str(self.parameters.values_esp[3]))
        self.value4.setStyleSheet("border: 1px solid black;") 
        self.label4 = QLabel("IE Ratio")
        self.label4.setBuddy(self.value4)

        self.value5 = QLabel(str(self.parameters.values_esp[4]))
        self.value5.setStyleSheet("border: 1px solid black;") 
        self.label5 = QLabel("PEEP (cmH2O)")
        self.label5.setBuddy(self.value5)

        self.value6 = QLabel(str(self.parameters.values_esp[5]))
        self.value6.setStyleSheet("border: 1px solid black;") 
        self.label6 = QLabel("PiMax (cmH2O)")
        self.label6.setBuddy(self.value6)

        self.value7 = QLabel(str(self.parameters.values_esp[6]))
        self.value7.setStyleSheet("border: 1px solid black;") 
        self.label7 = QLabel("TVi (mL)")
        self.label7.setBuddy(self.value7)

        self.value8 = QLabel(str(self.parameters.values_esp[7]))
        self.value8.setStyleSheet("border: 1px solid black;") 
        self.label8 = QLabel("TVe (mL)")
        self.label8.setBuddy(self.value8)

        self.value9 = QLabel(str(self.parameters.values_esp[8]))
        self.value9.setStyleSheet("border: 1px solid black;") 
        self.label9 = QLabel("Respiratory Rate (bpm)")
        self.label9.setBuddy(self.value9)

        self.value10 = QLabel(str(self.parameters.values_esp[9]))
        self.value10.setStyleSheet("border: 1px solid black;") 
        self.label10 = QLabel("Minute Ventilation (L/min)")
        self.label10.setBuddy(self.value10)

        self.value11 = QLabel(str(self.parameters.values_esp[10]))
        self.value11.setStyleSheet("border: 1px solid black;") 
        self.label11 = QLabel("Peak Insp Flow (lpm)")
        self.label11.setBuddy(self.value11)

        self.value12 = QLabel(str(self.parameters.values_esp[11]))
        self.value12.setStyleSheet("border: 1px solid black;") 
        self.label12 = QLabel("Mid Insp Flow (lpm)")
        self.label12.setBuddy(self.value12)

        self.values_timer = QTimer()
        self.values_timer.timeout.connect(self.update_values_tab)
        self.values_timer.start(500)

        # Set layouts
        values_layout = QGridLayout()
        values_layout.addWidget(self.label1, 0, 0)
        values_layout.addWidget(self.value1, 0, 1)
        values_layout.addWidget(self.label2, 1, 0)
        values_layout.addWidget(self.value2, 1, 1)
        values_layout.addWidget(self.label3, 2, 0)
        values_layout.addWidget(self.value3, 2, 1)
        values_layout.addWidget(self.label4, 3, 0)
        values_layout.addWidget(self.value4, 3, 1)
        values_layout.addWidget(self.label5, 4, 0)
        values_layout.addWidget(self.value5, 4, 1)
        values_layout.addWidget(self.label6, 5, 0)
        values_layout.addWidget(self.value6, 5, 1)
        values_layout.addWidget(self.label7, 0, 2)
        values_layout.addWidget(self.value7, 0, 3)
        values_layout.addWidget(self.label8, 1, 2)
        values_layout.addWidget(self.value8, 1, 3)
        values_layout.addWidget(self.label9, 2, 2)
        values_layout.addWidget(self.value9, 2, 3)
        values_layout.addWidget(self.label10,3, 2)
        values_layout.addWidget(self.value10,3, 3)
        values_layout.addWidget(self.label11,4, 2)
        values_layout.addWidget(self.value11,4, 3)
        values_layout.addWidget(self.label12,5, 2)
        values_layout.addWidget(self.value12,5, 3)
        self.valuesboxes.setLayout(values_layout)
        values_layout.setContentsMargins(0, 0, 0, 0)
    
    # -------- Plots in third tab -------------
    def create_plots(self):
        self.flowpresgraphs = QGroupBox("")
        self.flow_figure.graphbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.zero_button = QPushButton("Zero Flow and Pressure Plots")
        self.zero_button.clicked.connect(self.zero_graphs)
        self.zero_button.setFocusPolicy(Qt.NoFocus)
        self.zero_button.setStyleSheet("color: white; padding: 10px; font-size: 16px;")
        
        self.zero_button_timer = QTimer()
        self.zero_button_timer.timeout.connect(self.check_flow_on)
        self.zero_button_timer.start(500)

        self.flag = False

        self.show_IE_plot2 = QCheckBox("Show IE on plot")
        self.show_IE_plot2.setChecked(True)
        
        self.autoscale2 = QCheckBox("Autoscale Flow")
        self.autoscale2.setChecked(False)

        # self.elapsed_time_label = QLabel("00:00:00")
        # self.number_cycles_label = QLabel('0 stims')

        # self.elapsed_timer = QTimer()
        # self.elapsed_timer.timeout.connect(self.update_elapsed_time)
        # self.elapsed_timer.start(1000)

        # Set layouts
        pressflow_figures_layout = QGridLayout()
        # pressflow_figures_layout.addWidget(self.elapsed_time_label,         0, 0, 1, 1)
        # pressflow_figures_layout.addWidget(self.number_cycles_label,        0, 1, 1, 1)
        pressflow_figures_layout.addWidget(self.zero_button,                0, 0, 1, 3)
        pressflow_figures_layout.addWidget(self.show_IE_plot2,              0, 4, 1, 1)
        pressflow_figures_layout.addWidget(self.autoscale2,                 0, 5, 1, 1)
        pressflow_figures_layout.addWidget(self.flow_figure.graphbox,       1, 0, 1, 6) #,    0, 0, 1, 2
        pressflow_figures_layout.addWidget(self.pres_figure.graphbox,       2, 0, 1, 6)
        self.flowpresgraphs.setLayout(pressflow_figures_layout)

        self.show_IE_plot.stateChanged.connect(lambda state: self.show_IE_plot2.setChecked(state == Qt.Checked))
        self.show_IE_plot2.stateChanged.connect(lambda state: self.show_IE_plot.setChecked(state == Qt.Checked))
        self.autoscale.stateChanged.connect(lambda state: self.autoscale2.setChecked(state == Qt.Checked))
        self.autoscale2.stateChanged.connect(lambda state: self.autoscale.setChecked(state == Qt.Checked))

    
    # -------- ZEROING PLOTS ----------------------------------
    def check_flow_on(self):
        if self.parameters.iestate == 0:
            self.zero_button.setEnabled(True)
        else:
            self.zero_button.setEnabled(False)
        if not self.flag:
            self.parameters.zeroing = False
    
    def zero_graphs(self):
        self.flag = True
        if self.flag:
            self.parameters.zeroing = True
            if self.start_button.text() == "Start":
                self.parameters.thread_that_writes.trigger_send_to_serial(0)
            else:
                self.parameters.thread_that_writes.trigger_send_to_serial(1)
            self.flag = False

        # timer = QTimer(self)
        # timer.setSingleShot(True)
        # timer.timeout.connect(self.reset_zeroing)
        # timer.start(3000)

    # def reset_zeroing(self):
    #     self.parameters.zeroing = False
    
    # ------- PLOT ---------------------------------------------
    def create_figure(self):
        self.graphgroupbox = QGroupBox("")
        self.start_button = QPushButton("Start")
        self.start_button.setStyleSheet("color: gray; padding: 10px; font-size: 16px;")
        self.clear_button = QPushButton("Clear Data, Reset Defaults, Pause")
        self.clear_button.setStyleSheet("color: white; padding: 10px; font-size: 16px;")
        
        self.show_IE_plot = QCheckBox("Show IE on plot")
        self.show_IE_plot.setChecked(True)

        self.show_IE_plot.stateChanged.connect(self.figure.plot_IE_line)
        self.show_IE_plot.stateChanged.connect(self.flow_figure.plot_IE_line)
        
        self.autoscale = QCheckBox("Autoscale Flow")
        self.autoscale.setChecked(False)
        
        self.autoscale.stateChanged.connect(self.figure.autoscale)
        self.autoscale.stateChanged.connect(self.flow_figure.autoscale)

        self.start_button.setFocusPolicy(Qt.NoFocus)
        self.clear_button.setFocusPolicy(Qt.NoFocus)

        self.start_button.setEnabled(False)
        self.clear_button.setEnabled(False)


        self.plot_paused = False

        def toggle_button_state():
            if self.start_button.text() == "Start": # or self.value == 0
                self.parameters.update_from_main_window(self)
                self.start_button.setStyleSheet("background-color: red; color: white; padding: 10px; font-size: 16px;")
                self.start_button.setText('Stimulating. Press here to Pause')
                self.parameters.current_text = "Stimulating. Press here to Pause"
                self.test_burst_button.setEnabled(False)
                self.test_burst_button2.setEnabled(False)

                logger.info("Stimulation Enabled: " + str(datetime.datetime.now()))
                print("Stimulation Pattern Enabled: " + str(datetime.datetime.now()))
                self.parameters.thread_that_writes.trigger_send_to_serial(1)
                logger.info("Wave Type: {}; Rise: {}; Fall: {}; I ratio: {}; E ratio: {}; \
                        Stim 1 On/Off: {}; Stim 1 Peak current: {}; Stim 1 Voltage base: {}; Stim 1 Voltage amplitude: {}; \
                        Stim 2 On/Off: {}; Stim 2 Peak current: {}; Stim 2 Voltage base: {}; Stim 2 Voltage amplitude: {}; \
                        Symmetric: {}; Asymmetric # of breaths each side: {}".format(self.wave_type.currentText(), 
                                                                                     self.stim_rise_spinbox.value(), 
                                                                                     self.stim_fall_spinbox.value(), 
                                                                                     self.I_time.value(), 
                                                                                     self.E_time.value(), 
                                                                                     self.OnOff.isChecked(),
                                                                                     self.peak_current_spinbox.value(),
                                                                                     self.voltage_low_calc,
                                                                                     self.voltage_high_calc,
                                                                                     self.OnOff2.isChecked(),
                                                                                     self.peak_current_spinbox2.value(),
                                                                                     self.voltage_low_calc2,
                                                                                     self.voltage_high_calc2,
                                                                                     self.symmetric_checkbox.isChecked(),
                                                                                     self.asymmetric_spinbox.value(),
                                                                                     self.triggering_combobox.currentText(),
                                                                                     self.predictive_delay_spinbox.value()))
            elif self.start_button.text() == "Stimulating. Press here to Pause": # or self.value == 1
                self.start_button.setStyleSheet("background-color: green; color: white; padding: 10px; font-size: 16px;")
                self.start_button.setText('Start')
                logger.info("Stimulation Disabled: " + str(datetime.datetime.now()))
                print("Stimulation Disabled: " + str(datetime.datetime.now()))
                self.parameters.current_text = "Start"
                self.parameters.thread_that_writes.trigger_send_to_serial(0)
                self.test_burst_button.setEnabled(True)
                self.test_burst_button2.setEnabled(True)

        def clear_no_update():
            self.start_button.setStyleSheet("background-color: green; color: white; padding: 10px; font-size: 16px;")
            self.start_button.setText('Start')
            self.parameters.current_text == "Start"
            self.plot_paused = True
            self.figure.clear_plot()
            self.flow_figure.clear_plot()
            self.pres_figure.clear_plot()
            logger.info("Stimulation Disabled and Cleared: " + str(datetime.datetime.now()))
            print("Stimulation Disabled and Cleared: " + str(datetime.datetime.now()))
            self.parameters.thread_that_writes.trigger_send_to_serial(4)
            self.wave_type.setCurrentIndex(0)
            self.stim_rise_spinbox.setValue(0.050)
            self.stim_fall_spinbox.setValue(0.050)
            self.pulse_train_freq_spinbox.setValue(30)
            self.triggering_combobox.setCurrentIndex(0)
            self.I_time.setValue(1.5)
            self.resp_rate_spinbox.setValue(18)
            self.predictive_delay_spinbox.setValue(0)
            self.number_breaths_avg.setValue(3)
            self.OnOff.setChecked(True)
            self.peak_current_spinbox.setValue(100)
            self.current_high_spinbox.setValue(50)
            self.current_low_spinbox.setValue(0)
            self.OnOff2.setChecked(False)
            self.peak_current_spinbox2.setValue(100)
            self.current_high_spinbox2.setValue(50)
            self.current_low_spinbox2.setValue(0)
            self.same_rf_checkbox.setChecked(False)
            self.same_settings_checkbox.setChecked(False)
            self.symmetric_checkbox.setChecked(False)
            self.asymmetric_spinbox.setValue(1)
            self.current_bias_checkbox.setChecked(False)
            self.current_bias_checkbox2.setChecked(False)
            self.I_threshold.setValue(15)
            self.E_threshold.setValue(-10)
            
        self.start_button.clicked.connect(toggle_button_state)
        self.start_button.clicked.connect(self.figure.toggle_plot_state)
        self.start_button.clicked.connect(self.flow_figure.toggle_plot_state)

        self.clear_button.clicked.connect(clear_no_update)

        self.figure.graphbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Set layouts
        layout = QGridLayout()
        layout.addWidget(self.start_button,               0, 0, 1, 2)
        layout.addWidget(self.clear_button,               0, 2, 1, 2)
        layout.addWidget(self.show_IE_plot,               0, 4, 1, 1)
        layout.addWidget(self.autoscale,                  0, 5, 1, 1)
        self.graphgroupbox.setLayout(layout)
        layout.setContentsMargins(0, 0, 0, 0)

    # ------- EXAMPLE STIM PLOT ---------------------------------------------
    # def create_example_figure(self):
    #     self.example_groupbox = QGroupBox("")

    #     self.example_figure.start_plot
    #     self.parameters.update_from_main_window(self)
    #     # self.example_figure.graphbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    #     # Set layouts
    #     example_figure_layout = QGridLayout()
    #     example_figure_layout.addWidget(self.example_figure.graphbox) #,    0, 0, 1, 2
    #     self.example_groupbox.setLayout(example_figure_layout)
    
    # COM - QComboBox
    def select_com_port(self):
        self.GUI_setup = QGroupBox("")
        self.com_port_list = QComboBox()
        self.com_port_list.objectName = 'Placeholder'

        self.com_port_list.addItems([port.device for port in list_ports.comports() if "UART" in port.description])

        self.com_port_label = QLabel("Select COM Port:")
        self.com_port_label.setBuddy(self.com_port_list)
        self.com_connect = QLabel("Connect")
        self.com_port_list.currentIndexChanged.connect(self.connect_to_com)
        # self.connect_to_com()

        self.com_port_timer = pg.QtCore.QTimer(self)
        self.com_port_timer.timeout.connect(self.update_com_port_list)
        self.com_port_timer.timeout.connect(self.connect_to_com)
        self.com_port_timer.start(1000)
        
        layout = QGridLayout()
        layout.addWidget(self.com_port_label,                    0,0,1,1)
        layout.addWidget(self.com_port_list,                     0,1,1,2)
        layout.addWidget(self.com_connect,                       0,3,1,1)
        self.GUI_setup.setLayout(layout)
        layout.setContentsMargins(0, 0, 0, 0)
    # _____________________________________________________________________
    
    def update_com_port_list(self):
        current_ports = [self.com_port_list.itemText(i) for i in range(self.com_port_list.count())]
        new_ports = [port.device for port in list_ports.comports() if "UART" in port.description]

        # Remove existing items not in new list
        for i in range(self.com_port_list.count()):
            item = self.com_port_list.itemText(i)
            if item not in new_ports:
                self.com_port_list.removeItem(i)

        # Add new items not in current list
        for port in new_ports:
            if port not in current_ports:
                self.com_port_list.addItem(port)

    
    def connect_to_com(self):
        self.parameters.com_port = self.com_port_list.currentText()
        self.update_parameters_from_gui()

        if self.parameters.read_thread is not None:
            self.com_connect.setText("Connected")
            self.com_connect.setStyleSheet("background-color: green; color: white;")
            if self.parameters.flag == False:
                self.start_button.setStyleSheet("background-color: green; color: white; padding: 10px; font-size: 16px;")
                self.parameters.flag = True
        else:
            self.com_connect.setText("Not connected")
            self.com_connect.setStyleSheet("background-color: red; color: white;")
            self.start_button.setStyleSheet("color: gray; padding: 10px; font-size: 16px;")
            self.start_button.setText('Start')
            self.parameters.flag = False
            self.create_threads()

    # _____________________________________________________________________

    # Master Stim Settings - QGroupBox
    def create_wave_settings(self):
        self.GUI_wave_settings = QGroupBox("Waveform Settings")

        # Type of Stim Wave
        self.wave_type = QComboBox()
        self.wave_type.addItems(['Trapezoid','Triangle','Sine', 'HFOV', 'Bilevel'])
        self.wave_type.setEnabled(True)

        self.wave_type_label = QLabel("Wave Type")
        self.wave_type_label.setBuddy(self.wave_type)

        # Connect wave type combo box signal to toggle the rise and fall settings
        self.wave_type.currentIndexChanged.connect(self.toggle_rise_fall_settings)

        # Rise/Fall of Stim
        self.stim_rise_spinbox = QDoubleSpinBox()
        self.stim_rise_spinbox.setValue(0.05)
        self.stim_rise_spinbox.setSuffix(' s')
        self.stim_rise_spinbox.setSingleStep(0.05)
        self.stim_rise_spinbox.setDecimals(2)  # Set the number of decimals to 3
        self.stim_rise_spinbox.setEnabled(True)
        self.stim_rise_spinbox.objectName = 'PlaceHolder'

        self.stim_risefall_label = QLabel("Rise/Fall")
        self.stim_risefall_label.setBuddy(self.stim_rise_spinbox)

        self.stim_rise_label = QLabel("Rise")
        self.stim_rise_label.setBuddy(self.stim_rise_spinbox)

        self.stim_fall_spinbox = QDoubleSpinBox()
        self.stim_fall_spinbox.setValue(0.05)
        self.stim_fall_spinbox.setSuffix(' s')
        self.stim_fall_spinbox.setSingleStep(0.05)
        self.stim_fall_spinbox.setDecimals(2)  # Set the number of decimals to 3
        self.stim_fall_spinbox.setEnabled(True)
        self.stim_fall_spinbox.objectName = 'PlaceHolder'

        self.stim_fall_label = QLabel("Fall")
        self.stim_fall_label.setBuddy(self.stim_fall_spinbox)
        
        # HFOV Settings
        self.hfov_settings = QLabel("HFOV Settings")
        
        self.hfov_freq_spinbox = QSpinBox()
        self.hfov_freq_spinbox.setValue(3)
        self.hfov_freq_spinbox.setSuffix(' ST/insp')
        self.hfov_freq_spinbox.setRange(1,5)
        self.hfov_freq_spinbox.setEnabled(False)
        self.hfov_freq_spinbox.objectName = 'PlaceHolder'

        self.hfov_freq_label = QLabel("Freq")
        self.hfov_freq_label.setBuddy(self.hfov_freq_spinbox)

        self.hfov_base_spinbox = QSpinBox()
        self.hfov_base_spinbox.setValue(0)
        self.hfov_base_spinbox.setSuffix(' %')
        self.hfov_base_spinbox.setRange(0,100)
        self.hfov_base_spinbox.setEnabled(False)
        self.hfov_base_spinbox.objectName = 'PlaceHolder'

        self.hfov_base_label = QLabel("Base")
        self.hfov_base_label.setBuddy(self.hfov_base_spinbox)

        self.hfov_freq_spinbox.valueChanged.connect(self.update_HFOV)
        self.wave_type.currentIndexChanged.connect(self.update_HFOV)

        # Bilevel Settings

        self.bilevel_settings = QLabel("Level 1 Settings")

        self.bilevel_v_spinbox = QSpinBox()
        self.bilevel_v_spinbox.setValue(50)
        self.bilevel_v_spinbox.setRange(0,100)
        self.bilevel_v_spinbox.setSuffix(' %')
        self.bilevel_v_spinbox.setEnabled(False)

        self.bilevel_v_label = QLabel("Voltage")
        self.bilevel_v_label.setBuddy(self.bilevel_v_spinbox)


        self.bilevel_t_spinbox = QSpinBox()
        self.bilevel_t_spinbox.setValue(50)
        self.bilevel_t_spinbox.setRange(0,100)
        self.bilevel_t_spinbox.setSuffix(' %')
        self.bilevel_t_spinbox.setEnabled(False)

        self.bilevel_t_label = QLabel("Time")
        self.bilevel_t_label.setBuddy(self.bilevel_t_spinbox)

        # I:E TIME label
        self.IE_time = QLabel("Patient Settings")

        # Respiration Rate
        self.resp_rate_spinbox = QDoubleSpinBox()
        self.resp_rate_spinbox.setValue(18)
        self.resp_rate_spinbox.setSuffix(' bpm')
        self.resp_rate_spinbox.setDecimals(0)
        self.resp_rate_spinbox.setRange(3,50)
        self.resp_rate_spinbox.setEnabled(True)
        self.resp_rate_spinbox.objectName = 'PlaceHolder'

        self.resp_rate_label = QLabel("Stim RR")
        self.resp_rate_label.setBuddy(self.resp_rate_spinbox)

        # I in I:E Ratio
        self.I_ratio_spinbox = QLabel("1.0: 1.0")

        self.I_E_ratio_label = QLabel("Stim I:E")
        self.I_E_ratio_label.setBuddy(self.I_ratio_spinbox)

        self.resp_rate_value = QLabel("RR: 18 bpm")
        self.I_time_value = QLabel("I time: 1.5 s")

        self.resp_I_timer = QTimer(self)
        self.resp_I_timer.timeout.connect(self.update_resp_I)
        self.resp_I_timer.start(100)

        # E in I:E Ratio
        self.E_ratio_spinbox = QLabel("1.00")

        self.I_time = QDoubleSpinBox()
        self.I_time.setValue(1.5)
        self.I_time.setRange(0.1,3)
        self.I_time.setSingleStep(0.1)
        self.I_time.setReadOnly(False)
        self.I_time.setSuffix(' s')
        self.I_time_label = QLabel('I time ')

        self.E_time = QDoubleSpinBox()
        self.E_time.setValue(1.5)
        self.E_time.setRange(0.1,5)
        self.E_time.setSingleStep(0.1)
        self.E_time.setReadOnly(False)
        self.E_time.setSuffix(' s')
        self.E_time_label = QLabel('E time ')

        self.I_time.valueChanged.connect(self.update_values_I_time) #valueChanged # editingFinished
        self.resp_rate_spinbox.valueChanged.connect(self.update_values_I_time)

        self.I_time.valueChanged.connect(self.synch_update_values_I_time) #valueChanged # editingFinished
        self.resp_rate_spinbox.valueChanged.connect(self.synch_update_values_I_time)

        self.I_time.valueChanged.connect(self.update_HFOV)
        self.resp_rate_spinbox.valueChanged.connect(self.update_HFOV)

        self.I_time.valueChanged.connect(self.predictive_bounds)

        self.synch_I_timer = QTimer(self)
        self.synch_I_timer.timeout.connect(self.synch_update_values_I_time)
        # self.synch_I_timer.timeout.connect(self.predictive_bounds)
        self.synch_I_timer.start(1000)

        self.i_time = self.I_time.value()
        self.e_time = (60 / self.resp_rate_spinbox.value()) - self.i_time

        # Pulse train frequency
        self.pulse_train_freq_spinbox = QSpinBox()
        self.pulse_train_freq_spinbox.setValue(30)
        self.pulse_train_freq_spinbox.setRange(1,200)
        self.pulse_train_freq_spinbox.setSuffix('Hz')
        self.pulse_train_freq_spinbox.setSingleStep(5)
        self.pulse_train_freq_spinbox.setEnabled(True)
        self.pulse_train_freq_spinbox.objectName = 'PlaceHolder'

        self.pulse_train_freq_label = QLabel("Pulse Train Frequency")
        self.pulse_train_freq_label.setBuddy(self.pulse_train_freq_spinbox)

        self.example_figure.start_plot
        # self.parameters.update_from_main_window(self)
        # self.example_figure.graphbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)


        layout = QGridLayout()
        layout.addWidget(self.wave_type_label,                   0,1,1,1)
        layout.addWidget(self.wave_type,                         0,2,1,4)
        layout.addWidget(self.stim_risefall_label,               1,1,1,1)
        layout.addWidget(self.stim_rise_spinbox,                 1,2,1,2)
        layout.addWidget(self.stim_fall_spinbox,                 1,4,1,2)
        layout.addWidget(self.hfov_settings,                     2,1,1,1)
        layout.addWidget(self.hfov_freq_label,                   2,2,1,1)
        layout.addWidget(self.hfov_freq_spinbox,                 2,3,1,1)
        layout.addWidget(self.hfov_base_label,                   2,4,1,1)
        layout.addWidget(self.hfov_base_spinbox,                 2,5,1,1)
        layout.addWidget(self.bilevel_settings,                  3,1,1,1)
        layout.addWidget(self.bilevel_t_label,                   3,2,1,1)
        layout.addWidget(self.bilevel_t_spinbox,                 3,3,1,1)
        layout.addWidget(self.bilevel_v_label,                   3,4,1,1)
        layout.addWidget(self.bilevel_v_spinbox,                 3,5,1,1)
        layout.addWidget(self.IE_time,                           4,1,1,1)
        layout.addWidget(self.I_time_label,                      4,2,1,1)
        layout.addWidget(self.I_time,                            4,3,1,1)
        layout.addWidget(self.resp_rate_label,                   4,4,1,1)
        layout.addWidget(self.resp_rate_spinbox,                 4,5,1,1)
        layout.addWidget(self.I_E_ratio_label,                   5,2,1,1)
        layout.addWidget(self.I_ratio_spinbox,                   5,3,1,1)
        layout.addWidget(self.resp_rate_value,                   5,4,1,1)
        layout.addWidget(self.I_time_value,                      5,5,1,1)
        layout.addWidget(self.pulse_train_freq_label,            6,1,1,1)
        layout.addWidget(self.pulse_train_freq_spinbox,          6,2,1,4)
        layout.addWidget(self.example_figure.graphbox,           0,0,7,1, alignment=Qt.AlignCenter)
        self.GUI_wave_settings.setLayout(layout)
        # layout.setContentsMargins(0, 0, 0, 0)

    
    # Master Stim Settings - QGroupBox
    def create_trigger_settings(self):
        self.GUI_trigger_settings = QGroupBox("Trigger Settings")

        # Triggering Type
        self.triggering_combobox = QComboBox()
        self.triggering_combobox.addItems(["Mandatory", "Synchronous", "Synchronous Triggered", "Predictive", "Predictive Triggered"])
        self.triggering_label = QLabel("Triggering Type")

        self.flowsense_connect_label = QLabel("")

        self.flowconnect_timer = pg.QtCore.QTimer(self)
        self.flowconnect_timer.timeout.connect(self.check_flowsense)
        self.flowconnect_timer.start(500)

        self.triggering_combobox.currentIndexChanged.connect(self.toggle_trigger_settings)
        self.triggering_combobox.currentIndexChanged.connect(self.update_resp)
        
        self.resp_timer = pg.QtCore.QTimer(self)
        self.resp_timer.timeout.connect(self.update_resp)
        self.resp_timer.start(1000)

        # Synch insp or exp option
        self.button_group = QButtonGroup()
        self.radio_button1 = QRadioButton("Insp")
        self.radio_button2 = QRadioButton("Exp")
        self.radio_button3 = QRadioButton("Exp-Trig Insp")
        self.button_group.addButton(self.radio_button1)
        self.button_group.addButton(self.radio_button2)
        self.button_group.addButton(self.radio_button3)
        self.radio_button1.setEnabled(False)
        self.radio_button2.setEnabled(False)
        self.radio_button3.setEnabled(False)
        
        self.radio_button3.clicked.connect(self.toggle_radio_button)
        self.radio_button2.clicked.connect(self.toggle_radio_button)
        self.radio_button1.clicked.connect(self.toggle_radio_button)
        
        ## PHOEBE UNCOMMENT HERE IF YOU WANT TO BE ABLE TO CHANGE RESP AND I:E RATIO
        # self.I_ratio_spinbox.valueChanged.connect(self.update_values_IE)
        # self.E_ratio_spinbox.valueChanged.connect(self.update_values_IE)
        # self.resp_rate_spinbox.valueChanged.connect(self.update_values_IE)


        # Predictive Triggered delay
        self.predictive_delay_spinbox = QDoubleSpinBox()
        self.predictive_delay_spinbox.setValue(0)
        self.predictive_delay_spinbox.setSuffix(' ms')
        self.predictive_delay_spinbox.setRange(-1000,5000)
        self.predictive_delay_spinbox.setSingleStep(50)
        self.predictive_delay_spinbox.setEnabled(False)
        self.predictive_delay_spinbox.objectName = 'PlaceHolder'

        self.predictive_delay_label = QLabel("Delay")
        self.predictive_delay_label.setBuddy(self.predictive_delay_spinbox)

        # self.predictive_delay_spinbox.valueChanged.connect(self.predictive_bounds)

        # Predictive Triggered delay
        self.number_breaths_avg = QSpinBox()
        self.number_breaths_avg.setValue(3)
        self.number_breaths_avg.setSuffix(' breaths')
        self.number_breaths_avg.setRange(1,12)
        self.number_breaths_avg.setEnabled(False)
        self.number_breaths_avg.objectName = 'PlaceHolder'

        self.number_breaths_avg_label = QLabel("# Breaths Averaged")
        self.number_breaths_avg_label.setBuddy(self.number_breaths_avg)

        # Predictive Triggered delay
        self.I_threshold = QSpinBox()
        self.I_threshold.setValue(15)
        self.I_threshold.setSuffix(' mL/min')
        self.I_threshold.setRange(1,100)

        self.I_threshold_label = QLabel("Insp: ")

        # Predictive Triggered delay
        self.E_threshold = QSpinBox()
        self.E_threshold.setMinimum(-50)
        self.E_threshold.setMaximum(-1)
        self.E_threshold.setValue(-10)
        self.E_threshold.setSuffix(' mL/min')
        self.E_threshold.setRange(-50, -1)

        self.E_threshold_label = QLabel("Exp: ")

        self.IE_threshold_label = QLabel("Trigger Threshold")
        self.IE_threshold_label.setBuddy(self.E_threshold)

        self.I_threshold.valueChanged.connect(self.IEthreshold_change)
        self.E_threshold.valueChanged.connect(self.IEthreshold_change)

    
        layout = QGridLayout()
        layout.addWidget(self.triggering_label,                  1,0,1,1)
        layout.addWidget(self.triggering_combobox,               1,1,1,3)
        layout.addWidget(self.flowsense_connect_label,           1,4,1,1)
        layout.addWidget(self.radio_button1,                     2,1,1,1)
        layout.addWidget(self.radio_button2,                     2,2,1,1)
        layout.addWidget(self.radio_button3,                     2,3,1,2)
        # layout.addWidget(self.IE_time,                           3,0,1,1)
        # layout.addWidget(self.I_time_label,                      3,1,1,1)
        # layout.addWidget(self.I_time,                            3,2,1,1)
        # layout.addWidget(self.resp_rate_label,                   3,3,1,1)
        # layout.addWidget(self.resp_rate_spinbox,                 3,4,1,1)
        # layout.addWidget(self.I_E_ratio_label,                   4,1,1,1)
        # layout.addWidget(self.I_ratio_spinbox,                   4,2,1,1)
        # layout.addWidget(self.resp_rate_value,                   4,3,1,1)
        # layout.addWidget(self.I_time_value,                      4,4,1,1)
        layout.addWidget(self.predictive_delay_label,            3,0,1,1)
        layout.addWidget(self.predictive_delay_spinbox,          3,1,1,4)
        layout.addWidget(self.IE_threshold_label,                4,0,1,1)
        layout.addWidget(self.I_threshold_label,                 4,1,1,1)
        layout.addWidget(self.I_threshold,                       4,2,1,1)
        layout.addWidget(self.E_threshold_label,                 4,3,1,1)
        layout.addWidget(self.E_threshold,                       4,4,1,1)
        # layout.addWidget(self.show_IE_plot,                      10,1,1,1)
        layout.addWidget(self.number_breaths_avg_label,          5,0,1,1)
        layout.addWidget(self.number_breaths_avg,                5,1,1,4)
        self.GUI_trigger_settings.setLayout(layout)
        # layout.setContentsMargins(0, 0, 0, 0)

    # ------- Stim 1 Group ---------------------------------------------
    # Stim1 - QGroupBox
    def create_stim1_settings(self):
        self.GUI_stim1_settings = QGroupBox("AWG1 Settings")
        self.GUI_stim1_settings.setStyleSheet('QGroupBox { border: 1px solid blue}')
        # Stimulator 1 Toggle button
        self.OnOff = QCheckBox("Stim1 On")
        self.OnOff.setCheckable(True)
        self.OnOff.setChecked(True)

        self.OnOff.stateChanged.connect(self.toggle_onoff_settings)
        
        #Peak current amplitude
        self.peak_current_spinbox = QSpinBox()
        self.peak_current_spinbox.setSuffix(' mA')
        self.peak_current_spinbox.setMaximum(1000)
        self.peak_current_spinbox.setRange(0,1000)
        self.peak_current_spinbox.setValue(100)
        self.peak_current_spinbox.setSingleStep(5)
        self.peak_current_spinbox.setEnabled(True)
        self.peak_current_spinbox.objectName = 'PlaceHolder'

        self.peak_current_label = QLabel("Peak Current:")
        self.peak_current_label.setBuddy(self.peak_current_spinbox)

        # AWG High Setting - % of Peak Current
        self.current_settings = QGroupBox("")

        self.current_high_spinbox = QSpinBox()
        self.current_high_spinbox.setValue(50)
        self.current_high_spinbox.setSuffix('%')
        self.current_high_spinbox.setRange(0,100)
        self.current_high_spinbox.setSingleStep(5)
        self.current_high_spinbox.setEnabled(True)
        self.current_high_spinbox.objectName = 'PlaceHolder'

        self.current_high_label = QLabel('High (% of {} mA):'.format(self.peak_current_spinbox.value()))
        self.current_high_label.setBuddy(self.current_high_spinbox)

        self.current_high_calc = self.peak_current_spinbox.value()/100*self.current_high_spinbox.value()
        self.voltage_high_calc = self.current_high_spinbox.value()/10
        self.current_high_calclabel = QLabel('{:.1f} mA, {:.1f} V'.format(self.current_high_calc, self.voltage_high_calc))

        # AWG Low Setting - % of Peak Current
        self.current_low_spinbox = QSpinBox()
        self.current_low_spinbox.setValue(0)
        self.current_low_spinbox.setSuffix('%')
        self.current_low_spinbox.setRange(0,100)
        self.current_low_spinbox.setSingleStep(5)
        self.current_low_spinbox.setEnabled(True)
        self.current_low_spinbox.objectName = 'PlaceHolder'

        self.current_low_label = QLabel('Low (% of {} mA):'.format(self.peak_current_spinbox.value()))
        self.current_low_label.setBuddy(self.current_low_spinbox)

        self.current_low_calc = self.peak_current_spinbox.value()/100*self.current_low_spinbox.value()
        self.voltage_low_calc = self.current_low_spinbox.value()/10
        self.current_low_calclabel = QLabel('{:.1f} mA, {:.1f} V'.format(self.current_low_calc, self.voltage_low_calc))
        
        # Update peak and base current levels
        self.sync_timer_current = QTimer()
        # self.sync_timer_current.timeout.connect(self.update_values_current)
        self.peak_current_spinbox.valueChanged.connect(self.update_values_current)
        self.current_high_spinbox.valueChanged.connect(self.update_values_current)
        self.current_low_spinbox.valueChanged.connect(self.update_values_current)

        # AWG Bias Voltage - % of Peak Current
        self.current_bias_checkbox = QCheckBox('Bias')
        self.current_bias_checkbox.stateChanged.connect(self.bias_change)
        self.current_low_spinbox.valueChanged.connect(self.bias_change_num)

        # Lay out grid for high and low current values
        layout = QGridLayout()
        layout.addWidget(self.current_high_label,               0,3,1,1)
        layout.addWidget(self.current_high_spinbox,             0,4,1,2)
        layout.addWidget(self.current_high_calclabel,           0,6,1,1)
        layout.addWidget(self.current_low_label,                1,3,1,1)
        layout.addWidget(self.current_low_spinbox,              1,4,1,2)
        layout.addWidget(self.current_low_calclabel,            1,6,1,1)
        layout.addWidget(self.current_bias_checkbox,            1,7,1,1)

        self.current_settings.setLayout(layout)
        # layout.setContentsMargins(0, 0, 0, 0)

        # Rise time
        self.rise_fall_settings = QGroupBox("")

        self.rise_spinbox = QDoubleSpinBox()
        self.rise_spinbox.setValue(0.1)
        self.rise_spinbox.setSuffix(' s')
        self.rise_spinbox.setSingleStep(0.1)
        self.rise_spinbox.setEnabled(True)
        self.rise_spinbox.objectName = 'PlaceHolder'

        self.rise_label = QLabel("Rise:")
        self.rise_label.setBuddy(self.rise_spinbox)

        # Fall time
        self.fall_spinbox = QDoubleSpinBox()
        self.fall_spinbox.setValue(0.1)
        self.fall_spinbox.setSuffix(' s')
        self.fall_spinbox.setSingleStep(0.1)
        self.fall_spinbox.setEnabled(True)
        self.fall_spinbox.objectName = 'PlaceHolder'

        self.fall_label = QLabel("Fall:")
        self.fall_label.setBuddy(self.fall_spinbox)

        # Same settings across both stimulators checkbox
        self.sync_timer_rf = QTimer()
        self.sync_timer_rf.timeout.connect(self.update_values_rf)

        self.same_rf_checkbox = QCheckBox("Same as rise")
        self.same_rf_checkbox.stateChanged.connect(self.toggle_rf_settings)

        layout = QGridLayout()
        layout.addWidget(self.rise_label,                        0,0,1,1)
        layout.addWidget(self.rise_spinbox,                      0,1,1,2)
        layout.addWidget(self.fall_label,                        1,0,1,1)
        layout.addWidget(self.fall_spinbox,                      1,1,1,2)
        layout.addWidget(self.same_rf_checkbox,                  2,1,1,3)
        self.rise_fall_settings.setLayout(layout)

        self.OnOff.stateChanged.connect(lambda state: self.current_high_spinbox.setEnabled(state == Qt.Checked))
        self.OnOff.stateChanged.connect(lambda state: self.current_low_spinbox.setEnabled(state == Qt.Checked))
        self.OnOff.stateChanged.connect(lambda state: self.rise_spinbox.setEnabled(state == Qt.Checked))
        self.OnOff.stateChanged.connect(lambda state: self.fall_spinbox.setEnabled(state == Qt.Checked))
        self.OnOff.stateChanged.connect(lambda state: self.peak_current_spinbox.setEnabled(state == Qt.Checked))

        
        def update_elapsed_time():
            self.time_passed = self.time_passed.addMSecs(500)
        
        self.test_burst_flag = False
        self.time_passed = QTime(0,0)
        self.burst_timer = QTimer(self)
        self.burst_timer.timeout.connect(update_elapsed_time)
        self.burst_timer.start(500)  # Update elapsed time every 500 ms

        def start_test_burst():
            self.parameters.thread_that_writes.trigger_send_to_serial(2)
            self.test_burst_button.setStyleSheet("background-color: green; color: white; padding: 15px; font-size: 24px;")
            self.test_burst_button.setEnabled(False)
            self.test_burst_flag = True
            self.time_passed = QTime(0,0)
            self.start_button.setEnabled(False)
            # self.elapsed_time = self.elapsed_time.addSecs(1)
        
        def stop_test_burst():
            if self.parameters.switchstate == '1' and self.parameters.triggering_type != "Synchronous" and self.parameters.triggering_type != "Predictive" and self.start_button.text() == "Start":
                self.test_burst_button.setEnabled(True)
                self.burst_mode_alert.setText("")
                self.burst_mode_alert.setStyleSheet("color: white")
            
            else:
                self.test_burst_button.setEnabled(False)
            
            if self.parameters.burst_flag_ext1 == 1 and not self.test_burst_flag and self.parameters.triggering_type != "Synchronous" and self.parameters.triggering_type != "Predictive":
                start_test_burst()
                self.test_burst_flag = True
                # time.sleep(50)
            
            if self.parameters.triggering_type == "Synchronous" or self.parameters.triggering_type == "Predictive" or self.start_button.text() == "Stimulating. Press here to Pause":
                if self.parameters.triggering_type == "Synchronous" or self.parameters.triggering_type == "Predictive":
                    self.test_burst_button.setEnabled(False)
                    self.burst_mode_alert.setText("SWITCH TO MANDATORY TO ENABLE")
                    self.burst_mode_alert.setStyleSheet("color: red")
                    self.test_burst_button2.setEnabled(False)
                    self.burst_mode_alert2.setText("SWITCH TO MANDATORY TO ENABLE")
                    self.burst_mode_alert2.setStyleSheet("color: red")
                if self.start_button.text() == "Stimulating. Press here to Pause":
                    self.burst_mode_alert.setText("PAUSE STIM TO ENABLE")
                    self.burst_mode_alert.setStyleSheet("color: red")
                    self.burst_mode_alert2.setText("PAUSE STIM TO ENABLE")
                    self.burst_mode_alert2.setStyleSheet("color: red")
            else:
                self.burst_mode_alert.setText("")
                self.burst_mode_alert.setStyleSheet("color: white")
                self.burst_mode_alert2.setText("")
                self.burst_mode_alert2.setStyleSheet("color: white")
            
            if self.time_passed.msecsSinceStartOfDay() > 500 and self.parameters.burst_flag1 != 1 and self.test_burst_flag:#self.parameters.burst_flag_ext == 0
                self.test_burst_button.setStyleSheet("border: 1px solid #404040; background-color: solid gray; color: white; padding: 15px; font-size: 24px;")
                self.start_button.setEnabled(True)
                self.test_burst_flag = False
                self.test_burst_button.setEnabled(True)
        
        # def clicked_on_press(self, event):
        #     if event.button() == 1:  # Left mouse button
        #         self.clicked_on_press.emit()

        self.test_burst_button = QPushButton("Test Burst")
        self.test_burst_button.setFocusPolicy(Qt.NoFocus)
        # self.test_burst_button.clicked_on_press = pyqtSignal()  # Create a custom signal for the button
        # self.test_burst_button.mousePressEvent = clicked_on_press()  # Override mousePressEvent
        self.test_burst_button.setStyleSheet("border: 1px solid #404040; background-color: solid gray; color: white; padding: 15px; font-size: 24px;")
        self.test_burst_button.setEnabled(True)
        self.test_burst_button.clicked.connect(start_test_burst)

        self.test_burst_timer = QTimer(self)
        self.test_burst_timer.timeout.connect(stop_test_burst)
        self.test_burst_timer.start(20)

        self.burst_mode_alert = QLabel("")

        #### End code for stim 1 test burst

        self.current_settings.setStyleSheet("QGroupBox { border: 1px solid #404040; }")

        layout = QGridLayout()
        layout.addWidget(self.OnOff,                             0,0,1,1)
        layout.addWidget(self.peak_current_label,                1,0,1,1)
        layout.addWidget(self.peak_current_spinbox,              1,1,1,1)
        layout.addWidget(self.current_settings,                  4,0,2,4) #Change last # to 2 if you add rise_fall_settings back in
        layout.addWidget(self.test_burst_button,                 6,1,1,2)
        layout.addWidget(self.burst_mode_alert,                  7,1,1,2, alignment=Qt.AlignCenter)
        self.GUI_stim1_settings.setLayout(layout)
        # layout.setContentsMargins(0, 0, 0, 0)


    # ------- Stim 2 Group ---------------------------------------------
    # Stim2 - QGroupBox
    def create_stim2_settings(self):
        self.GUI_stim2_settings = QGroupBox("AWG2 Settings")
        self.GUI_stim2_settings.setStyleSheet('QGroupBox { border: 1px solid red}')
        # Stimulator 2 Toggle button
        self.OnOff2 = QCheckBox("Stim2 On")
        self.OnOff2.setCheckable(True)
        self.OnOff2.setChecked(False)

        self.OnOff2.stateChanged.connect(self.toggle_onoff_settings)
        
        # Peak current amplitude
        self.peak_current_spinbox2 = QSpinBox()
        self.peak_current_spinbox2.setSuffix(' mA')
        self.peak_current_spinbox2.setRange(0,1000)
        self.peak_current_spinbox2.setMaximum(1000)
        self.peak_current_spinbox2.setValue(100)
        self.peak_current_spinbox2.setSingleStep(5)
        self.peak_current_spinbox2.setEnabled(False)
        self.peak_current_spinbox2.objectName = 'PlaceHolder'

        self.peak_current_label2 = QLabel("Peak Current:")
        self.peak_current_label2.setBuddy(self.peak_current_spinbox2)

        # AWG High Setting - % of Peak Current
        self.current_settings2 = QGroupBox("")

        self.current_high_spinbox2 = QSpinBox()
        self.current_high_spinbox2.setValue(50)
        self.current_high_spinbox2.setSuffix('%')
        self.current_high_spinbox2.setRange(0,100)
        self.current_high_spinbox2.setSingleStep(5)
        self.current_high_spinbox2.setEnabled(False)
        self.current_high_spinbox2.objectName = 'PlaceHolder'

        self.current_high_label2 = QLabel("High (% of 10mA):")
        self.current_high_label2.setBuddy(self.current_high_spinbox2)

        self.current_high_calc2 = self.peak_current_spinbox2.value()/100*self.current_high_spinbox2.value()
        self.voltage_high_calc2 = self.current_high_spinbox2.value()/10
        self.current_high_calclabel2 = QLabel('{:.1f} mA, {:.1f} V'.format(self.current_high_calc2, self.voltage_high_calc2))

        # AWG Low Setting - % of Peak Current
        self.current_low_spinbox2 = QSpinBox()
        self.current_low_spinbox2.setValue(0)
        self.current_low_spinbox2.setSuffix('%')
        self.current_low_spinbox2.setRange(0,100)
        self.current_low_spinbox2.setSingleStep(5)
        self.current_low_spinbox2.setEnabled(False)
        self.current_low_spinbox2.objectName = 'PlaceHolder'

        self.current_low_label2 = QLabel("Low (% of 10mA):")
        self.current_low_label2.setBuddy(self.current_low_spinbox2)

        self.current_low_calc2 = self.peak_current_spinbox.value()/100*self.current_low_spinbox2.value()
        self.voltage_low_calc2 = self.current_low_spinbox2.value()/10
        self.current_low_calclabel2 = QLabel('{:.1f} mA, {:.1f} V'.format(self.current_low_calc2, self.voltage_low_calc2))

        # Update peak and base current levels
        # self.sync_timer_current.timeout.connect(self.update_values_current)
        self.peak_current_spinbox2.valueChanged.connect(self.update_values_current)
        self.current_high_spinbox2.valueChanged.connect(self.update_values_current)
        self.current_low_spinbox2.valueChanged.connect(self.update_values_current)

        # AWG Bias Voltage - % of Peak Current
        self.current_bias_checkbox2 = QCheckBox('Bias')
        self.current_bias_checkbox2.stateChanged.connect(self.bias_change2)
        self.current_low_spinbox2.valueChanged.connect(self.bias_change_num2)

        # Layout grid for high and low current
        layout = QGridLayout()
        layout.addWidget(self.current_high_label2,               0,3,1,1)
        layout.addWidget(self.current_high_spinbox2,             0,4,1,2)
        layout.addWidget(self.current_high_calclabel2,           0,6,1,2)
        layout.addWidget(self.current_low_label2,                1,3,1,1)
        layout.addWidget(self.current_low_spinbox2,              1,4,1,2)
        layout.addWidget(self.current_low_calclabel2,            1,6,1,2)
        layout.addWidget(self.current_bias_checkbox2,            1,7,1,1)
        self.current_settings2.setLayout(layout)
        # layout.setContentsMargins(0, 0, 0, 0)

        # Same settings across both stimulators checkbox
        self.sync_timer = QTimer()
        self.same_settings_checkbox = QCheckBox("Same as Stim1")
        self.same_settings_checkbox.clicked.connect(self.update_values)
        self.same_settings_checkbox.stateChanged.connect(self.toggle_same_settings)

        # Rise time
        self.rise_fall_settings2 = QGroupBox("")

        self.rise_spinbox2 = QDoubleSpinBox()
        self.rise_spinbox2.setValue(0.1)
        self.rise_spinbox2.setSuffix(' s')
        self.rise_spinbox2.setSingleStep(0.1)
        self.rise_spinbox2.setEnabled(False)
        self.rise_spinbox2.objectName = 'PlaceHolder'

        self.rise_label2 = QLabel("Rise:")
        self.rise_label2.setBuddy(self.rise_spinbox2)

        # Fall time
        self.fall_spinbox2 = QDoubleSpinBox()
        self.fall_spinbox2.setValue(0.1)
        self.fall_spinbox2.setSuffix(' s')
        self.fall_spinbox2.setSingleStep(0.1)
        self.fall_spinbox2.setEnabled(False)
        self.fall_spinbox2.objectName = 'PlaceHolder'

        self.fall_label2 = QLabel("Fall:")
        self.fall_label2.setBuddy(self.fall_spinbox2)

        # Same settings across both stimulators checkbox
        self.sync_timer_rf2 = QTimer()
        self.sync_timer_rf2.timeout.connect(self.update_values_rf2)

        self.same_rf_checkbox2 = QCheckBox("Same as rise")
        self.same_rf_checkbox2.stateChanged.connect(self.toggle_rf_settings2)

        layout = QGridLayout()
        layout.addWidget(self.rise_label2,                        0,0,1,1)
        layout.addWidget(self.rise_spinbox2,                      0,1,1,2)
        layout.addWidget(self.fall_label2,                        1,0,1,1)
        layout.addWidget(self.fall_spinbox2,                      1,1,1,2)
        layout.addWidget(self.same_rf_checkbox2,                  2,1,1,3)
        self.rise_fall_settings2.setLayout(layout)

        # Symmetric 1 and 2
        self.symmetric_checkbox = QCheckBox("Symmetric")
        self.symmetric_checkbox.stateChanged.connect(self.toggle_symmetric)

        # Asymmetric 1 and 2
        self.asymmetric_spinbox = QSpinBox()
        self.asymmetric_spinbox.setValue(1)
        self.asymmetric_spinbox.setRange(1,200)
        self.asymmetric_spinbox.setSuffix(' breaths each')
        self.asymmetric_spinbox.setEnabled(False)
        self.asymmetric_spinbox.objectName = 'PlaceHolder'

        # If 2 is off, grey out the boxes. If 2 is on, enable them.
        self.OnOff2.stateChanged.connect(lambda state: self.current_high_spinbox2.setEnabled(state == Qt.Checked))
        self.OnOff2.stateChanged.connect(lambda state: self.current_low_spinbox2.setEnabled(state == Qt.Checked))
        self.OnOff2.stateChanged.connect(lambda state: self.asymmetric_spinbox.setEnabled(state == Qt.Checked))
        self.OnOff2.stateChanged.connect(lambda state: self.peak_current_spinbox2.setEnabled(state == Qt.Checked))
        
        # If symmetric chebox is checked, disable asymmetric spinbox. If symmetric checkbox is unchecked, enable asymmetric spinbox.
        self.symmetric_checkbox.stateChanged.connect(lambda state: self.asymmetric_spinbox.setEnabled(state != Qt.Checked))

        # If same settings checkbox is checked, disable all entries for stim2. If unchecked, enable.
        self.same_settings_checkbox.stateChanged.connect(lambda state: self.current_high_spinbox2.setEnabled(state != Qt.Checked))
        self.same_settings_checkbox.stateChanged.connect(lambda state: self.current_low_spinbox2.setEnabled(state != Qt.Checked))
        self.same_settings_checkbox.stateChanged.connect(lambda state: self.rise_spinbox2.setEnabled(state != Qt.Checked))
        self.same_settings_checkbox.stateChanged.connect(lambda state: self.fall_spinbox2.setEnabled(state != Qt.Checked))
        self.same_settings_checkbox.stateChanged.connect(lambda state: self.peak_current_spinbox2.setEnabled(state != Qt.Checked))

        # Test burst for stim 2

        def update_elapsed_time2():
            self.time_passed2 = self.time_passed2.addMSecs(500)
        
        self.test_burst_flag2 = False
        self.time_passed2 = QTime(0,0)
        self.burst_timer2 = QTimer(self)
        self.burst_timer2.timeout.connect(update_elapsed_time2)
        self.burst_timer2.start(500)  # Update elapsed time every 500 ms

        def start_test_burst2():
            if self.test_burst_flag:
                self.parameters.thread_that_writes.trigger_send_to_serial(5)
            else:
                self.parameters.thread_that_writes.trigger_send_to_serial(3)
            self.test_burst_button2.setStyleSheet("background-color: green; color: white; padding: 15px; font-size: 24px;")
            self.test_burst_button2.setEnabled(False)
            self.test_burst_flag2 = True
            self.start_button.setEnabled(False)
            self.time_passed2 = QTime(0,0)
            # self.elapsed_time = self.elapsed_time.addSecs(1)
        
        def stop_test_burst2():
            if self.parameters.switchstate == '1' and self.parameters.triggering_type != "Synchronous" and self.parameters.triggering_type != "Predictive" and self.start_button.text() == "Start":
                self.test_burst_button2.setEnabled(True)
                self.burst_mode_alert2.setText("")
                self.burst_mode_alert2.setStyleSheet("color: white")

            else:
                self.test_burst_button2.setEnabled(False)
            
            if self.parameters.burst_flag_ext2 == 1 and not self.test_burst_flag2 and self.parameters.triggering_type != "Synchronous" and self.parameters.triggering_type != "Predictive":
                start_test_burst2()
                self.test_burst_flag2 = True
                # time.sleep(50)
            
            # if self.parameters.triggering_type == "Synchronous" or self.parameters.triggering_type == "Predictive":
            #     self.test_burst_button2.setEnabled(False)
            #     self.burst_mode_alert2.setText("SWITCH TO MANDATORY TO ENABLE")
            #     self.burst_mode_alert2.setStyleSheet("color: red")
            
            # else:
            #     self.burst_mode_alert2.setText("")
            #     self.burst_mode_alert2.setStyleSheet("color: white")
                        
            if self.time_passed2.msecsSinceStartOfDay() > 500 and self.parameters.burst_flag2 != 1 and self.test_burst_flag2:#self.parameters.burst_flag_ext == 0
                self.test_burst_button2.setStyleSheet("border: 1px solid #404040; background-color: solid gray; color: white; padding: 15px; font-size: 24px;")
                self.start_button.setEnabled(True)
                self.test_burst_flag2 = False
                self.test_burst_button2.setEnabled(True)

        self.test_burst_button2 = QPushButton("Test Burst")
        self.test_burst_button2.setFocusPolicy(Qt.NoFocus)
        self.test_burst_button2.setStyleSheet("border: 1px solid #404040; background-color: solid gray; color: white; padding: 15px; font-size: 24px;")
        self.test_burst_button2.setEnabled(True)
        self.test_burst_button2.clicked.connect(start_test_burst2)

        self.test_burst_timer2 = QTimer(self)
        self.test_burst_timer2.timeout.connect(stop_test_burst2)
        self.test_burst_timer2.start(20)

        self.burst_mode_alert2 = QLabel("")

        #### End code for stim 1 test burst

        self.current_settings2.setStyleSheet("QGroupBox { border: 1px solid #404040; }")

        layout = QGridLayout()
        layout.addWidget(self.OnOff2,                             0,0,1,1)
        layout.addWidget(self.same_settings_checkbox,             0,1,1,1)
        layout.addWidget(self.symmetric_checkbox,                 0,2,1,1)
        layout.addWidget(self.asymmetric_spinbox,                 0,3,1,1)
        layout.addWidget(self.peak_current_label2,                1,0,1,1)
        layout.addWidget(self.peak_current_spinbox2,              1,1,1,1)
        layout.addWidget(self.current_settings2,                  4,0,2,4)
        layout.addWidget(self.test_burst_button2,                 6,1,1,2)
        layout.addWidget(self.burst_mode_alert2,                  7,1,1,2, alignment=Qt.AlignCenter)
        self.GUI_stim2_settings.setLayout(layout)
        # layout.setContentsMargins(0, 0, 0, 0)

    # _____________________________________________________________________

    def predictive_bounds(self):
        if (self.parameters.triggering_type == "Synchronous" or self.parameters.triggering_type == "Synchronous Triggered") and self.parameters.I_trigger and self.parameters.values_esp[2] != 0:
            self.predictive_delay_spinbox.setRange(0, math.floor((self.parameters.values_esp[2] * 1000 - 200) / 50) * 50)
        elif (self.parameters.triggering_type == "Predictive" or self.parameters.triggering_type == "Predictive Triggered") and self.parameters.values_esp[2] != 0:
            self.predictive_delay_spinbox.setRange(-1000, math.floor((self.parameters.values_esp[2] * 1000 - 200) / 50) * 50)
        elif (self.parameters.triggering_type == "Synchronous" or self.parameters.triggering_type == "Synchronous Triggered") and self.parameters.E_trigger and self.parameters.last_e_time != 0:
            self.predictive_delay_spinbox.setRange(0, math.floor((self.parameters.last_e_time * 1000 - 200) / 50) * 50)
    
    def bias_change(self, state):
        if self.current_bias_checkbox.isChecked():
            self.parameters.bias1 = True
            self.parameters.bias_num = self.voltage_low_calc
            # self.parameters.thread_that_writes.trigger_send_to_serial(0)
        else:
            self.parameters.bias1 = False
            self.parameters.bias_num = 0
            # self.parameters.thread_that_writes.trigger_send_to_serial(0)
        
        if self.parameters.current_text == "Start": # or not self.OnOff.isChecked()
            self.parameters.thread_that_writes.trigger_send_to_serial(0)
    
    def bias_change_num(self):
        if self.current_bias_checkbox.isChecked():
            self.parameters.bias_num = self.voltage_low_calc
            if self.parameters.current_text == "Start": # or not self.OnOff.isChecked()
                self.parameters.thread_that_writes.trigger_send_to_serial(0)
    
    def bias_change2(self, state):
        if self.current_bias_checkbox2.isChecked():
            self.parameters.bias2 = True
            self.parameters.bias_num2 = self.voltage_low_calc2
            # self.parameters.thread_that_writes.trigger_send_to_serial(0)
        else:
            self.parameters.bias2 = False
            self.parameters.bias_num2 = 0
            # self.parameters.thread_that_writes.trigger_send_to_serial(0)
        
        if self.parameters.current_text == "Start": # or not self.OnOff2.isChecked()
            self.parameters.thread_that_writes.trigger_send_to_serial(0)
    
    def bias_change_num2(self):
        if self.current_bias_checkbox2.isChecked():
            self.parameters.bias_num2 = self.voltage_low_calc2
            if self.parameters.current_text == "Start": # or not self.OnOff2.isChecked()
                self.parameters.thread_that_writes.trigger_send_to_serial(0)
    
    def IEthreshold_change(self):
        if self.parameters.current_text == "Start":
            self.parameters.i_threshold = self.I_threshold.value()
            self.parameters.e_threshold = self.E_threshold.value()
            self.parameters.thread_that_writes.trigger_send_to_serial(0)
    
    def update_resp_I(self):
        self.resp_rate_value.setText("RR: {:.0f} bpm".format(self.parameters.values_esp[8]))
        self.I_time_value.setText("I time: {:.2f} s".format(self.parameters.last_i_time))
    
    def update_values_tab(self):
        self.value1.setText(str(self.parameters.values_esp[0]))
        self.value2.setText(str(self.parameters.values_esp[1]))
        self.value3.setText(str(self.parameters.values_esp[2]))
        if float(self.parameters.values_esp[3]) < 1:
            try:
                self.value4.setText("1: " + str(round(1 / float(self.parameters.values_esp[3]), 2)))
            except ZeroDivisionError:
                self.value4.setText("1")
        else:
            self.value4.setText(str(round(float(self.parameters.values_esp[3]), 2)) + ": 1")
        self.value5.setText(str(self.parameters.values_esp[4]))
        self.value6.setText(str(self.parameters.values_esp[5]))
        self.value7.setText(str(self.parameters.values_esp[6]))
        self.value8.setText(str(self.parameters.values_esp[7]))
        self.value9.setText(str(self.parameters.values_esp[8]))
        self.value10.setText(str(self.parameters.values_esp[9]))
        self.value11.setText(str(self.parameters.values_esp[10]))
        self.value12.setText(str(self.parameters.values_esp[11]))
    
    def toggle_rise_fall_settings(self):
        # Get the selected wave type
        selected_wave_type = self.wave_type.currentText()

        hfov_sin = (selected_wave_type == "HFOV" or selected_wave_type == "Sine")
        self.stim_rise_spinbox.setReadOnly(hfov_sin)
        self.stim_fall_spinbox.setReadOnly(hfov_sin)
        self.stim_rise_spinbox.setEnabled(not hfov_sin)
        self.stim_fall_spinbox.setEnabled(not hfov_sin)

        need_freq = selected_wave_type == "HFOV"
        self.hfov_freq_spinbox.setEnabled(need_freq)
        self.hfov_base_spinbox.setEnabled(need_freq)

        need_bilevel = selected_wave_type == "Bilevel"
        self.bilevel_t_spinbox.setEnabled(need_bilevel)
        self.bilevel_v_spinbox.setEnabled(need_bilevel)

    def toggle_radio_button(self):
        if self.radio_button3.isChecked():
            self.predictive_delay_spinbox.setValue(700)
        else:
            self.predictive_delay_spinbox.setValue(0)
    
    def toggle_trigger_settings(self):
        selected_triggering = self.triggering_combobox.currentText()

        need_i_time = (selected_triggering == "Synchronous" or selected_triggering == "Predictive")
        self.I_time.setReadOnly(need_i_time)
        self.I_time.setEnabled(not need_i_time)
        self.resp_rate_spinbox.setReadOnly(not need_i_time)

        need_resp = (selected_triggering == "Mandatory")
        self.resp_rate_spinbox.setReadOnly(not need_resp)
        self.resp_rate_spinbox.setEnabled(need_resp)
        
        need_delay = (selected_triggering == "Mandatory")
        self.predictive_delay_spinbox.setEnabled(not need_delay)

        only_trap = (selected_triggering == "Synchronous" or selected_triggering == "Predictive")
        not_selected = [1,2,3,4]
        if only_trap:
            for i in not_selected:
                self.wave_type.model().item(i).setEnabled(False)
            self.wave_type.setCurrentIndex(0)
            self.parameters.wave_type == "Trapezoid"
            self.wave_type.currentIndexChanged.connect(self.checkItime)
        else:
            for i in not_selected:
                self.wave_type.model().item(i).setEnabled(True)

        radio_buttons = (selected_triggering == "Synchronous" or selected_triggering == "Synchronous Triggered")
        self.radio_button1.setEnabled(radio_buttons)
        self.radio_button2.setEnabled(radio_buttons)
        # self.radio_button1.setChecked(radio_buttons)
        self.radio_button3.setEnabled(selected_triggering == "Synchronous")

        set_checked = (selected_triggering != None)
        self.radio_button1.setChecked(set_checked)

        if selected_triggering == "Synchronous" or selected_triggering == "Synchronous Triggered":
            self.predictive_delay_spinbox.setRange(0,5000)
            # self.radio_button1.setChecked(True)
        elif selected_triggering == "Predictive Triggered" or selected_triggering == "Predictive":
            self.predictive_delay_spinbox.setRange(-1000,5000)

        if not self.radio_button3.isChecked():
            self.predictive_delay_spinbox.setValue(0)

        if selected_triggering == "Predictive Triggered" or selected_triggering == "Predictive":
            self.number_breaths_avg.setEnabled(True)
        else:
            self.number_breaths_avg.setEnabled(False)
        
        self.I_time.setValue(1.5)
        self.resp_rate_spinbox.setValue(18)

    # Function to toggle OnOff2 settings
    def toggle_onoff_settings(self, state):
        if state != Qt.Checked:
            # When the checkbox is checked, start the timer to update the values
            self.sync_timer.start(100)  # Update every 100 ms (0.1 second)

            # Automatically uncheck the same_settings and symmetric checkboxes
            self.same_settings_checkbox.setChecked(False)
            self.symmetric_checkbox.setChecked(False)

        else:
            # When the checkbox is unchecked, stop the timer
            self.sync_timer.stop()

    # Function to toggle same settings for both stimulators mode
    def toggle_same_settings(self, state):
        if state == Qt.Checked:
            self.sync_timer.start(100)
            self.OnOff2.setChecked(True)
        
        elif state != Qt.Checked:
            self.sync_timer.start(100) 

        else:
            self.sync_timer.stop()

    def toggle_symmetric(self, state):
        if state == Qt.Checked:
            self.sync_timer.start(100)

            self.OnOff2.setChecked(True)
            self.OnOff.setChecked(True)

        else:
            self.sync_timer.stop()
    
    # Function to update box2 values based on box1
    def update_values(self):
        self.current_high_spinbox2.setValue(self.current_high_spinbox.value())
        self.current_low_spinbox2.setValue(self.current_low_spinbox.value())
        self.rise_spinbox2.setValue(self.rise_spinbox.value())
        self.fall_spinbox2.setValue(self.fall_spinbox.value())
        self.peak_current_spinbox2.setValue(self.peak_current_spinbox.value())

    # Function to toggle same settings for both stimulators mode
    def toggle_rf_settings(self, state):
        if state == Qt.Checked:
            self.sync_timer_rf.start(100)

        else:
            self.sync_timer_rf.stop()

    def update_values_rf(self):
        self.fall_spinbox.setValue(self.rise_spinbox.value())  
    
    def update_elapsed_time(self):
        self.elapsed_time_label.setText(self.parameters.elapsed_time_label)
        self.number_cycles_label.setText(self.parameters.number_cycles_label)

    def toggle_rf_settings2(self, state):
        if state == Qt.Checked:
            self.sync_timer_rf2.start(100)

        else:
            self.sync_timer_rf2.stop()
    
    def update_values_rf2(self):
        self.fall_spinbox2.setValue(self.rise_spinbox2.value()) 
    
    def change_IE(self):
        self.sync_timer_IE.start(100)

    # def update_values_IE(self):
    #     if self.parameters.triggering_type == "Synchronous" and float(self.parameters.values_esp[3]) != 0:
    #         if float(self.parameters.values_esp[3]) < 1:
    #             self.I_ratio_spinbox.setValue(1)
    #             self.E_ratio_spinbox.setValue(round(1 / float(self.parameters.values_esp[3]), 2))
    #         else:
    #             self.E_ratio_spinbox.setValue(1)
    #             self.I_ratio_spinbox.setValue(round(float(self.parameters.values_esp[3]), 2))
    #         self.resp_rate_spinbox.setValue(int(self.parameters.values_esp[8]))

    #     else:
    #         self.i_time = self.I_time.value()
    #         self.e_time = (60 / self.resp_rate_spinbox.value()) - self.i_time
    
    def update_resp(self):
        if (self.parameters.triggering_type != "Mandatory") and float(self.parameters.values_esp[3]) != 0:
            self.resp_rate_spinbox.setValue(int(self.parameters.values_esp[8]))

    def update_HFOV(self):
        if self.wave_type.currentIndex() == 3:
            self.stim_fall_spinbox.setValue(0)
            self.stim_rise_spinbox.setValue(self.I_time.value() / self.hfov_freq_spinbox.value() - (self.stim_fall_spinbox.value())) # * self.hfov_freq_spinbox.value()
    
    def eventFilter(self, source, event):
        if event.type() == QEvent.KeyPress:
            key = event.key()
            if key in [Qt.Key_Up, Qt.Key_Down]:
                self.update_values_I_time()

        return super().eventFilter(source, event)
    
    def synch_update_values_I_time(self):
        if self.parameters.triggering_type == "Synchronous":
            if float(self.parameters.values_esp[3]) != 0:
                if self.parameters.I_trigger:
                    self.i_time = self.parameters.last_i_time - (self.parameters.predictive_delay/1000) + self.parameters.micro_fall_time
                    self.e_time = self.parameters.last_e_time + (self.parameters.predictive_delay/1000) - self.parameters.micro_fall_time
                elif self.parameters.E_trigger:
                    self.i_time = self.parameters.last_e_time - (self.parameters.predictive_delay/1000) + self.parameters.micro_fall_time
                    self.e_time = self.parameters.last_i_time + (self.parameters.predictive_delay/1000) - self.parameters.micro_fall_time
                elif self.parameters.E_trigger_I:
                    self.i_time = self.parameters.last_i_time + self.parameters.last_e_time - (self.parameters.predictive_delay/1000) - 0.01 + self.parameters.micro_fall_time
                    self.e_time = 0.01 + (self.parameters.predictive_delay/1000) - self.parameters.micro_fall_time
            
                if float(self.parameters.values_esp[3]) < 1:
                    try:
                        self.I_ratio_spinbox.setText("1.0: " + str(round(1 / float(self.parameters.values_esp[3]), 2)))
                    except ZeroDivisionError:
                        self.I_ratio_spinbox.setText("1.0: 1.0")
                else:
                    self.I_ratio_spinbox.setText(str(round(float(self.parameters.values_esp[3]), 2)) + ": 1")
            
            self.I_time.setValue(self.i_time)
            if self.i_time > 0 or self.e_time > 0:
                self.resp_rate_spinbox.setValue(60 / (self.e_time + self.i_time))
        
        if self.parameters.triggering_type == "Predictive":
            if float(self.parameters.values_esp[3]) != 0:
                self.i_time = self.parameters.last_i_time - (self.parameters.predictive_delay/1000) + self.parameters.micro_fall_time
                self.e_time = self.parameters.last_e_time + (self.parameters.predictive_delay/1000) - self.parameters.micro_fall_time
            
            self.I_time.setValue(self.i_time)
            if self.i_time > 0 or self.e_time > 0:
                self.resp_rate_spinbox.setValue(60 / (self.e_time + self.i_time))
        
        elif self.parameters.triggering_type == "Synchronous Triggered" or self.parameters.triggering_type == "Predictive Triggered":
            self.i_time = self.I_time.value()
            
            if float(self.parameters.values_esp[3]) != 0:
                self.e_time = self.parameters.last_i_time + self.parameters.last_e_time - self.I_time.value()

                if (self.i_time / self.e_time) >= 1:
                    self.I_ratio_spinbox.setText('{:.2}: 1.0'.format(self.i_time / self.e_time))
                else:
                    self.I_ratio_spinbox.setText('1.0: {:.2}'.format(self.e_time / self.i_time))

    
    def update_values_I_time(self):
        # sender = self.sender()
        if self.parameters.triggering_type == "Mandatory":
            self.i_time = self.I_time.value()
            self.e_time = (60 / self.resp_rate_spinbox.value()) - self.i_time

            if (self.i_time / self.e_time) >= 1:
                self.I_ratio_spinbox.setText('{:.2}: 1.0'.format(self.i_time / self.e_time))
            else:
                self.I_ratio_spinbox.setText('1.0: {:.2}'.format(self.e_time / self.i_time))
    
    def change_current(self):
        self.sync_timer_current.start(100)  # Update every 100 ms (0.1 second)

    def update_values_current(self):
        self.current_high_label.setText('High (% of {} mA):'.format(self.peak_current_spinbox.value()))
        self.current_low_label.setText('Low (% of {} mA):'.format(self.peak_current_spinbox.value()))

        self.current_low_calc = self.peak_current_spinbox.value()/100.0*self.current_low_spinbox.value()
        self.current_high_calc = self.peak_current_spinbox.value()/100.0*self.current_high_spinbox.value()
        self.voltage_high_calc = self.current_high_spinbox.value()/10.0
        self.voltage_low_calc = self.current_low_spinbox.value()/10.0

        self.current_low_calclabel.setText('{:.1f} mA, {:.1f} V'.format(self.current_low_calc, self.voltage_low_calc))
        self.current_high_calclabel.setText('{:.1f} mA, {:.1f} V'.format(self.current_high_calc, self.voltage_high_calc))

        # Same for second stimulator
        self.current_high_label2.setText('High (% of {} mA):'.format(self.peak_current_spinbox2.value()))
        self.current_low_label2.setText('Low (% of {} mA):'.format(self.peak_current_spinbox2.value()))
        
        self.current_low_calc2 = self.peak_current_spinbox2.value()/100*self.current_low_spinbox2.value()
        self.current_high_calc2 = self.peak_current_spinbox2.value()/100*self.current_high_spinbox2.value()
        self.voltage_high_calc2 = self.current_high_spinbox2.value()/10
        self.voltage_low_calc2 = self.current_low_spinbox2.value()/10

        self.current_low_calclabel2.setText('{:.1f} mA, {:.1f} V'.format(self.current_low_calc2, self.voltage_low_calc2))
        self.current_high_calclabel2.setText('{:.1f} mA, {:.1f} V'.format(self.current_high_calc2, self.voltage_high_calc2))

    def checkRatios(self, value):
        sender = self.sender()

        if sender == self.I_ratio_spinbox:
            self.E_ratio_spinbox.setValue(1 - value)
        else:
            self.I_ratio_spinbox.setValue(1 - value)
        self.update_parameters_from_gui

    def checkAmplitudes(self, value):
        sender = self.sender()

        if sender == self.current_low_spinbox:
            if self.current_low_spinbox.value() >= self.current_high_spinbox.value():
                self.current_high_spinbox.setValue(self.current_low_spinbox.value() + 1)
        elif sender == self.current_high_spinbox:
            if self.current_high_spinbox.value() <= self.current_low_spinbox.value():
                self.current_low_spinbox.setValue(self.current_high_spinbox.value() - 1)
        elif sender == self.current_low_spinbox2:
            if self.current_low_spinbox2.value() >= self.current_high_spinbox2.value():
                self.current_high_spinbox2.setValue(self.current_low_spinbox2.value() + 1)
        elif sender == self.current_high_spinbox2:
            if self.current_high_spinbox2.value() <= self.current_low_spinbox2.value():
                self.current_low_spinbox2.setValue(self.current_high_spinbox2.value() - 1)
    
    def checkItime(self, value):

        sender = self.sender()

        self.stim_rise_spinbox.setRange(0,self.parameters.i_time)
        self.stim_fall_spinbox.setRange(0,self.parameters.i_time)

        if self.parameters.wave_type == "Triangle":
            # self.I_time.setRange(self.stim_rise_spinbox.value() + self.stim_fall_spinbox.value(), 5)
            if sender == self.stim_fall_spinbox:
                self.stim_rise_spinbox.setValue(self.parameters.i_time - value)
            elif sender == self.stim_rise_spinbox:
                self.stim_fall_spinbox.setValue(self.parameters.i_time - value)
            else:
                self.stim_rise_spinbox.setValue(self.parameters.i_time - self.parameters.stim_fall)
        elif self.parameters.wave_type == "Trapezoid":
            max_allowed_value = self.parameters.i_time - value - 0.05
            self.I_time.setRange(self.stim_rise_spinbox.value() + self.stim_fall_spinbox.value() + 0.05, 5)
            if sender == self.stim_fall_spinbox:
                self.stim_rise_spinbox.setValue(min(self.stim_rise_spinbox.value(), max_allowed_value))
            elif sender == self.stim_rise_spinbox:
                self.stim_fall_spinbox.setValue(min(self.stim_fall_spinbox.value(), max_allowed_value))
            if self.parameters.triggering_type == "Synchronous":
                self.stim_rise_spinbox.setRange(0, self.parameters.i_time - self.parameters.stim_fall - 0.05)
                # self.stim_fall_spinbox.setRange(0, self.parameters.i_time - self.parameters.stim_fall - 0.05)
        elif self.parameters.wave_type == "Bilevel":
            max_allowed_rise = (self.parameters.i_time - value)/2 - 0.05
            max_allowed_fall = self.parameters.i_time - (value*2) - 0.05
            self.I_time.setRange(2 * self.stim_rise_spinbox.value() + self.stim_fall_spinbox.value() + 0.05, 5)
            if sender == self.stim_fall_spinbox:
                self.stim_rise_spinbox.setValue(min(self.stim_rise_spinbox.value(), max_allowed_rise))
            elif sender == self.stim_rise_spinbox:
                self.stim_fall_spinbox.setValue(min(self.stim_fall_spinbox.value(), max_allowed_fall))
            if self.parameters.triggering_type == "Synchronous":
                self.stim_rise_spinbox.setRange(0, (self.parameters.i_time - self.parameters.stim_fall)/2 - 0.05)

        self.update_parameters_from_gui
    

    def check_flowsense(self):
        mandatory_only = not self.parameters.flow_attached
        not_selected = [1,2,3,4]  # Indexes of the items other than the first one

        if mandatory_only:
            for i in not_selected:
                self.triggering_combobox.model().item(i).setEnabled(False)
            self.triggering_combobox.setCurrentIndex(0)
            self.parameters.triggering_type = "Mandatory"
            self.flowsense_connect_label.setText("CONNECT FLOW")
            self.flowsense_connect_label.setStyleSheet("color: red")
        else:
            for i in not_selected:
                self.triggering_combobox.model().item(i).setEnabled(True)
            self.flowsense_connect_label.setText("")
    
    def processSerialData(self):
        if (self.parameters.switchstate == '0' and not self.parameters.sentZero):
            self.parameters.sentZero = True
            self.parameters.sentOnce = False
            print("Box Off: " + str(datetime.datetime.now()))
            if self.parameters.switchstate == '0' and (self.parameters.micro_allow_stim1 != 0 or self.parameters.micro_allow_stim2 != 0):
                self.parameters.thread_that_writes.trigger_send_to_serial(0)
                logger.info("Box Off (stim stopped): " + str(datetime.datetime.now()))
            self.start_button.setStyleSheet("color: gray; padding: 10px; font-size: 16px;")
            self.start_button.setText('Start')
            # self.figure.pause_plot()
            self.start_button.setEnabled(False)
            self.clear_button.setEnabled(False)
            # self.com_connect.setText("Not connected")
            # self.com_connect.setStyleSheet("background-color: red; color: white;")
        
        elif self.parameters.switchstate == '1' and not self.parameters.sentOnce and self.parameters.read_thread is not None: # and self.parameters.pres_status != 0
            print("Box On: " + str(datetime.datetime.now()))
            self.start_button.setStyleSheet("background-color: green; color: white; padding: 10px; font-size: 16px;")
            self.start_button.setEnabled(True)
            self.clear_button.setEnabled(True)
            self.com_connect.setText("Connected")
            self.com_connect.setStyleSheet("background-color: green; color: white;")
            self.parameters.sentZero = False
            self.parameters.sentOnce = True


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Roboto", 10))

    qdarktheme.setup_theme()
    
    # Get the style sheet from QDarkTheme
    dark_stylesheet = qdarktheme.load_stylesheet("dark")
    
    dark_GUI_stylesheet = "QWidget { background-color: #1e1e1e; }"
    text_stylesheet = """
        QCheckbox, QGroupBox, QLabel, QComboBox, QSpinBox, QRadioButton, QDoubleSpinBox, QButton, QTabWidget {
            color: white;
        }
    """
    background_stylesheet = """
        QComboBox, QSpinBox, QDoubleSpinBox, QTabWidget, QCheckbox {
            background-color: #3a3a3a;
        }
    """
    
    # Add custom modifications to the style sheet
    custom_stylesheet = """
        /* Set button symbols */
        QSpinBox::down-button, QDoubleSpinBox::down-button {
            width: 1.5em;
            height: 1.5em;
            subcontrol-origin: margin;
            subcontrol-position: center left;
        }

        QSpinBox::up-button, QDoubleSpinBox::up-button {
            width: 1.5em;
            height: 1.5em;
            subcontrol-origin: margin;
            subcontrol-position: center right;
        }
    """

    
    # Merge the style sheets
    # app.setStyleSheet(dark_GUI_stylesheet + text_stylesheet +  custom_stylesheet + background_stylesheet) #  dark_stylesheet + 
    app.setStyleSheet(dark_stylesheet + custom_stylesheet) #  

    gallery = MainWindow()
    # gallery.setStyleSheet("background-color: #1e1e1e; color: white")
    
    gallery.setGeometry(app.desktop().availableGeometry())
    gallery.showMaximized()
    sys.exit(app.exec())