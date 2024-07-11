import numpy as np
import serial

"""
@param {int} COM_port : com port to connect to
@param {int} baudrate : baudrate for serial communication
"""
def micro_connect_serial(COM_port, baudrate): # baud rate 921600
    try: 
        micro_serial = serial.Serial(COM_port, baudrate, timeout=2,  writeTimeout = 0)
    except:
        micro_serial = None
    
    if micro_serial is not None:
        if micro_serial.is_open:
            print('Microcontroller connected')
    else:
        print('Microcontroller not connected!')  

    return micro_serial

"""
@param {list} data : list of int/float variables
@param {serial} MICRO : serial object for sending data to box
"""
def send_data(data, MICRO):
    MICRO.reset_output_buffer()
    for count, value in enumerate(data):
        temp_holder = str(round(value, 2)) + ","
        MICRO.write(bytes(temp_holder, 'utf-8'))
    MICRO.write(b"\n")

"""
@param {int} breaths_per_min : number of breaths per minute
@param {float} IE_ratio : inspiration time / expiration time
@param {int} frequency : frequency in Hz
"""
def pack_data_to_transfer(n_breaths, insp_period, exp_period, frequency, wave_type, rise_time, fall_time, allow_stim1, amplitude1, base1, allow_stim2, amplitude2, base2, trigger_type, predictive_delay, hfov_base, num_breaths_avg, i_threshold, e_threshold, bilevel_t, bilevel_v, bias1, bias2, zeroing):
    data = np.zeros(24, dtype=float)

    data[0] = n_breaths
    data[1] = insp_period
    data[2] = exp_period
    data[3] = frequency 
    data[4] = wave_type
    data[5] = rise_time
    data[6] = fall_time
    data[7] = allow_stim1
    data[8] = amplitude1
    data[9] = base1
    data[10] = allow_stim2
    data[11] = amplitude2
    data[12] = base2
    data[13] = trigger_type
    data[14] = predictive_delay
    data[15] = hfov_base
    data[16] = num_breaths_avg
    data[17] = i_threshold
    data[18] = e_threshold
    data[19] = bilevel_t
    data[20] = bilevel_v
    data[21] = bias1
    data[22] = bias2
    data[23] = zeroing

    return data.tolist()