#! /usr/bin/python

import sys, os
import numpy as np
from scipy.signal import medfilt
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
from mpl_toolkits.mplot3d import Axes3D
import pyproj
import cv2

jsf_header_type = np.dtype([
    ("start_of_header", np.uint16),
    ("version", np.uint8),
    ("session_id", np.uint8),
    ("message_type", np.uint16),
    ("command_type", np.uint8),
    ("subsystem", np.uint8),
    ("channel", np.uint8),
    ("sequence_num", np.uint8),
    ("reserved", np.uint16),
    ("size", np.uint32)
])

jsf_sonar_msg_type = np.dtype([
    ("time", np.int32),
    ("start_depth", np.uint32),
    ("ping_num", np.uint32),
    ("r1", np.int16, 2),
    ("msbs", np.uint16),
    ("lsb", np.uint16),
    ("lsb2", np.uint16),
    ("r2", np.int16, 3),
    ("id_code", np.int16),
    ("validity", np.uint16),
    ("r3", np.uint16),
    ("data_format", np.int16),
    ("distance_aft", np.int16),
    ("distance_starboard", np.int16),
    ("r4", np.int16, 2),
    ("km_of_pipe", np.float32),
    ("r5", np.int16, 16),
    ("longitude", np.int32),
    ("latitude", np.int32),
    ("coordinate_units", np.int16),
    ("annotation_string", np.uint8, 24),
    ("samples", np.uint16),
    ("sampling_interval", np.uint32),
    ("adc_gain", np.uint16),
    ("user_transmit_level_setting", np.int16),
    ("r6", np.int16),
    ("transmit_pulse_start_freq", np.uint16),
    ("transmit_pulse_end_freq", np.uint16),
    ("sweep_length", np.uint16),
    ("pressure", np.int32),
    ("depth", np.int32),
    ("sample_freq", np.uint16),
    ("outgoing_pulse_id", np.uint16),
    ("altitude", np.int32),
    ("sound_speed", np.float32),
    ("mixer_freq", np.float32),
    ("year", np.int16),
    ("day", np.int16),
    ("hour", np.int16),
    ("minute", np.int16),
    ("second", np.int16),
    ("time_basis", np.int16),
    ("weighting_factor", np.int16),
    ("num_pulses", np.int16),
    ("compass_heading", np.uint16),
    ("pitch", np.int16),
    ("roll", np.int16),
    ("r7", np.int16, 2),
    ("trigger_source", np.int16),
    ("mark_number", np.uint16),
    ("position_fix_hour", np.int16),
    ("position_fix_minutes", np.int16),
    ("position_fix_seconds", np.int16),
    ("course_in_degrees", np.int16),
    ("speed", np.int16),
    ("position_fix_day", np.int16),
    ("position_fix_year", np.int16),
    ("milliseconds_today", np.uint32),
    ("max_adc_value", np.uint16),
    ("r8", np.int16, 2),
    ("sonar_software_version", np.int8, 6),
    ("spherical_correction_factor", np.int32),
    ("packet_number", np.uint16),
    ("adc_decimation", np.int16),
    ("r9", np.int16),
    ("water_temp", np.int16),
    ("layback", np.float32),
    ("r10", np.int32),
    ("cable_out", np.uint16),
    ("r11", np.uint16),
])

jsf_BathymetricDataMessageType = np.dtype([
    ('time', np.uint32),
    ('nanoseconds', np.uint32),
    ('ping_num', np.uint32),
    ('num_samples', np.uint16),
    ('channel', np.uint8),
    ('algorithm_type', np.uint8),
    ('num_pulses', np.uint8),
    ('pulse_phase', np.uint8),
    ('pulse_length', np.uint16),
    ('transmit_pulse_amplitude', np.float32),
    ('chirp_start_frequency', np.float32),
    ('chirp_end_frequency', np.float32),
    ('mixer_frequency', np.float32),
    ('sample_rate', np.float32),
    ('offset_to_first_sample', np.uint32),
    ('time_delay_uncertainty', np.float32),
    ('time_scale_factor', np.float32),
    ('time_scale_accuracy', np.float32),
    ('angle_scale_factor', np.float32), # documentation says uint32 but i think its a float
    ('reserved1', np.uint32),
    ('time_to_first_bottom_return', np.uint32),
    ('format_revision_level', np.uint8),
    ('binning_flag', np.uint8),
    ('tvg', np.uint8),
    ('reserved2', np.uint8),
    ('span', np.float32),
    ('reserved3', np.uint32)
])

jsf_BathymetricSampleType = np.dtype([
    ('time_delay', np.uint16),
    ('angle', np.int16),
    ('amplitude', np.uint8),
    ('angle_uncertainty', np.uint8),
    ('flag', np.uint8),
    ('snr_and_quality', np.uint8)
])

jsf_AttitudeMessageType = np.dtype([
    ('time', np.uint32),
    ('nanoseconds', np.uint32),
    ('data_valid_flag', np.uint32),
    ('heading', np.float32),
    ('heave', np.float32),
    ('pitch', np.float32),
    ('roll', np.float32),
    ('yaw', np.float32)
])

jsf_PressureMessageType = np.dtype([
    ('time', np.uint32),
    ('nanoseconds', np.uint32),
    ('data_valid_flag', np.uint32),
    ('absolute_pressure', np.float32),
    ('water_temperature', np.float32),
    ('salinity', np.float32),
    ('conductivity', np.float32),
    ('sound_velocity', np.float32),
    ('depth', np.float32)
])

jsf_AltitudeMessageType = np.dtype([
    ('time', np.uint32),
    ('nanoseconds', np.uint32),
    ('data_valid_flag', np.uint32),
    ('altitude', np.float32),
    ('speed', np.float32),
    ('heading', np.float32)
])

jsf_PositionMessageType = np.dtype([
    ('time', np.uint32),
    ('nanoseconds', np.uint32),
    ('data_valid_flag', np.uint16),
    ('utm_zone', np.uint16),
    ('easting', np.float64),
    ('northing', np.float64),
    ('latitude', np.float64),
    ('longitude', np.float64),
    ('speed', np.float32),
    ('heading', np.float32),
    ('antenna_height', np.float32)
])

jsf_StatusMessageType = np.dtype([
    ('time', np.uint32),
    ('nanoseconds', np.uint32),
    ('data_valid_flag', np.uint16),
    ('version', np.uint8),
    ('gga_status', np.uint8),
    ('ggk_status', np.uint8),
    ('number_of_satellites', np.uint8),
    ('reserved1', np.uint8),
    ('dilution_of_precision', np.float32),
    ('reserved2', np.uint32)
])

MESSAGE_TYPE = {
    "sonar": 80,
    "side_scan": 82,
    "nmea": 2002,
    "pitch_roll": 2020,
    "pressure": 2060,
    "dvl": 2080,
    "situation": 2090,
    "situation_comprehensive": 2091,
    "cable_counter_data": 2100,
    "kilometer_of_pipe_data": 2101,
    "container_timestamp": 2111,
    "bathymetric_data": 3000,
    "bathy_attitude": 3001,
    "bathy_pressure": 3002,
    "bathy_altitude": 3003,
    "bathy_position": 3004,
    "bathy_status": 3005
}

def extract_from_jsf(jsf_file, dtype):
    data = np.fromfile(jsf_file, dtype=dtype, count=1)
    if (len(data) == 0):
        return None
    else:
        return data[0]

def extract_jsf_header(jsf_file):
    return extract_from_jsf(jsf_file, jsf_header_type)

def extract_jsf_sonar(jsf_file):
    return extract_from_jsf(jsf_file, jsf_sonar_msg_type)

def extract_jsf_nmea(jsf_file, header):
    jsf_nmea = np.dtype([
        ("time", np.int32),
        ("milliseconds", np.int32),
        ("source", np.uint8),
        ("reserved", np.uint8, 3),
        ("string_data", 'S', header['size'] - 12)
    ])
    data = extract_from_jsf(jsf_file, jsf_nmea)
    return data

def parse_sonar(header, jsf_file):
    msg = extract_jsf_sonar(jsf_file)

    if msg['data_format'] != 0:
        print('bad')
    else:
        if header['size'] - 240 != msg['samples'] * 2:
            print("data size doesn't match sample number")
        data = np.fromfile(jsf_file, dtype=np.uint16, count=msg['samples'])

    return msg, data

def parse_bathymetric_data(header, jsf_file):
    msg = extract_from_jsf(jsf_file, jsf_BathymetricDataMessageType)
    samples = np.fromfile(jsf_file, dtype=jsf_BathymetricSampleType, count=msg['num_samples'])

    return msg, samples

def parse_jsf_message(header, jsf_file):
    if header['message_type'] == MESSAGE_TYPE['sonar']:
        return parse_sonar(header, jsf_file)
    elif header['message_type'] == MESSAGE_TYPE['bathymetric_data']:
        return parse_bathymetric_data(header, jsf_file)
    elif header['message_type'] == MESSAGE_TYPE['bathy_pressure']:
        msg = extract_from_jsf(jsf_file, jsf_PressureMessageType)
        return msg
    elif header['message_type'] == MESSAGE_TYPE['bathy_altitude']:
        msg = extract_from_jsf(jsf_file, jsf_AltitudeMessageType)
        return msg
    elif header['message_type'] == MESSAGE_TYPE['bathy_position']:
        msg = extract_from_jsf(jsf_file, jsf_PositionMessageType)
        return msg
    elif header['message_type'] == MESSAGE_TYPE['bathy_status']:
        msg = extract_from_jsf(jsf_file, jsf_StatusMessageType)
        return msg
    
    return None

def find_msg_types(jsf_filename):
    msg_types = set([])
    with open(jsf_filename, 'rb') as jsf_file:
        while True:
            header = extract_jsf_header(jsf_file)
            if header == None:
                break

            msg_types.add(header['message_type'])

            jsf_file.seek(header['size'], 1)

    return msg_types
    
def read_jsf(jsf_filename):
#    sonar_msgs = []
#    sonar_data = []
#    bathy_msgs = []
#    bathy_data = []
    msgs = []
    with open(jsf_filename, 'rb') as jsf_file:
        while True:
            header = extract_jsf_header(jsf_file)
            if header == None:
                break

            if header['start_of_header'] != 0x1601:
                print("Invalid start of message")
                break

            msg_type = header['message_type']

            jsf_message = parse_jsf_message(header, jsf_file)
            if jsf_message is None:
                jsf_file.seek(header['size'], 1)
            else:
                msgs.append((header, jsf_message))
    return msgs
#                if msg_type == MESSAGE_TYPE['sonar']:
#                    sonar_msgs.append(jsf_message[0])
#                    sonar_data.append(jsf_message[1])
#                elif msg_type == MESSAGE_TYPE['bathymetric_data']:
#                    bathy_msgs.append(jsf_message[0])
#                    bathy_data.append(jsf_message[1])

    #return sonar_msgs, sonar_data, bathy_msgs, bathy_data

def process_sonar_data(sonar_msgs, sonar_data):
    if len(sonar_msgs) == 0:
        print('No sonar messages to process')
        return

    sonar_msg = sonar_msgs[0]
    num_samples = sonar_msg['samples']

    sonar_d = sonar_data[0]
# https://stackoverflow.com/questions/13728392/moving-average-or-running-mean/27681394#27681394
    N = 10
    cumsum = np.cumsum(np.insert(sonar_d, 0, 0))
    smoothed_data = (cumsum[N:] - cumsum[:-N]) / N

    plt.figure()

    plt.subplot(211)
    plt.plot(sonar_d, 'r', linewidth=0.5)

    plt.title('Echo returns vs sample number (raw)')
    plt.xlabel('Sample')
    plt.ylabel('Echo Return Strength')

    plt.subplot(212)
    plt.plot(smoothed_data, 'r', linewidth=0.5)

    plt.title('Echo returns vs sample number (smoothed)')
    plt.xlabel('Sample')
    plt.ylabel('Echo Return Strength')

    #plt.show()
    print(len(sonar_msgs), len(sonar_data))
    print(sonar_data[0].shape)
    print(sonar_data[0].dtype)

    scanlines = np.array([np.concatenate([np.flip(port_data, 0), starboard_data]) for port_data,starboard_data in zip(sonar_data[::2], sonar_data[1::2])])
    #scanlines = scanlines / np.amax(scanlines)
    #scanlines = np.clip(scanlines, 0, 1)
    scanlines *= 255
    #scanlines = np.empty((len(sonar_data) // 2, len(sonar_data[0]) * 2), dtype=np.uint16)
    #for scanline, port_data, starboard_data in zip(scanlines, sonar_data[::2], sonar_data[1::2]):
        #scanline = np.concatenate([port_data, starboard_data])
        #port_data = np.flip(port_data, 1)
        #scanlines.append(np.concatenate([port_data, starboard_data]))
        #scanlines.append(np.stack([port_data, starboard_data]))
        
    print('here')
    print(scanlines.shape)
    cv2.imwrite('image.png', cv2.resize(scanlines, (0, 0), fx=0.2, fy=0.2))
    cv2.waitKey()

def process_bathy_data(bathy_msgs, bathy_data):
#    print(bathy_msgs[0].dtype.names)
#    print(bathy_data[0].dtype.names)
#    print(bathy_msgs[0])
#    print(bathy_data[0])

    sound_velocity = 1532

    msg = bathy_msgs[0]
    data = bathy_data[0]

#    for field in msg.dtype.names:
#        print(field, msg[field])

    xs = []
    ys = []
    zs = []

    y = 0
    for msg, data in zip(bathy_msgs, bathy_data):
        for sample in data:
            if sample['flag'] & 0x20:
                continue

            echo_time = msg['offset_to_first_sample'] * 1e-9 + sample['time_delay'] * msg['time_scale_factor']
            slant_range = sound_velocity * echo_time / 2
            angle_from_nadir = (-1)**(msg['channel'] + 1) * sample['angle'] * msg['angle_scale_factor']
            seafloor_x = slant_range * np.sin(np.radians(angle_from_nadir))
            seafloor_z = slant_range * np.cos(np.radians(angle_from_nadir))

            xs.append(seafloor_x)
            ys.append(y)
            zs.append(-seafloor_z)
        y += 1

#        for field in sample.dtype.names:
#            print(field, sample[field])
#        print('time delay {}\nangle {}\namplitude {}\nflag {:0x}\nsnr {}\nquality {}\n'.format(sample['time_delay'] * msg['time_scale_factor'], sample['angle'] * msg['angle_scale_factor'], sample['amplitude'], sample['flag'], sample['snr_and_quality'] >> 3, sample['snr_and_quality'] & 0x7))
#        print()

        #print(echo_time, slant_range, angle_from_nadir, seafloor_x, seafloor_z)

    plt.subplot(111, projection='3d')
    plt.plot(xs, ys, zs, 'bo', markersize=0.1)
    plt.title('Bathymetry Data')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()

def rotation_x(angle):
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

def rotation_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])

def rotation_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

def process_msgs(jsf_msgs):
    # State variables to be updated from jsf messages
    sound_velocity = 1532
    depth = 0

    altitude = 0

    heave = 0
    roll = 0
    pitch = 0

    latitude = 0
    longitude = 0
    speed = 0
    heading = 0

    lastTime = 0
    lastNanoseconds = 0
    deltaTime = 0
    deltaNanoseconds = 0

    xs = []
    ys = []
    zs = []

    y = 0
    for msg_hdr, msg in jsf_msgs:
        # Update state variables
        if msg_hdr['message_type'] == MESSAGE_TYPE['bathy_attitude']:
            if msg['data_valid_flag'] & 0x01:
                heading = msg['heading']
            if msg['data_valid_flag'] & 0x02:
                heave = msg['heave']
            if msg['data_valid_flag'] & 0x04:
                pitch = msg['pitch']
            if msg['data_valid_flag'] & 0x08:
                roll = msg['roll']
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathy_pressure']:
            if msg['data_valid_flag'] & 0x10:
                sound_velocity = msg['sound_velocity']
            if msg['data_valid_flag'] & 0x20:
                depth = msg['depth']
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathy_altitude']:
            if msg['data_valid_flag'] & 0x1:
                altitude = msg['altitude']
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathy_position']:
            if msg['data_valid_flag'] & 0x08:
                latitude = msg['latitude']
            if msg['data_valid_flag'] & 0x10:
                longitude = msg['longitude']
            if msg['data_valid_flag'] & 0x20:
                speed = msg['speed']
            if msg['data_valid_flag'] & 0x40:
                heading = msg['heading']
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathy_status']:
            pass
        elif msg_hdr['message_type'] == MESSAGE_TYPE['bathymetric_data']:
            msg, data = msg

            if lastTime == 0:
                lastTime = msg['time']
                lastNanoseconds = msg['nanoseconds']
            else:
                currentTime = msg['time']
                currentNanoseconds = msg['nanoseconds']
                deltaTime = currentTime - lastTime
                if currentNanoseconds < lastNanoseconds:
                    deltaNanoseconds = lastNanoseconds - currentNanoseconds
                else:
                    deltaNanoseconds = currentNanoseconds - lastNanoseconds
                lastTime = currentTime
                lastNanoseconds = currentNanoseconds

            speed_mps = speed / 3600 * 1852
            y += deltaTime * speed_mps

            for sample in data:
                snr = sample['snr_and_quality'] >> 3
                quality = sample['snr_and_quality'] & 0x7
                if sample['flag'] & 0x20 or snr < 20 or quality == 0:
                    continue

                echo_time = msg['offset_to_first_sample'] * 1e-9 + sample['time_delay'] * msg['time_scale_factor']
                slant_range = sound_velocity * echo_time / 2
                angle_from_nadir = (-1)**(msg['channel'] + 1) * sample['angle'] * msg['angle_scale_factor']
                seafloor_x = slant_range * np.sin(np.radians(angle_from_nadir - roll))
                seafloor_z = slant_range * np.cos(np.radians(angle_from_nadir - roll))

                xs.append(seafloor_x)
                ys.append(y)
                zs.append(-seafloor_z)


    plt.subplot(111, projection='3d')
    plt.plot(xs, ys, zs, 'bo', markersize=0.1)
    plt.title('Bathymetry Data')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("usage: " + sys.argv[0] + " <jsf file>")
        sys.exit(0)

    jsf_filename = sys.argv[1]
    if not os.path.isfile(jsf_filename):
        print("error: file '" + jsf_filename + "' does not exist")
        sys.exit(0)

    jsf_msgs = read_jsf(jsf_filename)

    process_msgs(jsf_msgs)

    #process_sonar_data(jsf_msgs[0], jsf_msgs[1])
    #process_bathy_data(jsf_msgs[2], jsf_msgs[3])

if __name__ == '__main__':
    main()

