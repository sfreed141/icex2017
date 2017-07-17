from jsf_types import *

def extract_from_jsf(jsf_file, dtype):
    data = np.fromfile(jsf_file, dtype=dtype, count=1)
    if len(data) == 0:
        return None
    else:
        return data[0]

def extract_jsf_header(jsf_file):
    return extract_from_jsf(jsf_file, JSF_HEADER_TYPE)

def extract_jsf_sonar(jsf_file):
    return extract_from_jsf(jsf_file, JSF_SONAR_MESSAGE_TYPE)

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
    msg = extract_from_jsf(jsf_file, JSF_BATHYMETRIC_DATA_MESSAGE_TYPE)
    samples = np.fromfile(jsf_file, dtype=JSF_BATHYMETRIC_SAMPLE_TYPE, count=msg['num_samples'])

    return msg, samples

def parse_jsf_message(header, jsf_file):
    if header['message_type'] == MESSAGE_TYPE['sonar']:
        return parse_sonar(header, jsf_file)
    elif header['message_type'] == MESSAGE_TYPE['bathymetric_data']:
        return parse_bathymetric_data(header, jsf_file)
    elif header['message_type'] == MESSAGE_TYPE['bathy_pressure']:
        msg = extract_from_jsf(jsf_file, JSF_PRESSURE_MESSAGE_TYPE)
        return msg
    elif header['message_type'] == MESSAGE_TYPE['bathy_altitude']:
        msg = extract_from_jsf(jsf_file, JSF_ALTITUDE_MESSAGE_TYPE)
        return msg
    elif header['message_type'] == MESSAGE_TYPE['bathy_position']:
        msg = extract_from_jsf(jsf_file, JSF_POSITION_MESSAGE_TYPE)
        return msg
    elif header['message_type'] == MESSAGE_TYPE['bathy_status']:
        msg = extract_from_jsf(jsf_file, JSF_STATUS_MESSAGE_TYPE)
        return msg
 
    return None

def read_jsf(jsf_filename):
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

def read_log(log_filename):
    log_dtype = np.dtype([
        ("latitude", np.float32),
        ("longitude", np.float32),
        ("time", 'U32'),
        ("date", 'U32'),
#        ("hour", np.uint8),
#        ("minute", np.uint8),
#        ("second", np.uint8),
#        ("month", np.uint8),
#        ("day", np.uint8),
#        ("year", np.uint8),
        ("number_of_sats", np.uint8),
        ("gps_speed", np.float32),
        ("gps_true_heading", np.float32),
        ("gps_magnetic_variation", np.float32),
        ("hdop", np.float32),
        ("c_magnetic_heading", np.float32),
        ("c_true_heading", np.float32),
        ("pitch_angle", np.float32),
        ("roll_angle", np.float32),
        ("c_inside_temp", np.float32),
        ("dfs_depth", np.float32),
        ("dtb_height", np.float32),
        ("total_water_column", np.float32),
        ("batt_percent", np.float32),
        ("power_watts", np.float32),
        ("watt_hours", np.float32),
        ("batt_volts", np.float32),
        ("batt_ampers", np.float32),
        ("batt_state", np.character),
        ("time_to_empty", np.float32),
        ("current_step", np.uint32),
        ("dist_to_next", np.float32),
        ("next_speed", np.float32),
        ("vehicle_speed", np.float32),
        ("motor_speed", np.float32),
        ("next_heading", np.float32),
        ("next_long", np.float32),
        ("next_lat", np.float32),
        ("next_depth", np.float32),
        ("depth_goal", np.float32),
        ("vehicle_state", np.int8),
        ("error_state", np.character),
        ("distance_to_track", np.float32),
        ("fin_pitch_r", np.float32),
        ("fin_pitch_l", np.float32),
        ("pitch_goal", np.float32),
        ("fin_yaw_t", np.float32),
        ("fin_yaw_b", np.float32),
        ("yaw_goal", np.float32),
        ("fin_roll", np.float32),
        ("dvl_x_speed", np.float32),
        ("dvl_y_speed", np.float32),
        ("dvl_temperature", np.float32),
        ("conductivity", np.float32),
        ("ctd_temperature", np.float32),
        ("salinity", np.float32),
        ("sound_speed", np.float32),
        ("blank", np.character)
    ])

    data = np.loadtxt(log_filename, dtype=log_dtype, delimiter=';', skiprows=1)

    return data
