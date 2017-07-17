import numpy as np

JSF_HEADER_TYPE = np.dtype([
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

JSF_SONAR_MESSAGE_TYPE = np.dtype([
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

JSF_BATHYMETRIC_DATA_MESSAGE_TYPE = np.dtype([
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

JSF_BATHYMETRIC_SAMPLE_TYPE = np.dtype([
    ('time_delay', np.uint16),
    ('angle', np.int16),
    ('amplitude', np.uint8),
    ('angle_uncertainty', np.uint8),
    ('flag', np.uint8),
    ('snr_and_quality', np.uint8)
])

JSF_ATTITUDE_MESSAGE_TYPE = np.dtype([
    ('time', np.uint32),
    ('nanoseconds', np.uint32),
    ('data_valid_flag', np.uint32),
    ('heading', np.float32),
    ('heave', np.float32),
    ('pitch', np.float32),
    ('roll', np.float32),
    ('yaw', np.float32)
])

JSF_PRESSURE_MESSAGE_TYPE = np.dtype([
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

JSF_ALTITUDE_MESSAGE_TYPE = np.dtype([
    ('time', np.uint32),
    ('nanoseconds', np.uint32),
    ('data_valid_flag', np.uint32),
    ('altitude', np.float32),
    ('speed', np.float32),
    ('heading', np.float32)
])

JSF_POSITION_MESSAGE_TYPE = np.dtype([
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

JSF_STATUS_MESSAGE_TYPE = np.dtype([
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
