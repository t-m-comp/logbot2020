import argparse
import configparser
import os
import codecs

config = configparser.ConfigParser(allow_no_value=True)
config.optionxform = str
config.read('default_args.txt')
defaults = config['DEFAULT']

parser = argparse.ArgumentParser()
parser.add_argument('CONTROL_TYPE', choices=["SAMPLE", "ACC_TREE", "GPS_TREE"])
parser.add_argument('--ACC_BUFFER_SIZE', help='ACC samples per second')
parser.add_argument('--APPLY_HEURISTICS', action='store_const', const=True, help="Whether to apply apply heuristics when controlling the camera based on GPS and ACC data")
parser.add_argument('--CONSEC', help='Number of consecutive windows that should be classified as the target class before activating camera when using GPS_TREE')
parser.add_argument('--DAWN', help='Time (in minutes since midnight) to wake logger when using SLEEP_CTRL_BY_TIME')
parser.add_argument('--DUSK', help='Time (in minutes since midnight) to put logger to sleep when using SLEEP_CTRL_BY_TIME')
parser.add_argument('--FLIGHT_COUNTDOWN', help='Number of consecutive windows of stationary behavior to wait before switching from flying state to stationary state')
parser.add_argument('--FLIGHT_MIN', help='Number of consecutive windows of flying behavior to wait before switching from stationary state to flying state')
parser.add_argument('--FPT_KM', help='KM used when computing FPT for GPS_TREE')
parser.add_argument('--GPS_BUFFER_SIZE', help='Window size in number of GPS locations to use when computing features for GPS_TREE')
parser.add_argument('--GPS_PER_MINUTE', action='store_const', const=True, help="Whether to record GPS locations once per minute (True) or once per second (False)")
parser.add_argument('--ILLUM_THRESHOLD', help='Minimum illumination (in lux) required for camera to be activated')
parser.add_argument('--MAX_GAP', help="Maximum gap allowed between GPS locations before resetting window when computing features for GPS_TREE")
parser.add_argument('--MAX_VIDEO_COUNT', help="Maximum number of videos that can be recorded, useful when the logger's battery life allows for recording more videos than can fit on SD card")
parser.add_argument('--PLUS_DEPTH', action='store_const', const=True, help="Whether to also allow depth based camera activation (in addition to SAMPLE/ACC_TREE/GPS_TREE)")
parser.add_argument('--PRESSURE_BUFFER_SIZE', help="Number of pressure readings to use when deciding whether to activate the camera based on depth readings")
parser.add_argument('--PRESSURE_THRESHOLD', help='Threshold difference in pressure readings that must be present in the buffer before activating the camera')
parser.add_argument('--RECORD_TIME_SEC', help="Number of seconds to record video after activating the camera")
parser.add_argument('--REST_TIME_SEC', help='Number of seconds to not record video after finishing RECORD_TIME_SEC. This determines the sampling rate when using the SAMPLE camera control method.')
parser.add_argument('--SIMULATE_8_BIT_RAW', action='store_const', const=True, help='Whether to convert the raw ACC values to simulate 8-bit raw values. Used when the ACC_TREE was trained using 8-bit AxyTrek data')
parser.add_argument('--SLEEP_CTRL_BY_TIME', action='store_const', const=True, help='Whether to have the logger sleep from DUSK to DAWN')
parser.add_argument('--STARTUP_DELAY', help='Number of minutes for logger to sleep after GPS initialization. Useful when initializing the device prior to travelling to the field.')
parser.add_argument('--TARGET_CLASS_NUMBER', help='Which GPS_TREE/ACC_TREE output class to use as the target class when activating the camera')
parser.add_argument('--TREE_FILE', help='Full path to text file containing ACC_TREE/GPS_TREE code to use onboard the logger')
parser.add_argument('--UTC_OFFSET', help='UTC_OFFSET to use when determining DAWN/DUSK based on UTC timestamps from GPS')
args = vars(parser.parse_args())

if args["CONTROL_TYPE"] == "SAMPLE":
    args["SAMPLE"] = True
elif args["CONTROL_TYPE"] == "ACC_TREE":
    args["ACC_TREE"] = True
elif args["CONTROL_TYPE"] == "GPS_TREE":
    args["GPS_TREE"] = True

build_options = dict(defaults)
build_options["SAMPLE"] = False
build_options["ACC_TREE"] = False
build_options["GPS_TREE"] = False

build_options.update({k: v for k, v in args.items() if v is not None})
build_options.update({k: (v.lower() == "true") for k, v in build_options.items() if v is not None and isinstance(v, str) and v.lower() in ["true", "false"]})

if sum([build_options["SAMPLE"], build_options["ACC_TREE"], build_options["GPS_TREE"]]) != 1:
    parser.error("Error: please specify only one camera control type (SAMPLE, ACC_TREE, or GPS_TREE)")

input_dir = "src_txt"
output_dir = "logbot" + "_" + build_options["CONTROL_TYPE"]
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

src_file_list = [
    "user_data_type",
    "logbot",
    "BH1721FVC",
    "BMX055",
    "camera",
    "CAMM8Q",
    "I2C_functions",
    "LPS22HB",
    "MS5837",
    "RX8564LC",
    "SD_functions",
]

for src_file in src_file_list:
    if src_file in ["user_data_type"]:
        extension = ".h"
    else:
        extension = ".ino"

    if src_file == "logbot":
        extension = "_" + build_options["CONTROL_TYPE"] + ".ino"

    with open(os.path.join(input_dir, src_file + ".txt"), "r", encoding="UTF-8") as read_file:
        with open(os.path.join(output_dir, src_file + extension), "w", encoding="UTF-8") as write_file:
            for line in read_file:
                line = line.rstrip()
                if line.startswith(codecs.BOM_UTF8.decode("UTF-8")):
                    line = line.lstrip(codecs.BOM_UTF8.decode("UTF-8"))

                if "//ADD_TREE_HERE" in line:
                    if build_options["TREE_FILE"] is not None:
                        with open(build_options["TREE_FILE"], "r", encoding="UTF-8") as tree_file:
                            for line in tree_file:
                                line = line.rstrip()
                                if line.startswith(codecs.BOM_UTF8.decode("UTF-8")):
                                    line = line.lstrip(codecs.BOM_UTF8.decode("UTF-8"))
                                write_file.write(line + "\n")
                    else:
                        write_file.write(line + "\n")
                else:
                    if line.startswith("@"): #set configurable value
                        line = line[1:]
                        write_file.write("#define " + line + " " + build_options[line] + "\n")
                    elif line.startswith("?"): #set flag
                        line = line[1:]
                        if build_options[line]:
                            write_file.write("#define " + line + "\n")
                    elif line.startswith("&"): #conditionally add this line
                        line = line[1:].split("@")
                        add = True
                        for item in line[:-1]:
                            if "!" in item:
                                add = add and not build_options[item[1:]]
                            else:
                                add = add and build_options[item]
                        if add:
                            write_file.write(line[-1] + "\n")
                    else:
                        write_file.write(line + "\n")