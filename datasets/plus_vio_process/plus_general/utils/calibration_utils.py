import numpy as np
import yaml
import os
import glob

default_drive_dir="/home/plusai/ysy/work/drive"
default_calib_db_dir="/home/plusai/ysy/work/calib_db/calib_db"


def opencv_matrix(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return np.mat(mat)


yaml.add_constructor("tag:yaml.org,2002:opencv-matrix", opencv_matrix)


def load_opencv_yaml(yaml_path):
    with open(yaml_path) as f:
        # ignore first two line
        f.readline()  # %YAML:1.0
        f.readline()  # ---
        data = yaml.load(f, Loader=yaml.Loader)
    return data


def load_opencv_yaml_str(s):
    pos = -1
    pos = s.find('\n', pos+1)
    pos = s.find('\n', pos+1)
    return yaml.load(s[pos+1:], Loader=yaml.Loader)


def search_calib_file(car_name, keyname, date, search_latest_before_data, search_latest, calib_db_dir=None):
    if calib_db_dir is None:
        calib_db_dir = default_calib_db_dir
    files = glob.glob(os.path.join(calib_db_dir, '*.yml'))
    base_files = [os.path.basename(f) for f in files]
    if not search_latest_before_data and not search_latest:
        file_name = car_name+'_'+date+'_'+keyname+'.yml'
        if file_name in base_files:
            return os.path.join(calib_db_dir, file_name)
        else:
            return None
    else:
        possibles = []
        for file in base_files:
            file_wo_postfix = os.path.splitext(file)[0]
            if file_wo_postfix.startswith(car_name) and file_wo_postfix.endswith(keyname):
                possibles.append(file)

        latest_date = None

        for possible in possibles:
            possible_date = possible.split('_')[1]
            to_judge = True if search_latest else possible_date < date
            if to_judge:
                if latest_date is None:
                    latest_date = possible_date
                else:
                    latest_date = possible_date if possible_date > latest_date else latest_date

        if latest_date is None:
            return None
        else:
            return os.path.join(calib_db_dir, car_name+'_'+latest_date+'_'+keyname+'.yml')


def grep_info_from_proto_lines(lines, key_list):
    from google.protobuf.text_format import Tokenizer
    tokenizer = Tokenizer(lines)
    key_id = 0
    info = None
    while not tokenizer.AtEnd():
        token = tokenizer.token
        if token == key_list[key_id]:
            tokenizer.NextToken()
            if tokenizer.TryConsume('{') or tokenizer.TryConsume(':'):
                if key_id == len(key_list) - 1:
                    info = tokenizer.token
                    break
                else:
                    key_id += 1
            else:
                raise ValueError
        tokenizer.NextToken()

    if info is None:
        return None

    if '"' in info:
        info = info.split('"')[1]
    return info


def grep_info_from_cfg(cfg_file, key_list):
    def grep_info(line):
        return line.split('"')[1]

    with open(cfg_file) as f:
        lines = f.readlines()

    parent_path = None
    if 'inherit' in lines[0]:
        parent_name = grep_info(lines[0])
        parent_path = os.path.join(os.path.dirname(cfg_file), parent_name)

    info = grep_info_from_proto_lines(lines, key_list)

    if info or parent_path is None:
        return info
    else:
        return grep_info_from_cfg(parent_path, key_list)


def get_side_cam_info_from_cfg(cfg_file, key):
    def grep_info(line):
        return line.split('"')[1]

    with open(cfg_file) as f:
        lines = f.readlines()

    parent_path = None
    if 'inherit' in lines[0]:
        parent_name = grep_info(lines[0])
        parent_path = os.path.join(os.path.dirname(cfg_file), parent_name)

    from google.protobuf.text_format import Tokenizer
    tokenizer = Tokenizer(lines)
    find_camera = False
    while not tokenizer.AtEnd():
        token = tokenizer.token
        if find_camera:
            if token == 'cameras':
                tokenizer.NextToken()
                tokenizer.Consume('{')
                calib_name = None
                calib_date = None
                while not tokenizer.AtEnd():
                    token = tokenizer.token
                    if token == '}':
                        break
                    if token == 'calib_name':
                        tokenizer.NextToken()
                        tokenizer.TryConsume(':')
                        name = tokenizer.token
                        name = name.split('"')[1]
                        if name == key:
                            calib_name = name
                    if token == 'calib_date':
                        tokenizer.NextToken()
                        tokenizer.TryConsume(':')
                        data = tokenizer.token
                        data = data.split('"')[1]
                        calib_date = data
                    tokenizer.NextToken()
                if calib_name == key:
                    return calib_date
        else:
            if token == 'unified_camera':
                find_camera = True
        tokenizer.NextToken()

    if parent_path is not None:
        return get_side_cam_info_from_cfg(parent_path, key)
    else:
        return None


