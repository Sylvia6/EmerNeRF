import os
import fnmatch
import yaml
import pickle


def make_dirs(dirname):
    if not os.path.exists((dirname)):
        os.makedirs(dirname, exist_ok=True)


def save_pickle(obj, path):
    with open(path, 'wb') as f:
        pickle.dump(obj, f)


def load_pickle(path):
    with open(path, 'rb') as f:
        return pickle.load(f)


def get_file(path, postfixs):
    result = []
    for root, dirs, files in os.walk(path):
        for file in files:
            # print(os.path.splitext(file)[1])
            postfix = os.path.splitext(file)[1]
            if postfix in postfixs:
                result.append(os.path.join(root, file))
    return result


def get_folder(path, condition=None):
    result = []
    for root, dirs, files in os.walk(path):
        for d in dirs:
            if condition is not None:
                if condition(d):
                    result.append(os.path.join(root, d))
            else:
                 result.append(os.path.join(root, d))
    return result


def recursive_glob(rootdir='.', pattern='*'):
    """Search recursively for files matching a specified pattern.
    """
    matches = []
    for root, dirnames, filenames in os.walk(rootdir):
        for filename in fnmatch.filter(filenames, pattern):
            matches.append(os.path.join(root, filename))

    return matches


def read_yaml(fpath):
    with open(fpath, "r") as f:
        cfg = yaml.load(f, yaml.Loader)
        if cfg.get('common', None) is not None:
            cfg['common']['cfg_path'] = fpath
    return cfg