import os
import functools
import traceback


def load_all_module_in_module_folder_orders(file, module_str, exception=None):
    if exception is None:
        exception = []
    file_dir = os.path.dirname(file)

    module_names = module_str.split('.')
    dst_dir = file_dir
    for m in module_names:
        dst_dir = os.path.join(dst_dir, m) if m else dst_dir

    files = os.listdir(dst_dir)
    orders = []
    for file in files:
        if file in exception or file == '__init__.py':
            continue
        if os.path.isfile(os.path.join(dst_dir, file)) and file.endswith('.py'):
            module_name = file[:-3]
            # exec("from {} import {}".format(".".join(module_names), module_name))
            orders.append("from {} import {}".format(".".join(module_names), module_name))
    return orders


def add_dict_with_prefix(dst_dict, new_dict, prefix):
    for k, v in new_dict.items():
        dst_dict[prefix + "_" + k] = v
    return dst_dict


def merge_dict(config, more_config, prefer_latter=True):
    if more_config is None:
        return config
    out_config = {}
    for k in config.keys():
        if k not in more_config:
            out_config[k] = config[k]
        else:
            v1 = config[k]
            v2 = more_config[k]
            if isinstance(v1, dict) and isinstance(v2, dict):
                out_config[k] = merge_dict(v1, v2,
                                           prefer_latter=prefer_latter)
            else:
                if prefer_latter:
                    out_config[k] = v2
                else:
                    out_config[k] = v1
    for k in more_config.keys():
        if k not in out_config:
            out_config[k] = more_config[k]
    return out_config


class AverageMeter(object):
    """A meter which calculate average value of objects
    """

    def __init__(self):
        self.data = {}

    def __setitem__(self, k, v):
        """
        if k not in self.data, it will be added to it,
        else the average value will be updated
        Args:
            k : object name
            v : object value
        """
        if k not in self.data:
            self.data[k] = [v, 1]
        else:
            last_v, cnt = self.data[k]
            next_v = (last_v * cnt + v) / (cnt + 1)
            self.data[k] = [next_v, cnt + 1]

    def dump(self):
        """
        return a dict which contain the average values of objects
        Returns:
            dict: a dict which contain the average values of objects

        """
        return {k: v[0] for k, v in self.data.items()}


def reraise_with_stack(func):
    # "http://gael-varoquaux.info/programming/decoration-in-python-done-right-decorating-\
    # and-pickling.html"
    # functools.wraps is needed to handle some name resolution issues in order
    # for wrapped functions to be pickleable.
    @functools.wraps(func)
    def wrapped(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            traceback_str = traceback.format_exc(e)
            raise Exception("Caught exception with stacktrace wrapper; traceback "
                            "is\n%s\n" % traceback_str)
    return wrapped
