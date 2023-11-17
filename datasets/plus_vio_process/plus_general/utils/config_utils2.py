import yaml
import copy
import os


def dump_object(obj):
    if isinstance(obj, dict):
        return {k: dump_object(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [dump_object(v) for v in obj]
    else:
        return obj


class AnyObject(object):
    """To be used as any type in CfgNode.

    If an object is AnyObject in CfgNode, type check will ignore this.
    """
    def __init__(self):
        pass


default_yaml_root = [
    os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..'))
]


class CfgNode(dict):
    def __init__(self):
        super(CfgNode, self).__init__()

    def __setattr__(self, name, value):
        self[name] = value

    def __getattr__(self, name):
        if name in self:
            return self[name]
        else:
            raise AttributeError(name)

    @classmethod
    def from_dict(cls, other_dict, check_hydra=False, extra_yaml_roots=None):
        if not isinstance(other_dict, dict):
            return other_dict

        overwrite_expr = {}
        if check_hydra:
            if "__inherit__" in other_dict:
                node = cls.from_yaml(other_dict['__inherit__'], extra_yaml_roots=extra_yaml_roots)
                other_dict.pop('__inherit__')
            else:
                node = cls()

            if "__overwrite__" in other_dict:
                overwrite_expr = other_dict['__overwrite__']
                other_dict.pop('__overwrite__')
        else:
            node = cls()

        for k, v in other_dict.items():
            if isinstance(v, dict):
                other_node = CfgNode.from_dict(v, check_hydra=check_hydra, extra_yaml_roots=extra_yaml_roots)
                if k in node and isinstance(node[k], CfgNode):
                    node[k].merge_cfg(other_node, strict=False)
                else:
                    node[k] = other_node
            elif isinstance(v, list):
                node[k] = [CfgNode.from_dict(obj, check_hydra=check_hydra, extra_yaml_roots=extra_yaml_roots) for obj in v]
            else:
                node[k] = v

        if overwrite_expr:
            for k, v in overwrite_expr.items():
                exec("node.{}=v".format(k))

        return node

    @classmethod
    def from_yaml(cls, other_yaml, extra_yaml_roots=None):
        yaml_roots = copy.deepcopy(default_yaml_root)
        if extra_yaml_roots is not None:
            yaml_roots.extend(extra_yaml_roots)

        if not os.path.exists(other_yaml):
            found = False
            for root in yaml_roots:
                if os.path.exists(os.path.join(root, other_yaml)):
                    other_yaml = os.path.join(root, other_yaml)
                    found = True
                    break
            if not found:
                raise IOError("file {} not found!".format(other_yaml))

        with open(other_yaml) as f:
            data = yaml.safe_load(f)
        hydra = False
        if data.get('__hydra__', False):
            hydra = True
            data.pop('__hydra__')
        yaml_dir = os.path.realpath(os.path.dirname(other_yaml))
        return cls.from_dict(data, check_hydra=hydra, extra_yaml_roots=[yaml_dir])

    def type_check(self, k, other_v):
        """Function that do type compare between two value
        If you need special type check, write it here.

        Args:
            k: key in this cfg
            other_v: value of other cfg

        Returns:
            Return if valid type.
        """
        if isinstance(k, (int, float, complex)):
            return isinstance(other_v, (int, float, complex))

        return isinstance(other_v, type(self[k]))

    def merge_cfg(self, other_cfg, strict=True, ignore_non_exist=True):
        """merge a CfgNode to this CfgNode

        Args:
            other_cfg (CfgNode): the CfgNode we need to merge.
            strict (bool, optional): whether check type. Defaults to False.
            ignore_non_exist (bool, optional): whether ignore unexpected params in the new cfg
        Raises:
            TypeError: when strict=True, we need other_cfg has save key type with this config.
        """
        if not isinstance(other_cfg, CfgNode):
            raise TypeError("merge_cfg need CfgNode, but get {}.".format(type(other_cfg)))

        for k, v in other_cfg.items():
            if strict:
                if k in self:
                    if isinstance(self[k], CfgNode):
                        if not isinstance(v, CfgNode):
                            raise TypeError("need CfgNode for {}, but get {}.".format(k, type(v)))
                        self[k].merge_cfg(v, strict=strict)
                    elif self[k] is None or isinstance(self[k], AnyObject):
                        self[k] = v
                    else:
                        if not self.type_check(k, v):
                            raise TypeError("type of {} should be {}.".format(k, type(self[k])))
                        self[k] = v
                else:
                    if not ignore_non_exist:
                        raise TypeError("unexcepted key: {}".format(k))
            else:
                self[k] = v
        return self

    def merge_dict(self, other_dict, strict=True, ignore_non_exist=True):
        """merge a dict to this CfgNode

        Args:
            other_dict (dict): the dict we need to merge.
            strict (bool, optional): whether check type. Defaults to False.
            ignore_non_exist (bool, optional): whether ignore unexpected params in the new cfg
        Raises:
            TypeError: when strict=True, we need other_cfg has save key type with this config.
        """
        if not isinstance(other_dict, dict):
            raise TypeError("need type dict, but get {}".format(type(other_dict)))
        other_cfg = CfgNode.from_dict(other_dict)
        self.merge_cfg(other_cfg, strict=strict, ignore_non_exist=ignore_non_exist)
        return self

    def merge_yaml(self, yaml_path, strict=True, ignore_non_exist=True):
        """merge a dict to this CfgNode

        Args:
            yaml_path (str): the yaml we need to merge.
            strict (bool, optional): whether check type. Defaults to False.
            ignore_non_exist (bool, optional): whether ignore unexpected params in the new cfg
        Raises:
            TypeError: when strict=True, we need other_cfg has save key type with this config.
        """
        self.merge_cfg(CfgNode.from_yaml(yaml_path), strict=strict, ignore_non_exist=ignore_non_exist)
        return self

    def merge(self, other, strict=True, ignore_non_exist=True):
        """merge dict, yaml or CfgNode to this CfgNode

        Args:
            other: must be dict, yaml path or CfgNode
            strict (bool, optional): whether check type. Defaults to False.
            ignore_non_exist (bool, optional): whether ignore unexpected params in the new cfg
        Returns:
        Raises:
            TypeError: if other is not in [dict, yaml, CfgNode], raise TypeError
        """
        if isinstance(other, str):
            return self.merge_yaml(other, strict=strict, ignore_non_exist=ignore_non_exist)
        elif isinstance(other, CfgNode):
            return self.merge_cfg(other, strict=strict, ignore_non_exist=ignore_non_exist)
        elif isinstance(other, dict):
            return self.merge_dict(other, strict=strict, ignore_non_exist=ignore_non_exist)
        else:
            raise TypeError("can only merge CfgNode, dict, yaml, but got {}".format(type(other)))

    def dump(self, yaml_path=None):
        """dump CfgNode to dict

        Args:
            yaml_path (str, optional): If not None, save This CfgNode to yaml

        Returns:
            dict: the dumped result

        """
        data = dump_object(self)
        if yaml_path is not None:
            with open(yaml_path, 'w') as f:
                yaml.dump(data, f, sort_keys=True, default_flow_style=None)
        return data

    def str(self):
        data = self.dump()
        return yaml.dump(data)


if __name__ == '__main__':
    # cfg = CfgNode()
    # cfg = cfg.from_yaml("/home/plusai/ysy/work/plus_general_model/train_config/20220701/unified_train_full.yaml")
    # cfg.dump("test.yaml")
    # from collections import OrderedDict
    # a = OrderedDict()
    b = "aaaa".split('\n')
    print(b)
