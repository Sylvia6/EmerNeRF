class Register(object):
    def __init__(self, name):
        self.dict = {}
        self.name = name

    def register(self, name):
        def warp(obj):
            self.dict[name] = obj
            return obj
        return warp

    def __call__(self, item):
        return self.register(item)

    def __getitem__(self, item):
        if item not in self.dict:
            raise NotImplementedError("{} not found in {}!".format(item, self.name))
        return self.dict[item]

    def __setitem__(self, key, value):
        self.dict[key] = value
