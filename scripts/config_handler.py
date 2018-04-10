from ConfigParser import ConfigParser


class ConfigHandler(object):
    def __init__(self):
        self.conf = ConfigParser()
        self.conf.read('../config/dagger.ini')

    def get(self, section, name):
        return self.conf.get(section, name)

    def getfloat(self, section, name):
        return self.conf.getfloat(section, name)

