from ConfigParser import ConfigParser

from rospkg import RosPack
import os

class ConfigHandler(object):
    def __init__(self):
        self.conf = ConfigParser()
        os.chdir(RosPack().get_path('panda_dagger'))
        self.conf.read('./config/dagger.ini')

    def get(self, section, name):
        return self.conf.get(section, name)

    def getfloat(self, section, name):
        return self.conf.getfloat(section, name)

    def getint(self, section, name):
        return self.conf.getint(section, name)
