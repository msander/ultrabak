from ultrabak.helpers import datetime_to_path
from ultrabak.logger import Logger
import datetime
import os


class BaseUltraBakModule:

    def __init__(self, config: dict):
        self.logger = Logger(self.__class__.logger_name)
        self.name = config.get("name", "Unnamed Backup")

        self.target_directory = config["target_directory"]

        self.target_name = datetime_to_path(datetime.datetime.utcnow())
        self.get_logger().debug("Setting target name to \"%s\"" % (self.target_name,))

        self.target_path = os.path.join(self.target_directory, self.target_name)
        self.get_logger().debug("Setting target path to \"%s\"" % (self.target_path,))

        self.source_directory = config["target_directory"]

    def get_logger(self):
        return self.logger
