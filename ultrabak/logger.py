import logging


class Logger:
    def __init__(self, name: str):
        self.log = logging.getLogger("ultrabak."+name)

    def info(self, message: str):
        self.log.info(message)

    def debug(self, message: str):
        self.log.debug(message)

    def warning(self, message: str):
        self.log.warning(message)

    def error(self, message: str):
        self.log.error(message)
