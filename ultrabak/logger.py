import logging


class Logger:
    def __init__(self, name: str):
        self.log = logging.getLogger("ultrabak."+name)
        self.log.setLevel(logging.DEBUG)

        # Console Handler
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)

        # Formatter
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)

        # add the handlers to the logger
        self.log.addHandler(ch)

    def info(self, message: str):
        self.log.info(message)

    def debug(self, message: str):
        self.log.debug(message)

    def warning(self, message: str):
        self.log.warning(message)

    def error(self, message: str):
        self.log.error(message)
