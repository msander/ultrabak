import fire

from ultrabak.config import Configurator


class CmdInterface(object):

    @classmethod
    def backup(cls, config="/etc/ultrabak.d/"):
        configurator = Configurator(config)


if __name__ == '__main__':
    fire.Fire(CmdInterface, name="ultrabak")
