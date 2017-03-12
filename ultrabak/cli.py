import fire

from ultrabak.config import load_config, UnknownUltraBakModuleException
from ultrabak.mods.postgresql import PostgresUltraBakModule


def get_mod(task):
    mod = None
    if task["mod"] == "postgresql":
        mod = PostgresUltraBakModule(task)

    if not mod:
        raise UnknownUltraBakModuleException()
    return mod


class CmdInterface(object):

    @classmethod
    def backup(cls, config="/etc/ultrabak.d/**"):
        config_dict = load_config(config)

        for task in config_dict["tasks"]:
            mod = get_mod(task)
            mod.backup()

if __name__ == '__main__':
    fire.Fire(CmdInterface, name="ultrabak")
