from ultrabak.config import load_config, UnknownUltraBakModuleException
from ultrabak.mods.postgresql import PostgresUltraBakModule


def get_mod(task):
    mod = None
    if task["mod"] == "postgresql":
        mod = PostgresUltraBakModule(task)

    if not mod:
        raise UnknownUltraBakModuleException()
    return mod


def backup(config):
    config_dict = load_config(config)

    for task in config_dict["tasks"]:
        mod = get_mod(task)
        mod.backup()


def list_backups(config):
    config_dict = load_config(config)
    backups = {}
    for task in config_dict["tasks"]:
        mod = get_mod(task)
        backups[task["name"]] = list(mod.list_backups())
    return backups
