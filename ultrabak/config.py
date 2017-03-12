import yaml
import glob


def load_config(path):
    collected_config = {
        "tasks": list(),
        "general": dict()
    }

    for configfile in glob.iglob(path):
        if configfile.endswith(".yml"):
            with open(configfile, "r") as f:
                config = yaml.load(f,)

                if "tasks" in config:
                    collected_config["tasks"] += config["tasks"]

                if "general" in config:
                    collected_config["general"].update(config["general"])

    return collected_config


class UnknownUltraBakModuleException(Exception):
    pass
