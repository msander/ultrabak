import yaml
import glob


class Configurator():
    def __init__(self, path):
        collected_config = {}
        for configfile in glob.glob(path, ".yml", True):
            with open(configfile, "r") as f:
                config = yaml.load(f,)

                if "tasks" in config:
                    collected_config["tasks"] += config["tasks"]

                if "general" in config:
                    collected_config["general"].update(config["general"])
        return collected_config
