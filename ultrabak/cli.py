import fire
import humanize
import tabulate

import ultrabak.ctrl as ctrl


class CmdInterface(object):

    @classmethod
    def backup(cls, config):
        ctrl.backup(config)

    @classmethod
    def list_backups(cls, config):
        backups = ctrl.list_backups(config)
        rows = list()
        for task, tbackups in backups.items():
            for tb in tbackups:
                rows.append([
                    task,
                    tb["datetime"],
                    humanize.naturalsize(tb["size"], gnu=True),
                    tb["path"]
                ])
        print(tabulate.tabulate(rows, headers=["Task", "Datetime (UTC)", "Size", "Path"]))


def main():
    fire.Fire(CmdInterface, name="ultrabak")

if __name__ == '__main__':
    main()
