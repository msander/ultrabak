import fire

import ultrabak.ctrl as ctrl


class CmdInterface(object):

    @classmethod
    def backup(cls, config):
        ctrl.backup(config)

    @classmethod
    def list_backups(cls, config):
        backups = ctrl.list_backups(config)



def main():
    fire.Fire(CmdInterface, name="ultrabak")

if __name__ == '__main__':
    main()
