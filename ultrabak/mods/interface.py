import zope.interface


class IUltraBakModule(zope.interface.Interface):

    def configure(config: dict = {}):
        """Configures the module"""

    def backup(self):
        """Run the backup job"""

    def list_backups(self):
        """List the currently available backups"""

    def restore(self):
        """Runs the restore job"""


class BackupNotConfiguredException(Exception):
    pass


class RestoreNotConfiguredException(Exception):
    pass