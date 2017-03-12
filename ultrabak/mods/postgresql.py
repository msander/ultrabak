from zope.interface.declarations import implements

from ultrabak.helpers import datetime_to_path
from ultrabak.mods.base import BaseUltraBakModule
from ultrabak.mods.interface import IUltraBakModule
import os
import subprocess


class PostgresUltraBakModule(BaseUltraBakModule):
    implements(IUltraBakModule)

    def __init__(self, config: dict):
        super().__init__()

        if "databases" in config:
            self.databases = config["databases"]
            self.all_databases = False
        else:
            self.all_databases = True

        self.format = config.get("format", "plain")
        self.no_owner = config.get("no_owner", False)
        self.no_acl = config.get("no_acl", False)
        self.clean = config.get("clean", False)
        self.inserts = config.get("inserts", False)
        self.if_exists = config.get("if_exists", False)
        self.lock_wait_timeout = config.get("lock_wait_timeout",10000)

        # We force to quote all identifiers for compatibility reasons with different pg versions
        self.quote_all_identifiers = True

        self.host = config.get("host", None)
        self.port = config.get("port", None)
        self.username = config.get("username", None)
        self.password = config.get("password", None)

        # Never ask for password in a script
        self.no_password = True

    def get_env(self):
        e = os.environ.copy()
        if self.password:
            e["PGPASSWORD"] = self.password
        return e

    def get_backup_params(self):
        params = list()

        params.append("--format="+self.format)

        if self.format == "directory":
            params.append("--file="+os.path.join(self.target_directory, self.target_name))

        params.append("--lock-wait-timeout=" + str(self.lock_wait_timeout))

        if self.no_owner:
            params.append("--no-owner")

        if self.no_acl:
            params.append("--no-acl")

        if self.clean:
            params.append("--clean")

        if self.inserts:
            params.append("--inserts")

        if self.if_exists:
            params.append("--if-exists")

        if self.quote_all_identifiers:
            params.append("--quote-all-identifiers")

        if self.host:
            params.append("--host="+str(self.host))

        if self.port:
            params.append("--port=" + str(self.port))

        if self.username:
            params.append("--username=" + str(self.username))

        if self.password:
            params.append("--password=" + str(self.password))

        if self.no_password:
            params.append("--no-password")

        return params

    def backup(self):
        self.get_logger().info("Running Postgres Backup \"%s\"." % (self.name,))

        backup_params = self.get_backup_params()
        backup_env = self.get_env()

        self.get_logger().debug(str(backup_params))
        self.get_logger().debug(str(backup_env))

        (out, err) = subprocess.Popen(['pg_dump'] + backup_params,
                                      stdout=subprocess.PIPE,
                                      env=backup_env).communicate()

        if out:
            self.get_logger().debug(out)

        if err:
            self.get_logger().error(err)
            self.get_logger().info("Postgres Backup \"%s\" failed." % (self.name,))
        else:
            self.get_logger().info("Postgres Backup \"%s\" successful." % (self.name,))

        return bool(err)
