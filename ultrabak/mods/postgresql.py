from zope.interface.declarations import implements, implementer

from ultrabak.helpers import datetime_to_path, path_to_datetime
from ultrabak.mods.base import BaseUltraBakModule
from ultrabak.mods.interface import IUltraBakModule
import os
import glob
import subprocess


@implementer(IUltraBakModule)
class PostgresUltraBakModule(BaseUltraBakModule):
    logger_name = "postgres"

    def __init__(self, config: dict):
        super().__init__(config)

        self.database = config["database"]

        self.format = "plain" #config.get("format", "plain")
        #TODO: Reactivate other methods, when restore is implemented

        if self.format == "directory":
            pass
        elif self.format == "plain":
            self.target_path += ".sql"
        elif self.format == "custom":
            self.target_path += ".dump"
        elif self.format == "tar":
            self.target_path += ".tar"

        self.no_owner = config.get("no_owner", False)
        self.no_acl = config.get("no_acl", False)
        self.clean = config.get("clean", False)
        self.inserts = config.get("inserts", False)
        self.if_exists = config.get("if_exists", False)
        self.lock_wait_timeout = config.get("lock_wait_timeout", 10000)

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

        params.append("--file=" + self.target_path)

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

        if self.no_password:
            params.append("--no-password")

        if self.database:
            params.append(self.database)

        return params

    def backup(self):
        self.get_logger().debug("Running Postgres Backup \"%s\"." % (self.name,))

        backup_params = self.get_backup_params()
        backup_env = self.get_env()

        self.get_logger().debug(str(backup_params))
        self.get_logger().debug(str(backup_env))

        backup_params = ['pg_dump', ] + backup_params

        if self.sudo:
            backup_params = ['sudo', '-u', self.sudo] + backup_params

        child = subprocess.Popen(backup_params,
                                 stdout=subprocess.PIPE,
                                 env=backup_env)
        (out, err) = child.communicate()
        rc = child.returncode

        if out:
            self.get_logger().debug(out)

        if err:
            self.get_logger().error(err)

        zip_status = True

        if rc > 0:
            self.get_logger().error("Postgres Backup \"%s\" failed." % (self.name,))
        else:
            self.get_logger().info("Postgres Backup \"%s\" successful." % (self.name,))

            if self.format in ("plain","custom","tar"):
                zip_status = self.zip_output()

        return rc == 0 and zip_status is True

    def zip_output(self):
        self.get_logger().debug("Zipping Postgres Backup \"%s\"." % (self.name,))
        child = subprocess.Popen(["bzip2", "-z", self.target_path])
        (out, err) = child.communicate()
        rc = child.returncode

        if out:
            self.get_logger().debug(out)

        if err:
            self.get_logger().error(err)

        if rc > 0:
            self.get_logger().error("Zipping Postgres Backup \"%s\" failed." % (self.name,))
        else:
            self.get_logger().info("Zipping Postgres Backup \"%s\" successful." % (self.name,))

        return rc == 0

    def list_backups(self):
        self.get_logger().debug("Listing Postgres Backups for \"%s\"." % (self.name,))
        for f in sorted(os.listdir(self.target_directory)):
            try:
                dt = path_to_datetime(f)
                self.get_logger().debug("File: \"%s\". Datetime: %s" % (f, dt.isoformat()))
                path = os.path.abspath(os.path.join(self.target_directory, f))
                yield {
                    "path": path,
                    "size": os.path.getsize(path),
                    "datetime": dt
                }
            except:
                self.get_logger().error("Error listing file: \"%s\"." % (f,))



