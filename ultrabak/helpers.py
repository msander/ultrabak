import datetime
import os

def datetime_to_path(dt: datetime.datetime) -> str:
    return dt.strftime("%Y-%m-%d-%H-%M-%S-%f")


def path_to_datetime(s: str) -> datetime.datetime:
    s = os.path.basename(s)
    return datetime.datetime.strptime(s[0:len("2017-03-12-09-26-02-680594")], "%Y-%m-%d-%H-%M-%S-%f")

