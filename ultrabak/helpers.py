import datetime


def datetime_to_path(dt: datetime.datetime):
    return dt.strftime("%Y%m%d%H%M%S%f")
