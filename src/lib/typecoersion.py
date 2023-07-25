
def coerce(value, proptype):
    """ coerces a value to a proptype"""

    if proptype == "string":
        return str(value)

    if proptype == "integer":

        if (value is True or value is "true"):
            return 1

        try:
            return int(float(value))  # TODO may not work in small processor ?

        except ValueError:
            return 0

    if proptype == "boolean":
        if (value is False or value is 0 or value is "" or value is "false"):
            return False
        else:
            return True

    if proptype == "url":
        return str(value)
