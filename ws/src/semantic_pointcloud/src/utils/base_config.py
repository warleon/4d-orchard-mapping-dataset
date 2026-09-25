from dataclasses import MISSING, fields, is_dataclass, dataclass
from typing import TypeVar

import rospy

TConfig = TypeVar("TConfig", bound="BaseConfig")


def _paramName(namespace: str, fieldName: str) -> str:
    # "~" (private ns) takes the field name directly ("~foo"); anything else
    # needs a separating slash ("~foo/bar").
    return (
        f"{namespace}{fieldName}"
        if namespace.endswith("~")
        else f"{namespace}/{fieldName}"
    )


@dataclass()
class BaseConfig:
    @classmethod
    def load(cls: type[TConfig], namespace: str = "~") -> TConfig:
        kwargs = {}
        for f in fields(cls):
            paramName = _paramName(namespace, f.name)

            if is_dataclass(f.type) and issubclass(f.type, BaseConfig):
                kwargs[f.name] = f.type.load(paramName)
            elif f.default is not MISSING:
                kwargs[f.name] = rospy.get_param(paramName, f.default)
            elif f.default_factory is not MISSING:
                kwargs[f.name] = rospy.get_param(paramName, f.default_factory())
            else:
                kwargs[f.name] = rospy.get_param(paramName)

        return cls(**kwargs)
