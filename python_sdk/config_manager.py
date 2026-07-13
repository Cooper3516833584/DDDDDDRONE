import configparser
import os
from typing import Any, Dict, Optional

import numpy as np


class ConfigManager:
    def __init__(
        self,
        file: str = "config.ini",
        default_setting: dict = {},
        section: str = "SETTINGS",
    ):
        """
        file: config file path
        default_setting: default settings dictionary, set if the option is not set yet
        section: section name
        """
        path = os.path.dirname(__file__)
        file_path = os.path.join(path, file)
        self._config_file = file_path
        self._config = configparser.ConfigParser()
        if os.path.exists(file_path):
            self._config.read(file_path)
        else:
            open(file_path, "w").close()
        self._section_name = section.upper()
        self._init_file(default_setting.copy())

    def reload(self):
        """
        Reload the config file
        """
        self._config.read(self._config_file)

    def save(self):
        """
        Save the config file
        """
        with open(self._config_file, "w") as f:
            self._config.write(f)

    def set(self, option: str, value: Any, section=None):
        """
        Set the value of an option
        """
        section = section if section is not None else self._section_name
        if section not in self._config.sections():
            self._config.add_section(section)
        if isinstance(value, np.ndarray):
            value = value.tolist()
        if not isinstance(value, str):
            value = repr(value)
        self._config.set(section, option, value)
        self.save()

    def remove(self, option: str, section=None):
        """
        Remove an option
        """
        section = section if section is not None else self._section_name
        self._config.remove_option(section, option)
        self.save()

    def _init_file(self, default_setting: dict):
        if self._section_name not in self._config.sections():
            self._config.add_section(self._section_name)
        for key, value in default_setting.items():
            if not self._config.has_option(self._section_name, key):
                self._config.set(self._section_name, key, str(value))
        self.save()

    def set_from_dict(self, setting_dict: dict, section=None):
        """
        Set values by a given dictionary
        """
        section = section if section is not None else self._section_name
        if section not in self._config.sections():
            self._config.add_section(section)
        for key, value in setting_dict.items():
            self._config.set(section, key, str(value))
        self.save()

    def dict(self, section=None) -> Dict[str, str]:
        """
        Return a dictionary of all options
        """
        section = section if section is not None else self._section_name
        items = self._config.items(section)
        return {i[0]: i[1] for i in items}

    def clear_all(self, section=None):
        """
        Remove all options
        """
        section = section if section is not None else self._section_name
        for key in self.dict().keys():
            self._config.remove_option(section, key)
        with open(self._config_file, "w") as f:
            self._config.write(f)

    def get(self, option: str, default: Optional[str] = None, section=None) -> Optional[str]:
        """
        Get the value of an option, return as string
        """
        section = section if section is not None else self._section_name
        if self._config.has_option(section, option):
            return self._config.get(section, option)
        else:
            if default is not None:
                self.set(option, default)
            return default

    def get_bool(self, option: str, default: Optional[bool] = None, section=None) -> Optional[bool]:
        """
        Get the value of an option, return as boolean
        """
        section = section if section is not None else self._section_name
        if self._config.has_option(section, option):
            return self._config.getboolean(section, option)
        else:
            if default is not None:
                self.set(option, default)
            return default

    def get_int(self, option: str, default: Optional[int] = None, section=None) -> Optional[int]:
        """
        Get the value of an option, return as integer
        """
        section = section if section is not None else self._section_name
        if self._config.has_option(section, option):
            return self._config.getint(section, option)
        else:
            if default is not None:
                self.set(option, default)
            return default

    def get_float(self, option: str, default: Optional[float] = None, section=None) -> Optional[float]:
        """
        Get the value of an option, return as float
        """
        section = section if section is not None else self._section_name
        if self._config.has_option(section, option):
            return self._config.getfloat(section, option)
        else:
            if default is not None:
                self.set(option, default)
            return default

    def get_eval(self, option: str, default: Optional[Any] = None, section=None) -> Optional[Any]:
        """
        Get the value of an option, return as python object
        """
        section = section if section is not None else self._section_name
        if self._config.has_option(section, option):
            return eval(self._config.get(section, option))
        else:
            if default is not None:
                self.set(option, default)
            return default

    def get_array(
        self, option: str, default: Optional[Any] = None, dtype: Optional[str] = None, section=None
    ) -> Optional[np.ndarray]:
        """
            Get the value of an option, return as numpy array

        section = section if section is not None else self._section_name    dtype: str, optional, numpy format, default None
        """
        section = section if section is not None else self._section_name
        if self._config.has_option(section, option):
            string = self._config.get(section, option).strip()
            # string = re.sub(r"\[\s+", r"[", string)
            # string = re.sub(r"\s(\d)", r",\1", string)
            # string = string.replace("\n", ",")
            if dtype == None:
                return np.array(eval(string))
            else:
                return np.array(eval(string), dtype)
        else:
            if default is not None:
                self.set(option, default)
            return default
