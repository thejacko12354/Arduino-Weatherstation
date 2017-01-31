"""
Support for an Arduino Weatherstation

For more details about this platform, please refer to the documentation
https://home-assistant.io/components/demo/
"""
import logging
from datetime import timedelta

import requests
import voluptuous as vol

from homeassistant.components.sensor import PLATFORM_SCHEMA
from homeassistant.components.weather import WeatherEntity
from homeassistant.const import (
    CONF_NAME, CONF_HOST, CONF_MONITORED_CONDITIONS, ATTR_ATTRIBUTION)
from homeassistant.helpers.entity import Entity
import homeassistant.helpers.config_validation as cv

_LOGGER = logging.getLogger(__name__)

CONF_UNITS = 'units'

DEFAULT_NAME = 'Weatherstation'

# Sensor types are defined like so:
# Name, si unit, us unit, ca unit, uk unit, uk2 unit
SENSOR_TYPES = {
    'air_temperature': ['Temperature',
                        '°C', '°F', '°C', '°C', '°C', 'mdi:thermometer'],
    'air_humidity': ['Humidity', '%', '%', '%', '%', '%', 'mdi:water-percent'],
    'air_pressure': ['Pressure', 'mbar', 'mbar', 'mbar', 'mbar', 'mbar',
                     'mdi:gauge'],
    'brightness': ['Brightness', 'lm', 'lm', 'lm', 'lm', 'lm', 'mdi:brightness-4'],
    'precipation': ['Precipation', None, None, None, None, None,
                    'mdi:weather-snowy-rainy'],
    'soil_moisture': ['Moisture', '%', '%', '%', '%', '%', 'mdi:water-percent'],
    'soil_temperature': ['Soil Temperature',
                         '°C', '°F', '°C', '°C', '°C', 'mdi:thermometer'],
    'wind_speed': ['Wind Speed', 'km/h', 'mph', 'm/s', 'mph', 'mph',
                   'mdi:weather-windy']
}

PLATFORM_SCHEMA = PLATFORM_SCHEMA.extend({
    vol.Required(CONF_MONITORED_CONDITIONS):
        vol.All(cv.ensure_list, [vol.In(SENSOR_TYPES)]),
    vol.Required(CONF_HOST): cv.string,
    vol.Optional(CONF_NAME, default=DEFAULT_NAME): cv.string,
    vol.Optional(CONF_UNITS): vol.In(['auto', 'si', 'us', 'ca', 'uk', 'uk2']),
})


CONDITION_CLASSES = {
    'cloudy': [],
    'fog': [],
    'hail': [],
    'lightning': [],
    'lightning-rainy': [],
    'partlycloudy': [],
    'pouring': [],
    'rainy': ['shower rain'],
    'snowy': [],
    'snowy-rainy': [],
    'sunny': ['sunshine'],
    'windy': [],
    'windy-variant': [],
    'exceptional': [],
}


def setup_platform(hass, config, add_devices, discovery_info=None):
    """Setup the Arduino Weatherstation."""

    if CONF_UNITS in config:
        units = config[CONF_UNITS]
    elif hass.config.units.is_metric:
        units = 'si'
    else:
        units = 'us'

    name = config.get(CONF_NAME)

    data = ArduinoWeatherData(hass, config.get(CONF_HOST))
    sensors = []
    for variable in config[CONF_MONITORED_CONDITIONS]:
        sensors.append(ArduinoWeatherSensor(data, variable))

    try:
        data.update()
    except ValueError as err:
        _LOGGER.error("Received error from Weatherstation: %s", err)
        return False

    add_devices(sensors, True)
    return True


class ArduinoWeatherSensor(Entity):
    """Representation of a weather condition."""
    #das muss noch

    def __init__(self, data, condition):
        """Initialize the weather condition."""
        self._condition = condition
        self.data = data
        self._state = None

    @property
    def name(self):
        """Return the name of the sensor."""
        return "PWS_" + self._condition

    @property
    def state(self):
        """Return the state."""
        return self._state

    @property
    def unit_of_measurement(self):
        """Return the unit of measurement."""
        try:
            return SENSOR_TYPES[self._condition][1]
        except Exception as err:
            _LOGGER.error("Received error from Weatherstation: %s", err)
            return None

    def update(self):
        """Update conditions"""
        self.data.update()
        self._state = self.data.data[self._condition]

class ArduinoWeatherData(object):
    """Get data from the Weatherstation"""

    def __init__(self, hass, resource):
        """Initialize the data object"""
        self._resource = resource
        self.data = []


    def update(self):
        """Get the latest data from the Weatherstation"""
        try:
            url = ("http://"+self._resource)
            result = requests.get(url,
                                  timeout=10).json()
            #_LOGGER.info(result)
            self.data = result["variables"]

        except requests.exceptions.RequestException:
            _LOGGER.error("Error fetching data from: %s", self._resource)
            self.data = None
            raise
