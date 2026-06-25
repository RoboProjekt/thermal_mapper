"""Hilfsfunktionen fuer Temperatur-Skalierung und Farb-Mapping."""

import numpy as np


def gray_to_celsius(gray_value, temp_min=0.0, temp_max=100.0):
    """Lineare Umrechnung 8-Bit Grauwert (0-255) in Grad Celsius."""
    gray = float(np.clip(gray_value, 0, 255))
    return temp_min + (gray / 255.0) * (temp_max - temp_min)


def temperature_to_color(temp, temp_min, temp_max):
    """
    Mappt Temperatur auf RGB (Turbo-Approximation).
    Gibt (r, g, b) als float 0..1 zurueck.
    """
    if temp_max <= temp_min:
        return (0.5, 0.5, 0.5)

    t = float(np.clip((temp - temp_min) / (temp_max - temp_min), 0.0, 1.0))

    # Einfache Jet-Colormap (blau -> cyan -> gelb -> rot)
    if t < 0.25:
        r, g, b = 0.0, 4.0 * t, 1.0
    elif t < 0.5:
        r, g, b = 0.0, 1.0, 1.0 - 4.0 * (t - 0.25)
    elif t < 0.75:
        r, g, b = 4.0 * (t - 0.5), 1.0, 0.0
    else:
        r, g, b = 1.0, 1.0 - 4.0 * (t - 0.75), 0.0

    return (float(r), float(g), float(b))
