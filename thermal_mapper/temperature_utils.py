"""Hilfsfunktionen fuer Temperatur-Skalierung und Farb-Mapping."""

import numpy as np


def gray_to_celsius(gray_value, temp_min=0.0, temp_max=100.0):
    """Lineare Umrechnung 8-Bit Grauwert (0-255) in Grad Celsius."""
    gray = float(np.clip(gray_value, 0, 255))
    return temp_min + (gray / 255.0) * (temp_max - temp_min)


def grays_to_celsius(gray_array, temp_min=0.0, temp_max=100.0):
    """Vektorisierte Grauwert-zu-Celsius-Umrechnung."""
    gray = np.clip(gray_array.astype(np.float64), 0.0, 255.0)
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


def temperatures_to_rgb(temp_array, temp_min, temp_max):
    """Vektorisierte Jet-Colormap: Temperatur-Array -> (N,3) uint8 RGB."""
    if temp_max <= temp_min:
        n = len(temp_array)
        return np.full((n, 3), 128, dtype=np.uint8)

    t = np.clip(
        (temp_array.astype(np.float64) - temp_min) / (temp_max - temp_min),
        0.0, 1.0
    )
    r = np.zeros_like(t)
    g = np.zeros_like(t)
    b = np.zeros_like(t)

    m = t < 0.25
    g[m] = 4.0 * t[m]
    b[m] = 1.0

    m = (t >= 0.25) & (t < 0.5)
    g[m] = 1.0
    b[m] = 1.0 - 4.0 * (t[m] - 0.25)

    m = (t >= 0.5) & (t < 0.75)
    r[m] = 4.0 * (t[m] - 0.5)
    g[m] = 1.0

    m = t >= 0.75
    r[m] = 1.0
    g[m] = 1.0 - 4.0 * (t[m] - 0.75)

    rgb = np.stack([r, g, b], axis=1)
    return (np.clip(rgb, 0.0, 1.0) * 255.0).astype(np.uint8)
