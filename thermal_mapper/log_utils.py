"""Zeitstempel-Hilfen fuer Logging und Debugging."""

from datetime import datetime


def ts():
    """Kompakter Zeitstempel fuer Konsolen-Ausgabe."""
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


def ts_iso():
    """ISO-aehnlicher Zeitstempel fuer CSV-Logs."""
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
