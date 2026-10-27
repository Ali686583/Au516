"""Protocole de telemetrie de diagnostic des encodeurs AU516.

Une trame ASCII par ligne : ENC,<compteur_M1>,<compteur_M3>\n
Les compteurs proviennent des entiers signes 32 bits du driver moteur.
"""


def parse_encoder_line(raw):
    """Retourne (M1, M3) ou None pour une trame non conforme."""
    try:
        text = raw.decode('ascii', errors='strict').strip()
        parts = text.split(',')
        if len(parts) != 3 or parts[0] != 'ENC':
            return None
        m1, m3 = int(parts[1]), int(parts[2])
        if not (-2**31 <= m1 < 2**31 and -2**31 <= m3 < 2**31):
            return None
        return m1, m3
    except (UnicodeDecodeError, ValueError, TypeError):
        return None
