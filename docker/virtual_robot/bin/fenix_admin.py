#!/usr/bin/env python3
"""Docenten-PIN voor de Beheer-pagina (Fenix).

Sinds 0.21.0 hebben ook de Beheer-acties zelf het token nodig
(status_page_server.py /api/admin/action): ze zijn geen ROS-services meer,
dus ROS op een studentenlaptop kan ze niet aanroepen. Wie een shell op de
robot heeft (docent: SSH/ttyd) kan uiteraard alles. De PIN staat nooit in de pagina of in git, enkel als
PBKDF2-hash in /root/turtlebot_state/admin_pin (host: ~/turtlebot_setup/state).

PIN instellen of wijzigen op een robot (zelfde PIN voor de hele vloot):
    docker exec -it turtlebot_<nr> fenix_admin.py set
of zonder vraag (bv. vanuit een script):
    docker exec turtlebot_<nr> fenix_admin.py set 4827
PIN wissen (Beheer dan dicht tot er een nieuwe is): fenix_admin.py clear
"""
import getpass
import hashlib
import hmac
import os
import secrets
import sys
import threading
import time

PIN_FILE = "/root/turtlebot_state/admin_pin"
ITERATIONS = 200000
SESSION_S = 12 * 3600          # ontgrendeld tot de browser sluit, max 12 u
MAX_FAILS = 5                  # daarna wachten: 30 s, 60 s, 120 s, ... (max 15 min)


def hash_pin(pin, salt=None, iterations=ITERATIONS):
    salt = salt or secrets.token_hex(16)
    h = hashlib.pbkdf2_hmac("sha256", pin.encode(), bytes.fromhex(salt), iterations).hex()
    return "pbkdf2_sha256:%d:%s:%s" % (iterations, salt, h)


def pin_set():
    try:
        return os.path.getsize(PIN_FILE) > 0
    except OSError:
        return False


def check_pin(pin):
    try:
        with open(PIN_FILE) as f:
            algo, it, salt, h = f.read().strip().split(":")
    except (OSError, ValueError):
        return False
    if algo != "pbkdf2_sha256":
        return False
    return hmac.compare_digest(hash_pin(pin, salt, int(it)).split(":")[3], h)


def valid_pin(pin):
    return pin.isdigit() and 4 <= len(pin) <= 8


def write_pin(pin):
    tmp = PIN_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(hash_pin(pin) + "\n")
    os.chmod(tmp, 0o600)
    os.replace(tmp, PIN_FILE)


class Sessions:
    """Tokens in memory (container restart = opnieuw ontgrendelen) and a
    lockout per client against guessing the PIN."""

    def __init__(self):
        self.lock = threading.Lock()
        self.tokens = {}       # token -> expiry
        self.fails = {}        # client ip -> (count, locked_until)

    def unlock(self, ip, pin):
        """-> (http status, dict)"""
        now = time.time()
        with self.lock:
            count, until = self.fails.get(ip, (0, 0))
            if now < until:
                return 429, {"error": "te veel foute pogingen - wacht %d s" % (until - now + 1)}
        if not pin_set():
            return 409, {"error": "nog geen PIN ingesteld op deze robot (zie Info > Beheer instellen)"}
        if not check_pin(str(pin)):
            with self.lock:
                count += 1
                wait = 0 if count < MAX_FAILS else min(900, 30 * 2 ** (count - MAX_FAILS))
                self.fails[ip] = (count, now + wait)
            return 401, {"error": "verkeerde PIN" + (" - wacht %d s" % wait if wait else "")}
        token = secrets.token_urlsafe(24)
        with self.lock:
            self.fails.pop(ip, None)
            self.tokens = {t: e for t, e in self.tokens.items() if e > now}
            self.tokens[token] = now + SESSION_S
        return 200, {"token": token}

    def valid(self, token):
        with self.lock:
            return bool(token) and self.tokens.get(token, 0) > time.time()

    def lock_token(self, token):
        with self.lock:
            self.tokens.pop(token, None)


def main():
    args = sys.argv[1:]
    if args[:1] == ["clear"]:
        try:
            os.remove(PIN_FILE)
        except OSError:
            pass
        print("PIN gewist - Beheer blijft dicht tot er een nieuwe PIN is")
        return 0
    if args[:1] == ["set"]:
        pin = args[1] if len(args) > 1 else getpass.getpass("Nieuwe docenten-PIN (4-8 cijfers): ")
        if len(args) == 1 and getpass.getpass("Nog eens: ") != pin:
            print("De PIN's verschillen - niets veranderd")
            return 1
        if not valid_pin(pin):
            print("PIN moet 4 tot 8 cijfers zijn - niets veranderd")
            return 1
        write_pin(pin)
        print("PIN ingesteld (%s)" % PIN_FILE)
        return 0
    if args[:1] == ["status"]:
        print("PIN ingesteld" if pin_set() else "geen PIN ingesteld")
        return 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
