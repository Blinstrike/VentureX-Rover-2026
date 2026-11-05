
import board
import busio
import time

from adafruit_pn532.i2c import PN532_I2C

# -----------------------------
# Initialize PN532
# -----------------------------

i2c = busio.I2C(board.SCL, board.SDA)

pn532 = PN532_I2C(i2c, debug=False)

ic, ver, rev, support = pn532.firmware_version

print("================================")
print("       PN532 NFC READER")
print("================================")
print(f"Firmware: {ver}.{rev}")

pn532.SAM_configuration()

print("\nReader initialized!")
print("Waiting for Android NFC emulation...")


# -----------------------------
# APDU COMMANDS
# -----------------------------

SELECT_NDEF_APP = bytes.fromhex(
    "00 A4 04 00 07 D2 76 00 00 85 01 01 00"
)


# -----------------------------
# Response decoder
# -----------------------------

def decode_response(response):

    if not response:
        print("Empty response")
        return

    print("\n========== RESPONSE ==========")

    print("RAW HEX:")
    print(response.hex(" ").upper())

    if len(response) < 2:
        print("Response too short")
        return

    status = response[-2:].hex().upper()
    data = response[:-2]

    print("\nSTATUS WORD:", status)

    statuses = {
        "9000": "Success",
        "6A82": "Application or file not found",
        "6D00": "Unsupported instruction",
        "6A86": "Incorrect parameters",
        "6700": "Incorrect command length",
        "6982": "Security status not satisfied",
        "6985": "Conditions of use not satisfied",
    }

    print("MEANING:", statuses.get(status, "Unknown status"))

    if data:

        print("\nRESPONSE DATA HEX:")
        print(data.hex(" ").upper())

        print("\nASCII:")

        try:
            print(data.decode("ascii"))
        except UnicodeDecodeError:
            print("Binary data (not plain ASCII)")

    print("==============================")


# -----------------------------
# Main loop
# -----------------------------

last_uid = None

while True:

    try:

        uid = pn532.read_passive_target(timeout=1)

        if uid is None:
            last_uid = None
            continue

        uid_hex = uid.hex()

        if uid_hex == last_uid:
            continue

        last_uid = uid_hex

        print("\n\n################################")
        print("         CARD DETECTED")
        print("################################")

        print("UID:", uid_hex)

        print("\nSelecting NDEF application...")

        response = pn532.call_function(
            0x40,
            response_length=64,
            params=bytes([1]) + SELECT_NDEF_APP,
            timeout=5
        )

        decode_response(response)

        print("\nRemove and present the card again.")

    except KeyboardInterrupt:
        print("\nStopping reader...")
        break

    except Exception as e:
        print("ERROR:", e)
        time.sleep(1)
