
import board
import busio
import time
import ndef

from adafruit_pn532.i2c import PN532_I2C

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize PN532
pn532 = PN532_I2C(i2c, debug=False)

ic, ver, rev, support = pn532.firmware_version

print("PN532 initialized")
print("Firmware:", ver, rev)

pn532.SAM_configuration()


def read_ndef_type2(uid):
    """Read publicly accessible Type 2 tag memory."""

    data = bytearray()

    # Type 2 tags store data in 4-byte pages.
    # Page 4 is normally the beginning of user memory.
    for page in range(4, 40):
        try:
            block = pn532.ntag2xx_read_block(page)

            if block is None:
                break

            data.extend(block)

        except Exception as e:
            print("Page read stopped:", e)
            break

    return bytes(data)


def decode_ndef(raw):
    """Decode NDEF records from tag memory."""

    # Find the NDEF TLV (0x03)
    try:
        start = raw.index(0x03)
    except ValueError:
        print("No NDEF message found.")
        return

    length = raw[start + 1]

    if length == 0xFF:
        length = int.from_bytes(raw[start + 2:start + 4], "big")
        start += 4
    else:
        start += 2

    message = raw[start:start + length]

    if not message:
        print("Empty NDEF message.")
        return

    try:
        records = list(ndef.message_decoder(message))

        for record in records:
            print("\nRecord type:", record.type)

            if record.type == "urn:nfc:wkt:T":
                print("Text:", record.text)

            elif record.type == "urn:nfc:wkt:U":
                print("URL:", record.uri)

            else:
                print("Data:", record)

    except Exception as e:
        print("NDEF decoding error:", e)


print("\nWaiting for NFC cards...")

last_uid = None

while True:

    uid = pn532.read_passive_target(timeout=0.5)

    if uid is not None:

        uid_hex = uid.hex()

        if uid_hex != last_uid:

            print("\n========== CARD DETECTED ==========")
            print("UID:", uid_hex)

            raw = read_ndef_type2(uid)

            print("\nRaw memory:")
            print(raw.hex(" "))

            print("\nNDEF contents:")
            decode_ndef(raw)

            print("\n===================================")

            last_uid = uid_hex

    else:
        last_uid = None

    time.sleep(0.1)
