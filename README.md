# RC522Controller

RC522 RFID controller library built on FlexibleI2C for ESP32, providing HTTP API endpoints for RFID operations.

## Features

- Full RC522 RFID module support via I2C
- MIFARE Classic card operations (read/write/authenticate)
- Card detection and identification
- HTTP API endpoints via FlexibleEndpoints integration
- Built on extensible FlexibleI2C architecture
- JSON responses for all operations
- Automatic card type detection
- Self-test capabilities

## Supported Cards

- MIFARE Classic 1K/4K
- MIFARE Ultralight
- MIFARE Plus
- ISO14443-4 compatible cards

## HTTP Endpoints

- `POST /initRC522` - Initialize RC522 module
- `GET /readRFIDCard` - Read card information
- `GET /getRFIDCardStatus` - Check card presence
- `GET /getRC522Status` - Get module status
- `POST /controlRC522Antenna` - Control antenna
- `GET /readMIFAREBlock` - Read MIFARE block
- `POST /writeMIFAREBlock` - Write MIFARE block
- `GET /testRC522` - Perform self test

## Usage

```cpp
#include <RC522Controller.h>
#include <FlexibleEndpoints.h>

RC522Controller rfid;

// Initialize with FlexibleEndpoints
rfid.init(endpoints);

// Initialize I2C bus first
rfid.initBus(0, 21, 22); // SDA=21, SCL=22

// Initialize RC522
rfid.initRC522(0, 0x28); // bus_id=0, address=0x28

// Check for cards
if (rfid.isCardPresent()) {
    CardInfo card = rfid.readCard();
    if (card.valid) {
        String uid = rfid.getCardUID(card);
        Serial.println("Card UID: " + uid);
    }
}
```

## MIFARE Operations

```cpp
// Default MIFARE key
uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Authenticate and read block
CardInfo card = rfid.readCard();
if (rfid.mifareAuth(PICC_CMD_MF_AUTH_KEY_A, 4, key, card) == STATUS_OK) {
    uint8_t buffer[16];
    if (rfid.mifareRead(4, buffer) == STATUS_OK) {
        // Process block data
    }
}

// Write block
uint8_t data[16] = {0x01, 0x02, 0x03, /*...*/ };
if (rfid.mifareWrite(4, data) == STATUS_OK) {
    Serial.println("Write successful");
}

// Always stop crypto after operations
rfid.mifareStopCrypto();
```

## HTTP API Examples

### Initialize RC522
```
POST /initRC522?bus_id=0&address=0x28
```

### Read Card
```
GET /readRFIDCard
```
Response:
```json
{
    "success": true,
    "card": {
        "uid": "04:52:4C:B2:5C:60:80",
        "card_type": "MIFARE Classic 1K",
        "sak": "0x08",
        "uid_size": 7
    }
}
```

### Read MIFARE Block
```
GET /readMIFAREBlock?block=4&key=FFFFFFFFFFFF
```

## Hardware Connection

- VCC → 3.3V
- GND → GND
- SDA → GPIO 21 (configurable)
- SCL → GPIO 22 (configurable)
- RST → Not used in I2C mode

## License

MIT License