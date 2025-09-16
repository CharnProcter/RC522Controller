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

- `POST /rc522/init` - Initialize RC522 module
- `GET /rc522/read_card` - Read card information
- `GET /rc522/card_status` - Check card presence
- `GET /rc522/status` - Get module status
- `POST /rc522/antenna` - Control antenna
- `GET /rc522/mifare_read` - Read MIFARE block
- `POST /rc522/mifare_write` - Write MIFARE block
- `GET /rc522/self_test` - Perform self test

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
POST /rc522/init?bus_id=0&address=0x28
```

### Read Card
```
GET /rc522/read_card
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
GET /rc522/mifare_read?block=4&key=FFFFFFFFFFFF
```

## Hardware Connection

- VCC → 3.3V
- GND → GND
- SDA → GPIO 21 (configurable)
- SCL → GPIO 22 (configurable)
- RST → Not used in I2C mode

## License

MIT License