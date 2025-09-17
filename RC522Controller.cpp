#include "RC522Controller.h"

RC522Controller::RC522Controller() : FlexibleI2C(),
    rc522_bus_id(255), rc522_address(0x28), rc522_initialized(false),
    card_present(false), last_scan_time(0), scan_interval_ms(100) {
}

RC522Controller::~RC522Controller() {
}

bool RC522Controller::initRC522(uint8_t bus_id, uint8_t device_address) {
    if (!isBusInitialized(bus_id)) {
        return false;
    }

    rc522_bus_id = bus_id;
    rc522_address = device_address;

    if (!isDevicePresent(bus_id, device_address)) {
        return false;
    }

    if (!softReset()) {
        return false;
    }

    delay(50);

    uint8_t version = getVersion();
    if (version == 0x00 || version == 0xFF) {
        return false;
    }

    writeRegister(RC522_REG_T_MODE, 0x8D);
    writeRegister(RC522_REG_T_PRESCALER, 0x3E);
    writeRegister(RC522_REG_T_RELOAD_L, 30);
    writeRegister(RC522_REG_T_RELOAD_H, 0);

    writeRegister(RC522_REG_TX_ASK, 0x40);
    writeRegister(RC522_REG_MODE, 0x3D);

    enableAntenna(true);

    rc522_initialized = true;
    return true;
}

bool RC522Controller::isCardPresent() {
    if (!rc522_initialized) {
        return false;
    }

    if (millis() - last_scan_time < scan_interval_ms) {
        return card_present;
    }

    last_scan_time = millis();

    uint8_t buffer_atqa[2];
    uint8_t buffer_size = sizeof(buffer_atqa);

    RC522Status result = piccRequest(PICC_CMD_REQA, buffer_atqa, &buffer_size);
    bool new_card_present = (result == STATUS_OK);

    if (new_card_present != card_present) {
        if (new_card_present) {
            CardInfo card = readCard();
            if (card.valid) {
                last_card = card;
                onCardPresent(card);
            }
        } else {
            if (last_card.valid) {
                onCardRemoved(last_card);
            }
        }
    }

    card_present = new_card_present;
    return card_present;
}

CardInfo RC522Controller::readCard() {
    CardInfo card;

    if (!rc522_initialized) {
        return card;
    }

    uint8_t buffer_atqa[2];
    uint8_t buffer_size = sizeof(buffer_atqa);

    RC522Status result = piccRequest(PICC_CMD_REQA, buffer_atqa, &buffer_size);
    if (result != STATUS_OK) {
        return card;
    }

    result = piccAnticoll(card.uid, &card.uid_size);
    if (result != STATUS_OK) {
        return card;
    }

    result = piccSelect(card.uid, card.uid_size, &card.sak);
    if (result != STATUS_OK) {
        return card;
    }

    card.valid = true;
    card.card_type = getCardType(card.sak);
    card.last_seen = millis();

    return card;
}

String RC522Controller::getCardUID(const CardInfo& card) {
    if (!card.valid) {
        return "Invalid";
    }

    String uid = "";
    for (uint8_t i = 0; i < card.uid_size; i++) {
        if (uid.length() > 0) uid += ":";
        if (card.uid[i] < 0x10) uid += "0";
        uid += String(card.uid[i], HEX);
    }
    uid.toUpperCase();
    return uid;
}

String RC522Controller::getCardType(uint8_t sak) {
    switch (sak) {
        case 0x04: return "MIFARE Classic 1K";
        case 0x02: return "MIFARE Classic 1K";
        case 0x08: return "MIFARE Classic 1K";
        case 0x18: return "MIFARE Classic 4K";
        case 0x00: return "MIFARE Ultralight";
        case 0x10:
        case 0x11: return "MIFARE Plus";
        case 0x01: return "MIFARE TNP3xxx";
        case 0x20: return "ISO14443-4";
        default: return "Unknown (SAK: 0x" + String(sak, HEX) + ")";
    }
}

bool RC522Controller::softReset() {
    if (!rc522_initialized && rc522_bus_id == 255) {
        return false;
    }

    writeRegister(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET);

    int timeout = 50;
    while (timeout-- > 0) {
        uint8_t command_reg = readRegister(RC522_REG_COMMAND);
        if (!(command_reg & 0x10)) {
            break;
        }
        delay(1);
    }

    return (timeout > 0);
}

bool RC522Controller::selfTest() {
    if (!rc522_initialized) {
        return false;
    }

    if (!softReset()) {
        return false;
    }

    flushFIFO();
    writeRegister(RC522_REG_COMMAND, RC522_CMD_MEM);

    uint8_t test_data[25] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00
    };

    for (uint8_t i = 0; i < 25; i++) {
        writeRegister(RC522_REG_FIFO_DATA, test_data[i]);
    }

    writeRegister(RC522_REG_COMMAND, RC522_CMD_CALC_CRC);

    if (!waitForCommand()) {
        return false;
    }

    uint8_t result_low = readRegister(RC522_REG_CRC_RESULT_L);
    uint8_t result_high = readRegister(RC522_REG_CRC_RESULT_M);

    return (result_low == 0x63 && result_high == 0x63);
}

uint8_t RC522Controller::getVersion() {
    if (rc522_bus_id == 255) {
        return 0x00;
    }
    return readRegister(RC522_REG_VERSION);
}

bool RC522Controller::setAntennaGain(uint8_t gain) {
    if (!rc522_initialized) {
        return false;
    }

    if (gain > 7) {
        gain = 7;
    }

    uint8_t rf_cfg = readRegister(RC522_REG_RF_CFG);
    rf_cfg = (rf_cfg & 0x8F) | ((gain & 0x07) << 4);
    writeRegister(RC522_REG_RF_CFG, rf_cfg);

    return true;
}

bool RC522Controller::enableAntenna(bool enable) {
    if (!rc522_initialized && rc522_bus_id == 255) {
        return false;
    }

    if (enable) {
        setBitMask(RC522_REG_TX_CONTROL, 0x03);
    } else {
        clearBitMask(RC522_REG_TX_CONTROL, 0x03);
    }

    return true;
}

RC522Status RC522Controller::communicateWithPICC(uint8_t command, uint8_t* send_data, uint8_t send_len,
                                                 uint8_t* back_data, uint8_t* back_len,
                                                 uint8_t* valid_bits, uint8_t rx_align) {
    if (!rc522_initialized) {
        return STATUS_ERROR;
    }

    uint8_t wait_irq = 0x00;
    uint8_t command_value = 0x00;
    uint8_t irq_en = 0x00;

    switch (command) {
        case RC522_CMD_MF_AUTHENT:
            irq_en = 0x12;
            wait_irq = 0x10;
            break;
        case RC522_CMD_TRANSCEIVE:
            irq_en = 0x77;
            wait_irq = 0x30;
            break;
        default:
            return STATUS_INVALID;
    }

    writeRegister(RC522_REG_COMIEN, irq_en | 0x80);
    clearBitMask(RC522_REG_COMIRQ, 0x80);
    setBitMask(RC522_REG_FIFO_LEVEL, 0x80);

    writeRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);

    for (uint8_t i = 0; i < send_len; i++) {
        writeRegister(RC522_REG_FIFO_DATA, send_data[i]);
    }

    writeRegister(RC522_REG_COMMAND, command);
    if (command == RC522_CMD_TRANSCEIVE) {
        setBitMask(RC522_REG_BIT_FRAMING, 0x80);
    }

    uint16_t timeout = 2000;
    uint8_t n;
    do {
        n = readRegister(RC522_REG_COMIRQ);
        timeout--;
    } while (timeout != 0 && !(n & 0x01) && !(n & wait_irq));

    clearBitMask(RC522_REG_BIT_FRAMING, 0x80);

    if (timeout == 0) {
        return STATUS_TIMEOUT;
    }

    uint8_t error_reg = readRegister(RC522_REG_ERROR);
    if (error_reg & 0x13) {
        return STATUS_ERROR;
    }

    RC522Status result = STATUS_OK;

    if (n & irq_en & 0x01) {
        result = STATUS_TIMEOUT;
    } else if (command == RC522_CMD_TRANSCEIVE) {
        n = readRegister(RC522_REG_FIFO_LEVEL);
        uint8_t last_bits = readRegister(RC522_REG_CONTROL) & 0x07;

        if (last_bits) {
            *back_len = (n - 1) * 8 + last_bits;
        } else {
            *back_len = n * 8;
        }

        if (n == 0) {
            n = 1;
        }
        if (n > 16) {
            n = 16;
        }

        for (uint8_t i = 0; i < n; i++) {
            back_data[i] = readRegister(RC522_REG_FIFO_DATA);
        }

        if (valid_bits) {
            *valid_bits = last_bits;
        }
    }

    return result;
}

RC522Status RC522Controller::piccRequest(uint8_t req_code, uint8_t* back_data, uint8_t* back_bits) {
    writeRegister(RC522_REG_BIT_FRAMING, 0x07);

    uint8_t tx_data = req_code;
    RC522Status result = communicateWithPICC(RC522_CMD_TRANSCEIVE, &tx_data, 1, back_data, back_bits);

    if (*back_bits != 0x10) {
        result = STATUS_ERROR;
    }

    return result;
}

RC522Status RC522Controller::piccSelect(uint8_t* uid, uint8_t uid_size, uint8_t* sak) {
    bool uid_complete = false;
    bool select_done = false;
    bool use_cascade_tag;
    uint8_t cascade_level = 1;
    RC522Status result;
    uint8_t count;
    uint8_t check_bit;
    uint8_t index;
    uint8_t uid_index;
    uint8_t current_level_known_bits;
    uint8_t buffer[9];
    uint8_t buffer_used;
    uint8_t rx_align;
    uint8_t tx_last_bits;
    uint8_t* response_buffer;
    uint8_t response_length;

    if (uid_size > 4) {
        return STATUS_NO_ROOM;
    }

    clearBitMask(RC522_REG_COLL, 0x80);

    uid_index = 0;
    while (!uid_complete) {
        switch (cascade_level) {
            case 1: buffer[0] = PICC_CMD_SEL_CL1; break;
            case 2: buffer[0] = PICC_CMD_SEL_CL2; break;
            case 3: buffer[0] = PICC_CMD_SEL_CL3; break;
            default: return STATUS_INTERNAL_ERROR;
        }

        current_level_known_bits = 0;
        buffer[1] = 0x20;

        use_cascade_tag = (uid_index + 4) < uid_size;
        if (use_cascade_tag) {
            buffer[2] = PICC_CMD_CT;
            memcpy(&buffer[3], &uid[uid_index], 3);
            current_level_known_bits = 32;
        } else {
            memcpy(&buffer[2], &uid[uid_index], uid_size - uid_index);
            current_level_known_bits = (uid_size - uid_index) * 8;
        }

        if (current_level_known_bits >= 32) {
            buffer[1] = 0x70;
            buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];

            if (!calculateCRC(buffer, 7, &buffer[7])) {
                return STATUS_CRC_WRONG;
            }

            buffer_used = 9;
            tx_last_bits = 0;
            response_buffer = &buffer[6];
            response_length = 3;
        } else {
            tx_last_bits = current_level_known_bits % 8;
            count = current_level_known_bits / 8;
            index = 2 + count;
            buffer[1] = (index << 4) + tx_last_bits;
            buffer_used = index + (tx_last_bits ? 1 : 0);
            response_buffer = &buffer[index];
            response_length = sizeof(buffer) - index;
        }

        rx_align = tx_last_bits;
        writeRegister(RC522_REG_BIT_FRAMING, (rx_align << 4) + tx_last_bits);

        result = communicateWithPICC(RC522_CMD_TRANSCEIVE, buffer, buffer_used, response_buffer, &response_length, &tx_last_bits, rx_align);
        if (result == STATUS_COLLISION) {
            uint8_t coll_reg = readRegister(RC522_REG_COLL);
            if (coll_reg & 0x20) {
                return STATUS_COLLISION;
            }
            uint8_t collision_pos = coll_reg & 0x1F;
            if (collision_pos == 0) {
                collision_pos = 32;
            }
            if (collision_pos <= current_level_known_bits) {
                return STATUS_INTERNAL_ERROR;
            }
            // Handle collision - simplified for this implementation
            return STATUS_COLLISION;
        } else if (result != STATUS_OK) {
            return result;
        } else {
            if (current_level_known_bits >= 32) {
                select_done = true;
            } else {
                return STATUS_INTERNAL_ERROR;
            }
        }

        if (select_done) {
            if (response_length != 3 || tx_last_bits != 0) {
                return STATUS_ERROR;
            }

            if (!calculateCRC(response_buffer, 1, &buffer[2])) {
                return STATUS_CRC_WRONG;
            }
            if ((buffer[2] != response_buffer[1]) || (buffer[3] != response_buffer[2])) {
                return STATUS_CRC_WRONG;
            }

            if (sak) {
                *sak = response_buffer[0];
            }

            if (response_buffer[0] & 0x04) {
                cascade_level++;
                uid_index += 3;
            } else {
                uid_complete = true;
                if (use_cascade_tag) {
                    memcpy(&uid[uid_index], buffer, 3);
                    uid_index += 3;
                }
            }
        }
    }

    return STATUS_OK;
}

RC522Status RC522Controller::piccAnticoll(uint8_t* uid, uint8_t* uid_size) {
    uint8_t cascade_level = 1;
    RC522Status result;
    uint8_t count;
    uint8_t index;
    uint8_t uid_index;
    bool uid_complete = false;
    bool select_done = false;
    bool use_cascade_tag;
    uint8_t current_level_known_bits;
    uint8_t buffer[9];
    uint8_t buffer_used;
    uint8_t tx_last_bits;
    uint8_t* response_buffer;
    uint8_t response_length;

    clearBitMask(RC522_REG_COLL, 0x80);

    uid_index = 0;
    while (!uid_complete) {
        switch (cascade_level) {
            case 1: buffer[0] = PICC_CMD_SEL_CL1; break;
            case 2: buffer[0] = PICC_CMD_SEL_CL2; break;
            case 3: buffer[0] = PICC_CMD_SEL_CL3; break;
            default: return STATUS_INTERNAL_ERROR;
        }

        current_level_known_bits = 0;
        buffer[1] = 0x20;
        buffer_used = 2;
        tx_last_bits = 0;
        response_buffer = &buffer[2];
        response_length = sizeof(buffer) - 2;

        writeRegister(RC522_REG_BIT_FRAMING, 0);

        result = communicateWithPICC(RC522_CMD_TRANSCEIVE, buffer, buffer_used, response_buffer, &response_length, &tx_last_bits);
        if (result != STATUS_OK) {
            return result;
        }

        count = response_length / 8;
        if (count >= 5) {
            count = 4;
        }

        use_cascade_tag = (response_buffer[0] == PICC_CMD_CT);
        if (use_cascade_tag) {
            memcpy(&uid[uid_index], &response_buffer[1], count - 1);
            uid_index += count - 1;
        } else {
            memcpy(&uid[uid_index], response_buffer, count);
            uid_index += count;
        }

        if (response_buffer[0] & 0x04) {
            cascade_level++;
        } else {
            uid_complete = true;
        }
    }

    *uid_size = uid_index;
    return STATUS_OK;
}

RC522Status RC522Controller::mifareAuth(uint8_t command, uint8_t block_addr, uint8_t* key, CardInfo& card) {
    if (!rc522_initialized || !card.valid) {
        return STATUS_ERROR;
    }

    uint8_t send_data[12];
    send_data[0] = command;
    send_data[1] = block_addr;
    for (uint8_t i = 0; i < 6; i++) {
        send_data[2 + i] = key[i];
    }
    for (uint8_t i = 0; i < card.uid_size; i++) {
        send_data[8 + i] = card.uid[i];
    }

    RC522Status result = communicateWithPICC(RC522_CMD_MF_AUTHENT, send_data, 12, nullptr, 0);

    if (result != STATUS_OK) {
        return result;
    }

    uint8_t status2 = readRegister(RC522_REG_STATUS2);
    if (!(status2 & 0x08)) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

RC522Status RC522Controller::mifareRead(uint8_t block_addr, uint8_t* buffer) {
    if (!rc522_initialized) {
        return STATUS_ERROR;
    }

    uint8_t cmd_buffer[4];
    cmd_buffer[0] = PICC_CMD_MF_READ;
    cmd_buffer[1] = block_addr;

    if (!calculateCRC(cmd_buffer, 2, &cmd_buffer[2])) {
        return STATUS_CRC_WRONG;
    }

    uint8_t response_length = 18;
    RC522Status result = communicateWithPICC(RC522_CMD_TRANSCEIVE, cmd_buffer, 4, buffer, &response_length);

    if (result != STATUS_OK) {
        return result;
    }

    if (response_length != 16 * 8) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

RC522Status RC522Controller::mifareWrite(uint8_t block_addr, uint8_t* data) {
    if (!rc522_initialized) {
        return STATUS_ERROR;
    }

    uint8_t cmd_buffer[4];
    cmd_buffer[0] = PICC_CMD_MF_WRITE;
    cmd_buffer[1] = block_addr;

    if (!calculateCRC(cmd_buffer, 2, &cmd_buffer[2])) {
        return STATUS_CRC_WRONG;
    }

    uint8_t response[2];
    uint8_t response_length = 2;
    RC522Status result = communicateWithPICC(RC522_CMD_TRANSCEIVE, cmd_buffer, 4, response, &response_length);

    if (result != STATUS_OK || response_length != 4 || (response[0] & 0x0F) != 0x0A) {
        return STATUS_MIFARE_NACK;
    }

    uint8_t data_buffer[18];
    memcpy(data_buffer, data, 16);
    if (!calculateCRC(data_buffer, 16, &data_buffer[16])) {
        return STATUS_CRC_WRONG;
    }

    result = communicateWithPICC(RC522_CMD_TRANSCEIVE, data_buffer, 18, response, &response_length);

    if (result != STATUS_OK || response_length != 4 || (response[0] & 0x0F) != 0x0A) {
        return STATUS_MIFARE_NACK;
    }

    return STATUS_OK;
}

void RC522Controller::mifareStopCrypto() {
    clearBitMask(RC522_REG_STATUS2, 0x08);
}

bool RC522Controller::calculateCRC(uint8_t* data, uint8_t length, uint8_t* result) {
    if (!rc522_initialized) {
        return false;
    }

    writeRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
    writeRegister(RC522_REG_DIVIRQ, 0x04);
    setBitMask(RC522_REG_FIFO_LEVEL, 0x80);

    for (uint8_t i = 0; i < length; i++) {
        writeRegister(RC522_REG_FIFO_DATA, data[i]);
    }

    writeRegister(RC522_REG_COMMAND, RC522_CMD_CALC_CRC);

    uint16_t timeout = 5000;
    uint8_t n;
    do {
        n = readRegister(RC522_REG_DIVIRQ);
        timeout--;
    } while (timeout != 0 && !(n & 0x04));

    if (timeout == 0) {
        return false;
    }

    result[0] = readRegister(RC522_REG_CRC_RESULT_L);
    result[1] = readRegister(RC522_REG_CRC_RESULT_M);

    return true;
}

void RC522Controller::clearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t value = readRegister(reg);
    writeRegister(reg, value & (~mask));
}

void RC522Controller::setBitMask(uint8_t reg, uint8_t mask) {
    uint8_t value = readRegister(reg);
    writeRegister(reg, value | mask);
}

bool RC522Controller::writeRegister(uint8_t reg, uint8_t value) {
    return FlexibleI2C::writeRegister(rc522_bus_id, rc522_address, reg, value);
}

bool RC522Controller::writeRegister(uint8_t reg, uint8_t* values, uint8_t count) {
    return FlexibleI2C::writeBytes(rc522_bus_id, rc522_address, reg, values, count);
}

uint8_t RC522Controller::readRegister(uint8_t reg) {
    return FlexibleI2C::readRegister(rc522_bus_id, rc522_address, reg);
}

bool RC522Controller::readRegister(uint8_t reg, uint8_t* values, uint8_t count) {
    return FlexibleI2C::readBytes(rc522_bus_id, rc522_address, reg, values, count);
}

bool RC522Controller::waitForCommand() {
    uint16_t timeout = 1000;
    uint8_t status;
    do {
        status = readRegister(RC522_REG_COMMAND);
        timeout--;
    } while (timeout != 0 && (status != RC522_CMD_IDLE));

    return (timeout > 0);
}

void RC522Controller::flushFIFO() {
    setBitMask(RC522_REG_FIFO_LEVEL, 0x80);
}

void RC522Controller::registerCustomEndpoints(FlexibleEndpoints& endpoints) {
    endpoints.setLibraryName("RC522Controller");

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/initRC522")
        .summary("Initialize RC522 RFID reader")
        .description("Initialize RC522 on specified I2C bus and address")
        .params({
            REQUIRED_INT_PARAM("bus_id", "I2C Bus ID"),
            INT_PARAM("address", "I2C device address (default 0x28)")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleInitRC522(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/readRFIDCard")
        .summary("Read RFID card")
        .description("Read information from present RFID card")
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleReadCard(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/getRFIDCardStatus")
        .summary("Get card presence status")
        .description("Check if a card is currently present")
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleCardStatus(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/getRC522Status")
        .summary("Get RC522 status")
        .description("Get RC522 module status and version information")
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleRC522Status(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/controlRC522Antenna")
        .summary("Control antenna")
        .description("Enable or disable RC522 antenna")
        .params({
            EndpointParam("enable", "Enable antenna (true/false)", "boolean", true)
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleAntennaControl(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/readMIFAREBlock")
        .summary("Read MIFARE block")
        .description("Read data from MIFARE Classic block")
        .params({
            REQUIRED_INT_PARAM("block", "Block number to read"),
            REQUIRED_STR_PARAM("key", "Authentication key (hex, e.g., 'FFFFFFFFFFFF')")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleMifareRead(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/writeMIFAREBlock")
        .summary("Write MIFARE block")
        .description("Write data to MIFARE Classic block")
        .params({
            REQUIRED_INT_PARAM("block", "Block number to write"),
            REQUIRED_STR_PARAM("key", "Authentication key (hex)"),
            REQUIRED_STR_PARAM("data", "32 hex characters representing 16 bytes")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleMifareWrite(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/testRC522")
        .summary("Perform self test")
        .description("Run RC522 internal self test")
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleSelfTest(params);
        })
    );
}

void RC522Controller::onDeviceFound(uint8_t bus_id, uint8_t address) {
    for (auto& device : known_devices) {
        if (device.bus_id == bus_id && device.address == address) {
            if (address == 0x28) {
                device.device_name = "RC522 RFID Reader";
            }
            break;
        }
    }
}

std::pair<String, int> RC522Controller::handleInitRC522(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (params.find("bus_id") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing bus_id parameter";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t bus_id = params["bus_id"].toInt();
    uint8_t address = (params.find("address") != params.end()) ?
        strtol(params["address"].c_str(), NULL, 16) : 0x28;

    bool success = initRC522(bus_id, address);

    response["success"] = success;
    response["bus_id"] = bus_id;
    response["address"] = "0x" + String(address, HEX);

    if (success) {
        response["version"] = "0x" + String(getVersion(), HEX);
        response["initialized"] = true;
    } else {
        response["error"] = "Failed to initialize RC522";
    }

    String output;
    serializeJson(response, output);
    return {output, success ? 200 : 500};
}

std::pair<String, int> RC522Controller::handleReadCard(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (!rc522_initialized) {
        response["success"] = false;
        response["error"] = "RC522 not initialized";
        String output;
        serializeJson(response, output);
        return {output, 500};
    }

    CardInfo card = readCard();

    response["success"] = card.valid;
    if (card.valid) {
        response["card"] = cardInfoToJson(card);
    } else {
        response["error"] = "No card found or read failed";
    }

    String output;
    serializeJson(response, output);
    return {output, card.valid ? 200 : 404};
}

std::pair<String, int> RC522Controller::handleCardStatus(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (!rc522_initialized) {
        response["success"] = false;
        response["error"] = "RC522 not initialized";
        String output;
        serializeJson(response, output);
        return {output, 500};
    }

    bool present = isCardPresent();

    response["success"] = true;
    response["card_present"] = present;

    if (present && last_card.valid) {
        response["last_card"] = cardInfoToJson(last_card);
    }

    String output;
    serializeJson(response, output);
    return {output, 200};
}

std::pair<String, int> RC522Controller::handleRC522Status(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    response["success"] = true;
    response["initialized"] = rc522_initialized;
    response["bus_id"] = rc522_bus_id;
    response["address"] = "0x" + String(rc522_address, HEX);

    if (rc522_initialized) {
        response["version"] = "0x" + String(getVersion(), HEX);
        response["card_present"] = isCardPresent();
    }

    String output;
    serializeJson(response, output);
    return {output, 200};
}

std::pair<String, int> RC522Controller::handleAntennaControl(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (!rc522_initialized) {
        response["success"] = false;
        response["error"] = "RC522 not initialized";
        String output;
        serializeJson(response, output);
        return {output, 500};
    }

    if (params.find("enable") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing enable parameter";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    bool enable = (params["enable"] == "true" || params["enable"] == "1");
    bool success = enableAntenna(enable);

    response["success"] = success;
    response["antenna_enabled"] = enable;

    String output;
    serializeJson(response, output);
    return {output, success ? 200 : 500};
}

std::pair<String, int> RC522Controller::handleMifareRead(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (!rc522_initialized) {
        response["success"] = false;
        response["error"] = "RC522 not initialized";
        String output;
        serializeJson(response, output);
        return {output, 500};
    }

    if (params.find("block") == params.end() || params.find("key") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing required parameters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t block = params["block"].toInt();
    String key_str = params["key"];

    if (key_str.length() != 12) {
        response["success"] = false;
        response["error"] = "Key must be 12 hex characters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t key[6];
    for (int i = 0; i < 6; i++) {
        String byte_str = key_str.substring(i*2, i*2 + 2);
        key[i] = strtol(byte_str.c_str(), NULL, 16);
    }

    CardInfo card = readCard();
    if (!card.valid) {
        response["success"] = false;
        response["error"] = "No card present";
        String output;
        serializeJson(response, output);
        return {output, 404};
    }

    RC522Status auth_result = mifareAuth(PICC_CMD_MF_AUTH_KEY_A, block, key, card);
    if (auth_result != STATUS_OK) {
        response["success"] = false;
        response["error"] = "Authentication failed: " + statusToString(auth_result);
        String output;
        serializeJson(response, output);
        return {output, 401};
    }

    uint8_t buffer[16];
    RC522Status read_result = mifareRead(block, buffer);

    response["success"] = (read_result == STATUS_OK);
    response["block"] = block;

    if (read_result == STATUS_OK) {
        JsonArray data_array = response["data"].to<JsonArray>();
        for (int i = 0; i < 16; i++) {
            data_array.add("0x" + String(buffer[i], HEX));
        }

        String hex_string = "";
        for (int i = 0; i < 16; i++) {
            if (buffer[i] < 0x10) hex_string += "0";
            hex_string += String(buffer[i], HEX);
        }
        response["hex_string"] = hex_string;
    } else {
        response["error"] = "Read failed: " + statusToString(read_result);
    }

    mifareStopCrypto();

    String output;
    serializeJson(response, output);
    return {output, (read_result == STATUS_OK) ? 200 : 500};
}

std::pair<String, int> RC522Controller::handleMifareWrite(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (!rc522_initialized) {
        response["success"] = false;
        response["error"] = "RC522 not initialized";
        String output;
        serializeJson(response, output);
        return {output, 500};
    }

    if (params.find("block") == params.end() || params.find("key") == params.end() || params.find("data") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing required parameters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t block = params["block"].toInt();
    String key_str = params["key"];
    String data_str = params["data"];

    if (key_str.length() != 12) {
        response["success"] = false;
        response["error"] = "Key must be 12 hex characters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    if (data_str.length() != 32) {
        response["success"] = false;
        response["error"] = "Data must be 32 hex characters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t key[6];
    for (int i = 0; i < 6; i++) {
        String byte_str = key_str.substring(i*2, i*2 + 2);
        key[i] = strtol(byte_str.c_str(), NULL, 16);
    }

    uint8_t data[16];
    for (int i = 0; i < 16; i++) {
        String byte_str = data_str.substring(i*2, i*2 + 2);
        data[i] = strtol(byte_str.c_str(), NULL, 16);
    }

    CardInfo card = readCard();
    if (!card.valid) {
        response["success"] = false;
        response["error"] = "No card present";
        String output;
        serializeJson(response, output);
        return {output, 404};
    }

    RC522Status auth_result = mifareAuth(PICC_CMD_MF_AUTH_KEY_A, block, key, card);
    if (auth_result != STATUS_OK) {
        response["success"] = false;
        response["error"] = "Authentication failed: " + statusToString(auth_result);
        String output;
        serializeJson(response, output);
        return {output, 401};
    }

    RC522Status write_result = mifareWrite(block, data);

    response["success"] = (write_result == STATUS_OK);
    response["block"] = block;

    if (write_result != STATUS_OK) {
        response["error"] = "Write failed: " + statusToString(write_result);
    }

    mifareStopCrypto();

    String output;
    serializeJson(response, output);
    return {output, (write_result == STATUS_OK) ? 200 : 500};
}

std::pair<String, int> RC522Controller::handleSelfTest(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (!rc522_initialized) {
        response["success"] = false;
        response["error"] = "RC522 not initialized";
        String output;
        serializeJson(response, output);
        return {output, 500};
    }

    bool test_passed = selfTest();

    response["success"] = true;
    response["self_test_passed"] = test_passed;
    response["version"] = "0x" + String(getVersion(), HEX);

    String output;
    serializeJson(response, output);
    return {output, 200};
}

DynamicJsonDocument RC522Controller::cardInfoToJson(const CardInfo& card) {
    DynamicJsonDocument doc(512);

    doc["valid"] = card.valid;
    doc["uid"] = getCardUID(card);
    doc["uid_size"] = card.uid_size;
    doc["sak"] = "0x" + String(card.sak, HEX);
    doc["card_type"] = card.card_type;
    doc["last_seen"] = card.last_seen;

    JsonArray uid_array = doc["uid_bytes"].to<JsonArray>();
    for (uint8_t i = 0; i < card.uid_size; i++) {
        uid_array.add("0x" + String(card.uid[i], HEX));
    }

    return doc;
}

String RC522Controller::statusToString(RC522Status status) {
    switch (status) {
        case STATUS_OK: return "OK";
        case STATUS_ERROR: return "Error";
        case STATUS_COLLISION: return "Collision";
        case STATUS_TIMEOUT: return "Timeout";
        case STATUS_NO_ROOM: return "No room";
        case STATUS_INTERNAL_ERROR: return "Internal error";
        case STATUS_INVALID: return "Invalid";
        case STATUS_CRC_WRONG: return "CRC wrong";
        case STATUS_MIFARE_NACK: return "MIFARE NACK";
        default: return "Unknown status";
    }
}