#ifndef RC522_CONTROLLER_H
#define RC522_CONTROLLER_H

#include <FlexibleI2C.h>
#include <ArduinoJson.h>

// RC522 I2C Register Addresses
#define RC522_REG_COMMAND       0x01
#define RC522_REG_COMIEN        0x02
#define RC522_REG_DIVIEN        0x03
#define RC522_REG_COMIRQ        0x04
#define RC522_REG_DIVIRQ        0x05
#define RC522_REG_ERROR         0x06
#define RC522_REG_STATUS1       0x07
#define RC522_REG_STATUS2       0x08
#define RC522_REG_FIFO_DATA     0x09
#define RC522_REG_FIFO_LEVEL    0x0A
#define RC522_REG_WATER_LEVEL   0x0B
#define RC522_REG_CONTROL       0x0C
#define RC522_REG_BIT_FRAMING   0x0D
#define RC522_REG_COLL          0x0E
#define RC522_REG_MODE          0x11
#define RC522_REG_TX_MODE       0x12
#define RC522_REG_RX_MODE       0x13
#define RC522_REG_TX_CONTROL    0x14
#define RC522_REG_TX_ASK        0x15
#define RC522_REG_TX_SEL        0x16
#define RC522_REG_RX_SEL        0x17
#define RC522_REG_RX_THRESHOLD  0x18
#define RC522_REG_DEMOD         0x19
#define RC522_REG_MF_TX         0x1C
#define RC522_REG_MF_RX         0x1D
#define RC522_REG_SERIAL_SPEED  0x1F
#define RC522_REG_CRC_RESULT_M  0x21
#define RC522_REG_CRC_RESULT_L  0x22
#define RC522_REG_MOD_WIDTH     0x24
#define RC522_REG_RF_CFG        0x26
#define RC522_REG_GS_N          0x27
#define RC522_REG_CW_GS_P       0x28
#define RC522_REG_MOD_GS_P      0x29
#define RC522_REG_T_MODE        0x2A
#define RC522_REG_T_PRESCALER   0x2B
#define RC522_REG_T_RELOAD_H    0x2C
#define RC522_REG_T_RELOAD_L    0x2D
#define RC522_REG_T_COUNTER_VAL_H 0x2E
#define RC522_REG_T_COUNTER_VAL_L 0x2F
#define RC522_REG_VERSION       0x37

// RC522 Commands
#define RC522_CMD_IDLE          0x00
#define RC522_CMD_MEM           0x01
#define RC522_CMD_CALC_CRC      0x03
#define RC522_CMD_TRANSMIT      0x04
#define RC522_CMD_NO_CMD_CHG    0x07
#define RC522_CMD_RECEIVE       0x08
#define RC522_CMD_TRANSCEIVE    0x0C
#define RC522_CMD_MF_AUTHENT    0x0E
#define RC522_CMD_SOFT_RESET    0x0F

// PICC Commands
#define PICC_CMD_REQA           0x26
#define PICC_CMD_WUPA           0x52
#define PICC_CMD_CT             0x88
#define PICC_CMD_SEL_CL1        0x93
#define PICC_CMD_SEL_CL2        0x95
#define PICC_CMD_SEL_CL3        0x97
#define PICC_CMD_HLTA           0x50
#define PICC_CMD_MF_AUTH_KEY_A  0x60
#define PICC_CMD_MF_AUTH_KEY_B  0x61
#define PICC_CMD_MF_READ        0x30
#define PICC_CMD_MF_WRITE       0xA0

// Status codes
enum RC522Status {
    STATUS_OK = 0,
    STATUS_ERROR,
    STATUS_COLLISION,
    STATUS_TIMEOUT,
    STATUS_NO_ROOM,
    STATUS_INTERNAL_ERROR,
    STATUS_INVALID,
    STATUS_CRC_WRONG,
    STATUS_MIFARE_NACK = 0xFF
};

struct CardInfo {
    uint8_t uid[10];
    uint8_t uid_size;
    uint8_t sak;
    bool valid;
    String card_type;
    unsigned long last_seen;

    CardInfo() : uid_size(0), sak(0), valid(false), card_type("Unknown"), last_seen(0) {
        memset(uid, 0, sizeof(uid));
    }
};

class RC522Controller : public FlexibleI2C {
public:
    RC522Controller();
    virtual ~RC522Controller();

    // Initialize RC522 on specific I2C bus and address
    bool initRC522(uint8_t bus_id, uint8_t device_address = 0x28);

    // Card detection and management
    bool isCardPresent();
    CardInfo readCard();
    String getCardUID(const CardInfo& card);
    String getCardType(uint8_t sak);

    // Low-level RC522 operations
    bool softReset();
    bool selfTest();
    uint8_t getVersion();
    bool setAntennaGain(uint8_t gain);
    bool enableAntenna(bool enable = true);

    // PICC communication
    RC522Status communicateWithPICC(uint8_t command, uint8_t* send_data, uint8_t send_len,
                                   uint8_t* back_data, uint8_t* back_len,
                                   uint8_t* valid_bits = nullptr, uint8_t rx_align = 0);

    RC522Status piccRequest(uint8_t req_code, uint8_t* back_data, uint8_t* back_bits);
    RC522Status piccSelect(uint8_t* uid, uint8_t uid_size, uint8_t* sak = nullptr);
    RC522Status piccAnticoll(uint8_t* uid, uint8_t* uid_size);

    // MIFARE Classic operations
    RC522Status mifareAuth(uint8_t command, uint8_t block_addr, uint8_t* key, CardInfo& card);
    RC522Status mifareRead(uint8_t block_addr, uint8_t* buffer);
    RC522Status mifareWrite(uint8_t block_addr, uint8_t* data);
    void mifareStopCrypto();

    // Utility functions
    bool calculateCRC(uint8_t* data, uint8_t length, uint8_t* result);
    void clearBitMask(uint8_t reg, uint8_t mask);
    void setBitMask(uint8_t reg, uint8_t mask);

    // Virtual override for custom endpoints
    virtual void registerCustomEndpoints(FlexibleEndpoints& endpoints) override;

    // Device event callbacks
    virtual void onDeviceFound(uint8_t bus_id, uint8_t address) override;
    virtual void onCardPresent(const CardInfo& card) {}
    virtual void onCardRemoved(const CardInfo& card) {}

protected:
    uint8_t rc522_bus_id;
    uint8_t rc522_address;
    bool rc522_initialized;
    CardInfo last_card;
    bool card_present;
    unsigned long last_scan_time;
    uint16_t scan_interval_ms;

    // RC522 register operations
    bool writeRegister(uint8_t reg, uint8_t value);
    bool writeRegister(uint8_t reg, uint8_t* values, uint8_t count);
    uint8_t readRegister(uint8_t reg);
    bool readRegister(uint8_t reg, uint8_t* values, uint8_t count);

    // Internal helper methods
    bool waitForCommand();
    void flushFIFO();

    // Endpoint handlers
    std::pair<String, int> handleInitRC522(std::map<String, String>& params);
    std::pair<String, int> handleReadCard(std::map<String, String>& params);
    std::pair<String, int> handleCardStatus(std::map<String, String>& params);
    std::pair<String, int> handleRC522Status(std::map<String, String>& params);
    std::pair<String, int> handleAntennaControl(std::map<String, String>& params);
    std::pair<String, int> handleMifareRead(std::map<String, String>& params);
    std::pair<String, int> handleMifareWrite(std::map<String, String>& params);
    std::pair<String, int> handleSelfTest(std::map<String, String>& params);

    // Helper methods
    DynamicJsonDocument cardInfoToJson(const CardInfo& card);
    String statusToString(RC522Status status);
};

#endif // RC522_CONTROLLER_H