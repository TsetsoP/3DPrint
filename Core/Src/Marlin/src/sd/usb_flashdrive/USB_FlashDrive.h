#pragma once
#include "../SdFatConfig.h"


class Sd2Card {
  public:
    bool init(const uint8_t sckRateID, const pin_t chipSelectPin);

    inline bool readStart(const uint32_t block)                  { pos = block; return ready(); }
    inline bool readData(uint8_t* dst)                           { return readBlock(pos++, dst); }
    inline bool readStop() const                                 { return true; }

    inline bool writeStart(const uint32_t block, const uint32_t) { pos = block; return ready(); }
    inline bool writeData(uint8_t* src)                          { return writeBlock(pos++, src); }
    inline bool writeStop() const                                { return true; }

    bool readBlock(uint32_t block, uint8_t* dst);
    bool writeBlock(uint32_t blockNumber, const uint8_t* src);

    uint32_t cardSize();
    static bool isInserted();
    static bool ready();

  private:
    uint32_t pos;
};
