#include "drivers/vision_i2c_client.h"

namespace balance_car::drivers
{
namespace
{
constexpr uint8_t kMagic = 0xA5;
constexpr uint8_t kVersion = 1;
constexpr uint8_t kPacketSize = 16;
constexpr uint8_t kFlagFound = 0x01;
constexpr uint8_t kFlagHeld = 0x02;
constexpr uint8_t kFlagTargetValid = 0x04;

#pragma pack(push, 1)
struct VisionPacket
{
  uint8_t magic;
  uint8_t version;
  uint16_t sequence;
  uint8_t flags;
  int16_t centerErrorPermille;
  int16_t targetErrorPermille;
  int16_t angleCdeg;
  uint8_t validRows;
  uint8_t missedFrames;
  uint8_t thresholdUsed;
  uint8_t reserved;
  uint8_t crc8;
};
#pragma pack(pop)

static_assert(sizeof(VisionPacket) == kPacketSize, "Vision packet must be 16 bytes");

uint8_t crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0;
  while (length--)
  {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                           : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}
} // namespace

VisionI2cClient::VisionI2cClient(TwoWire &wire, const config::VisionI2cPins &pins)
    : _wire(wire), _pins(pins)
{
}

bool VisionI2cClient::begin()
{
  _healthy = _wire.begin(_pins.sda, _pins.scl, kFrequencyHz);
  return _healthy;
}

bool VisionI2cClient::read(VisionSample &sample)
{
  VisionPacket packet = {};
  const size_t received = _wire.requestFrom(kAddress, static_cast<size_t>(sizeof(packet)), true);
  if (received != sizeof(packet))
  {
    while (_wire.available()) _wire.read();
    _healthy = false;
    return false;
  }

  uint8_t *raw = reinterpret_cast<uint8_t *>(&packet);
  for (size_t index = 0; index < sizeof(packet); ++index)
  {
    raw[index] = static_cast<uint8_t>(_wire.read());
  }
  if (packet.magic != kMagic || packet.version != kVersion ||
      packet.crc8 != crc8(raw, sizeof(packet) - 1))
  {
    _healthy = false;
    return false;
  }

  sample.sequence = packet.sequence;
  sample.found = (packet.flags & kFlagFound) != 0;
  sample.held = (packet.flags & kFlagHeld) != 0;
  sample.targetValid = (packet.flags & kFlagTargetValid) != 0;
  sample.centerErrorPermille = packet.centerErrorPermille;
  sample.targetErrorPermille = packet.targetErrorPermille;
  sample.angleCdeg = packet.angleCdeg;
  sample.validRows = packet.validRows;
  sample.missedFrames = packet.missedFrames;
  sample.thresholdUsed = packet.thresholdUsed;
  _healthy = true;
  return true;
}

bool VisionI2cClient::isHealthy() const
{
  return _healthy;
}
} // namespace balance_car::drivers
