/* V14 wire format only: 12-byte CCSDS/cFE-style header + ten float32.
 * Header is big endian; payload byte order is configured by the receiver.
 * There is no checksum in this firmware, so framing checks cannot detect every
 * corrupt packet. No text/CSV/legacy KEY=value records are accepted here.
 */
(function (root) {
  'use strict';
  const FIELDS = Object.freeze([
    'Temperature', 'Pressure', 'Roll', 'Pitch', 'Yaw', 'Voltage',
    'RelativeAltitude', 'Latitude', 'Longitude', 'RemainingEnergy'
  ]);
  class Decoder {
    constructor({ messageId = 0x0881, littleEndian = true } = {}) {
      if (!Number.isInteger(messageId) || messageId < 0 || messageId > 0xFFFF
          || (messageId & 0xF800) !== 0x0800) throw new RangeError('Invalid telemetry Message ID');
      this.messageId = messageId;
      this.littleEndian = !!littleEndian;
      this.buffer = new Uint8Array(52);
      this.reset();
    }
    reset() { this.used = 0; this.discarded = 0; this.packets = 0; }
    headerMatches() {
      return ((this.buffer[0] << 8) | this.buffer[1]) === this.messageId
        && (this.buffer[2] & 0xC0) === 0xC0
        && this.buffer[4] === 0 && this.buffer[5] === 45;
    }
    push(chunk) {
      if (!(chunk instanceof Uint8Array)) throw new TypeError('Expected UART bytes');
      const result = [];
      for (const byte of chunk) {
        this.buffer[this.used++] = byte;
        while (this.used >= 6 && !this.headerMatches()) {
          this.buffer.copyWithin(0, 1, this.used);
          this.used--; this.discarded++;
        }
        if (this.used !== 52) continue;
        const view = new DataView(this.buffer.buffer);
        const values = {};
        FIELDS.forEach((name, index) => {
          const value = view.getFloat32(12 + index * 4, this.littleEndian);
          values[name] = Number.isFinite(value) ? value : null;
        });
        result.push({
          messageId: view.getUint16(0, false),
          sequence: view.getUint16(2, false) & 0x3FFF,
          uptimeSeconds: view.getUint32(6, false) + view.getUint16(10, false) / 65536,
          values
        });
        this.used = 0; this.packets++;
      }
      return result;
    }
  }
  const api = Object.freeze({ FIELDS, Decoder, PACKET_BYTES: 52 });
  root.CfeTelemetry = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(globalThis);
