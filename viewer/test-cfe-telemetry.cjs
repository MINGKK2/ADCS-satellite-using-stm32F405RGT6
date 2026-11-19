const assert = require('node:assert/strict');
const fs = require('node:fs');
const {Decoder, FIELDS} = require('./cfe-telemetry.js');
function packet(sequence = 0, {big = false, mid = 0x0881, values} = {}) {
  const b = Buffer.alloc(52);
  b.writeUInt16BE(mid,0); b.writeUInt16BE(0xC000 | sequence,2);
  b.writeUInt16BE(45,4); b.writeUInt32BE(1,6); b.writeUInt16BE(32768,10);
  (values || [25.95,101325,1,-2,169,1.2345,125.5,21.028833,105.852,12.4])
    .forEach((v,i) => big ? b.writeFloatBE(v,12+4*i) : b.writeFloatLE(v,12+4*i));
  return b;
}
for(let split=0;split<=52;split++) {
  const d=new Decoder(), b=packet(37);
  const result=[...d.push(b.subarray(0,split)),...d.push(b.subarray(split))];
  assert.equal(result.length,1); assert.equal(result[0].sequence,37);
  assert.equal(result[0].uptimeSeconds,1.5);
  assert.equal(result[0].values.Pressure,101325);
  assert.equal(result[0].values.Roll,1); assert.equal(result[0].values.Pitch,-2);
  assert.ok(Math.abs(result[0].values.Voltage-1.2345)<1e-6);
  assert.ok(Math.abs(result[0].values.RemainingEnergy-12.4)<1e-5);
}
let d=new Decoder(); let rows=[];
for(const byte of packet()) rows.push(...d.push(Uint8Array.of(byte)));
assert.equal(rows.length,1);
d=new Decoder(); rows=d.push(Buffer.concat([packet(16383),packet(0),packet(1)]));
assert.deepEqual(rows.map(r=>r.sequence),[16383,0,1]);
const invalid=packet(); invalid[4]=1;
d=new Decoder(); rows=d.push(Buffer.concat([Buffer.from('SEQ=4 R=1000\r\n'),invalid,
  packet(1,{mid:0x0882}),packet(2)]));
assert.equal(rows.length,1); assert.equal(rows[0].sequence,2); assert.ok(d.discarded>0);
d=new Decoder(); d.push(Buffer.alloc(100000,0)); assert.ok(d.used<=5);
assert.equal(d.buffer.length,52); assert.equal(d.push(packet()).length,1);
d=new Decoder({messageId:0x0882,littleEndian:false});
rows=d.push(packet(2,{mid:0x0882,big:true})); assert.equal(rows[0].values.Yaw,169);
d=new Decoder(); rows=d.push(packet(0,{values:Array(10).fill(NaN)}));
assert.deepEqual(Object.values(rows[0].values),Array(10).fill(null));
d=new Decoder(); rows=d.push(packet(0,{values:[Infinity,-Infinity,0,0,0,0,0,0,0,0]}));
assert.equal(rows[0].values.Temperature,null); assert.equal(rows[0].values.Pressure,null);
assert.throws(()=>new Decoder({messageId:0x1881}),RangeError);
assert.throws(()=>new Decoder({messageId:0x10881}),RangeError);
assert.throws(()=>d.push('not bytes'),TypeError);
d.reset(); assert.equal(d.used,0); assert.equal(d.packets,0); assert.equal(d.discarded,0);
if(process.argv[2]) {
  const real=fs.readFileSync(process.argv[2]); const r=new Decoder().push(real);
  assert.equal(real.length,52); assert.equal(r.length,1);
  assert.ok(Math.abs(r[0].values.Temperature-25.95)<1e-5);
  assert.equal(r[0].values.Pressure,101325);
  assert.equal(r[0].values.Pitch,-2);
  assert.ok(Math.abs(r[0].values.Voltage-1.2345)<1e-6);
  assert.ok(Math.abs(r[0].values.RemainingEnergy-12.4)<1e-5);
  console.log('PASS packet emitted by actual firmware serializer');
}
assert.equal(FIELDS.length,10);
console.log('PASS binary decoder: every split, bytewise/coalesced packets, noise resync, wrong MID/length, bounded storage, endian modes, NaN/Infinity, sequence wrap');
module.exports={packet};
