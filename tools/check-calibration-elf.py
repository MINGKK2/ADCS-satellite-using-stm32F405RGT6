import struct,sys
from pathlib import Path
elf=Path(sys.argv[1]);data=elf.read_bytes()
assert data[:6]==b'\x7fELF\x01\x01','expected ELF32 little endian'
phoff=struct.unpack_from('<I',data,28)[0]
entry_size,count=struct.unpack_from('<HH',data,42)
maximum=0
for i in range(count):
    kind,offset,va,pa,size,memsize,flags,align=struct.unpack_from('<8I',data,phoff+i*entry_size)
    if kind==1 and size and 0x08000000<=pa<0x08100000:
        assert pa+size<=0x080E0000,'loadable firmware overlaps calibration journal'
        maximum=max(maximum,pa+size)
map_text=elf.with_suffix('.map').read_text(errors='replace')
assert '__calibration_flash_start__' in map_text and '__calibration_flash_end__' in map_text
print(f'PASS ELF load segments end at 0x{maximum:08X}; sector 11 reserved, no loadable calibration data')
