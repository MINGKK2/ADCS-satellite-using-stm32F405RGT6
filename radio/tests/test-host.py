import importlib.util,struct,tempfile,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('s3_updater',Path(__file__).parents[1]/'s3_updater.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class Port:
    def __init__(self):self.writes=[];self.lines=[];self.lost=False
    def write(self,data):
        self.writes.append(data);_,session,seq,op,_=data.decode().strip().split()
        if self.lost and len(self.writes)==1:return
        self.lines=[b'SEQ=10 R=0 P=0 Y=0\r\n',f'@FWACK {int(session)+1} {seq} {op} 0 626164\r\n'.encode(),f'SEQ=11 R=0 PWM3=0,@FWACK {session} {seq} {op} 0 6F6B\r\n'.encode()]
    def flush(self):pass
    def readline(self):return self.lines.pop(0) if self.lines else b''
class HostTests(unittest.TestCase):
    def test_retry_uses_identical_id_and_bytes(self):
        gateway=object.__new__(module.Gateway);gateway.port=Port();gateway.session=42;gateway.seq=0;gateway.timeout=.002
        gateway.port.lost=True;self.assertEqual(gateway.rpc(3,b'\0\1\2'),'ok'.encode())
        self.assertEqual(len(gateway.port.writes),2);self.assertEqual(gateway.port.writes[0],gateway.port.writes[1]);self.assertEqual(gateway.seq,1)
    def test_image_guard(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'image.bin';p.write_bytes(struct.pack('<II',0x20020000,0x08000009)+b'1234')
            data,crc=module.load_image(p);self.assertEqual(len(data),12);self.assertIsInstance(crc,int)
            for payload in [b'',b'abc',struct.pack('<II',0x20020000,0x080E0001)+b'1234',struct.pack('<II',0x20020004,0x08000009)+b'1234']:
                p.write_bytes(payload)
                with self.assertRaises(ValueError):module.load_image(p)
if __name__=='__main__':unittest.main()
