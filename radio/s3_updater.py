"""Stage a checked image on C3, then flash STM32 through S3/ESP-NOW/UART.
No STM32 USB programmer, mass erase or option-byte operations are used.
"""
from pathlib import Path
import argparse,json,re,secrets,struct,sys,time,zlib,hashlib
try:import serial
except ImportError:
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/python-packages'))
    import serial

class Gateway:
    def __init__(self,port,timeout=3):
        self.port=serial.Serial(port=None,baudrate=115200,timeout=.25)
        self.port.dtr=False;self.port.rts=False;self.port.port=port;self.port.open()
        self.session=secrets.randbits(32) or 1;self.seq=0;self.timeout=timeout
    def close(self):self.port.close()
    def rpc(self,op,data=b'',timeout=None):
        assert len(data)<=195
        self.seq+=1
        request=f'@FW {self.session} {self.seq} {op} {data.hex() or "-"}\n'.encode('ascii')
        for attempt in range(3):
            self.port.write(request);self.port.flush();deadline=time.monotonic()+(timeout or self.timeout)
            while time.monotonic()<deadline:
                line=self.port.readline().decode('ascii','replace').strip()
                if not line:continue
                m=re.fullmatch(r'(@FWACK|@FWPROGRESS) (\d+) (\d+) (\d+) (\d+)(?: ([0-9A-Fa-f]*))?',line)
                if not m:continue
                kind,s,seq,received_op,status,body=m.groups()
                if (int(s),int(seq),int(received_op))!=(self.session,self.seq,op):continue
                body=bytes.fromhex(body or '')
                if kind=='@FWPROGRESS':
                    if len(body)==2:print(f'PHASE={body[0]} PROGRESS={body[1]}%',flush=True)
                    continue
                if int(status):raise RuntimeError(f'RPC {op} status={status}: {body.decode("ascii","replace")}')
                return body
            print(f'RPC_RETRY op={op} id={self.seq} attempt={attempt+1}',flush=True)
        raise TimeoutError(f'No matching C3 ACK for op={op}. Existing telemetry-only bridges do not support this updater.')

def load_image(path):
    data=Path(path).read_bytes()
    if len(data)<8 or len(data)>0xE0000 or len(data)%4:raise ValueError('Image length/range/alignment invalid')
    stack,reset=struct.unpack_from('<II',data)
    if not (0x20000000<stack<=0x20020000 and stack%8==0 and reset&1 and 0x08000000<=(reset&~1)<0x08000000+len(data)):
        raise ValueError('Invalid F405 vector table or image linked to wrong address')
    return data,zlib.crc32(data)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='COM23');parser.add_argument('--image',type=Path)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--ping',action='store_true');mode.add_argument('--stage-only',action='store_true');mode.add_argument('--flash-staged',action='store_true')
    parser.add_argument('--motor-power-off',action='store_true',help='Required physical condition before staging/flashing')
    parser.add_argument('--bootloader-ready',action='store_true',help='STM32 B0/Reset completed, motor power disconnected')
    parser.add_argument('--log-dir',type=Path,default=Path('wireless-deployment'))
    args=parser.parse_args()
    if not args.ping and (not args.image or not args.motor_power_off):parser.error('--image and --motor-power-off are required')
    if args.flash_staged and not args.bootloader_ready:parser.error('--bootloader-ready is required; no automatic BOOT0/NRST wiring is configured')
    image,crc=load_image(args.image) if args.image else (b'',0)
    gateway=Gateway(args.port)
    try:
        info=gateway.rpc(1).decode('ascii');print('GATEWAY',info,flush=True)
        if not info.startswith('C3_STM32_UPDATER_V1 '):raise RuntimeError('Unexpected bridge firmware')
        if args.ping:return
        args.log_dir.mkdir(parents=True,exist_ok=True)
        if args.stage_only:
            # User has disconnected motor power; request an additional explicit
            # software ESTOP while the application may still be running.
            gateway.port.write(b'ESTOP\r\n');gateway.port.flush();time.sleep(.2)
            gateway.rpc(2,struct.pack('<II',len(image),crc));print('STAGING bytes=',len(image),flush=True)
            for offset in range(0,len(image),160):
                chunk=image[offset:offset+160]
                result=gateway.rpc(3,struct.pack('<I',offset)+chunk)
                if result!=struct.pack('<I',offset+len(chunk)):raise RuntimeError('C3 stored offset mismatch')
                if offset%8000==0:print(f'STORED={offset+len(chunk)}/{len(image)}',flush=True)
            gateway.rpc(4);print('IMAGE_SEALED_CRC_VECTOR_OK',flush=True)
            print('NEXT: enter STM32 B0/Reset manually, release B0, then use --flash-staged.',flush=True)
            return
        if f'READY=1 SIZE={len(image)} CRC={crc:08X}' not in info:raise RuntimeError('C3 staged image does not match this file; stage again')
        print('ROM',gateway.rpc(5,timeout=8).decode('ascii'),flush=True)
        before=gateway.rpc(6,timeout=35)
        if len(before)!=32:raise RuntimeError('Calibration SHA-256 response length')
        print('CAL_BEFORE_SHA256',before.hex(),flush=True)
        result={'firmware_sha256':hashlib.sha256(image).hexdigest(),'firmware_bytes':len(image),'calibration_sha256_before':before.hex(),'transport':'USB S3 -> ESP-NOW -> C3 -> STM32 USART1 8E1','motor_test_performed':False}
        (args.log_dir/'before.json').write_text(json.dumps(result,indent=2)+'\n')
        after=gateway.rpc(7,timeout=120)
        if after!=before:raise RuntimeError('Calibration changed or image verify not confirmed; DO NOT GO')
        result.update(verified_readback=True,calibration_sha256_after=after.hex(),calibration_unchanged=True)
        print('PROGRAM_READBACK_AND_CALIBRATION_VERIFIED',flush=True)
        gateway.rpc(8,timeout=8);result['go_acknowledged']=True
        print('APPLICATION_GO_ACK',flush=True)
        deadline=time.monotonic()+15;frames=0;fw=False;last=''
        gateway.port.write(b'STATUS\r\n')
        while time.monotonic()<deadline:
            line=gateway.port.readline().decode('ascii','replace').strip()
            if line.startswith('SEQ='):frames+=1;last=line
            if 'FW=MANUAL_PID_V1' in line:fw=True
            if frames>=10 and fw:break
        result.update(fast_frames_after_go=frames,expected_firmware_seen=fw,last_fast_frame=last)
        result['recorded_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
        (args.log_dir/'deployment.json').write_text(json.dumps(result,indent=2)+'\n')
        print('TELEMETRY_FRAMES',frames,'EXPECTED_FW',fw,flush=True)
        if last:print(last,flush=True)
        if not frames or not fw:raise RuntimeError('Flash verified, but new application telemetry not confirmed; do not arm PID')
    finally:gateway.close()
if __name__=='__main__':main()
