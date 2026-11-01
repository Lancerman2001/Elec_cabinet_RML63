"""Save actual RGB renderer output as PNG without Pillow."""
import struct
import zlib
from pathlib import Path


def write_png(path, pixels):
    height,width,channels = pixels.shape
    if channels != 3: raise ValueError('Expected RGB')
    def chunk(kind, data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    scan = b''.join(b'\0'+row.tobytes() for row in pixels)
    result = b'\x89PNG\r\n\x1a\n'
    result += chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))
    result += chunk(b'IDAT',zlib.compress(scan,6))+chunk(b'IEND',b'')
    path = Path(path); path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(result)
