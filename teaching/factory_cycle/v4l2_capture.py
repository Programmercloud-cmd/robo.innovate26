"""MJPEG capture with the kernel's monotonic exposure/frame timestamps.

The ABI below mirrors /usr/include/linux/videodev2.h on this Linux workstation.
No camera inference or robot access occurs here.
"""
import ctypes as C
import errno
import fcntl
import mmap
import os
import select

import cv2
import numpy as np


U=C.c_uint32


class Pix(C.Structure):
    _fields_=[(n,U) for n in ('width','height','pixelformat','field','bytesperline',
        'sizeimage','colorspace','priv','flags','ycbcr_enc','quantization','xfer_func')]


class FmtUnion(C.Union):
    _fields_=[('pix',Pix),('raw',C.c_ubyte*200),('align',C.c_void_p)]


class Format(C.Structure):
    _fields_=[('type',U),('fmt',FmtUnion)]


class Request(C.Structure):
    _fields_=[('count',U),('type',U),('memory',U),('capabilities',U),
              ('flags',C.c_ubyte),('reserved',C.c_ubyte*3)]


class Timeval(C.Structure):
    _fields_=[('sec',C.c_long),('usec',C.c_long)]


class Timecode(C.Structure):
    _fields_=[('type',U),('flags',U),('data',C.c_ubyte*8)]


class Memory(C.Union):
    _fields_=[('offset',U),('userptr',C.c_ulong),('planes',C.c_void_p),('fd',C.c_int)]


class Buffer(C.Structure):
    _fields_=[('index',U),('type',U),('bytesused',U),('flags',U),('field',U),
              ('timestamp',Timeval),('timecode',Timecode),('sequence',U),
              ('memory',U),('m',Memory),('length',U),('reserved2',U),('request_fd',C.c_int)]


def ioctl(fd,n,obj,read=True):
    code=(0xC0000000 if read else 0x40000000)|(C.sizeof(obj)<<16)|(ord('V')<<8)|n
    fcntl.ioctl(fd,code,obj)


class Capture:
    def __init__(self,device,width,height):
        if (C.sizeof(Format),C.sizeof(Request),C.sizeof(Buffer))!=(208,20,88):
            raise RuntimeError('Unsupported V4L2 ABI')
        self.fd=os.open(device,os.O_RDWR|os.O_NONBLOCK);self.maps=[];self.streaming=False
        self.consecutive_errors=0;self.corrupt_frames=0
        try:
            f=Format();f.type=1;f.fmt.pix.width=width;f.fmt.pix.height=height
            f.fmt.pix.pixelformat=int.from_bytes(b'MJPG','little');f.fmt.pix.field=1
            ioctl(self.fd,5,f)
            if (f.fmt.pix.width,f.fmt.pix.height)!=(width,height) or f.fmt.pix.pixelformat!=int.from_bytes(b'MJPG','little'):
                raise RuntimeError('Camera did not accept exact MJPEG image settings')
            request=Request();request.type=1;request.memory=1;request.count=3;ioctl(self.fd,8,request)
            if request.count<2:raise RuntimeError('Camera has insufficient capture buffers')
            for i in range(request.count):
                b=Buffer();b.type=1;b.memory=1;b.index=i;ioctl(self.fd,9,b)
                self.maps.append(mmap.mmap(self.fd,b.length,mmap.MAP_SHARED,mmap.PROT_READ|mmap.PROT_WRITE,offset=b.m.offset))
                ioctl(self.fd,15,b)
            ioctl(self.fd,18,C.c_int(1),False);self.streaming=True
        except Exception:self.close();raise

    def read(self,timeout=.4):
        if not select.select([self.fd],[],[],timeout)[0]:raise RuntimeError('Camera frame timeout')
        b=Buffer();b.type=1;b.memory=1
        try:ioctl(self.fd,17,b)
        except OSError as e:
            if e.errno==errno.EAGAIN:return None
            raise
        try:
            if b.flags&0xE000!=0x2000:raise RuntimeError('Camera lacks kernel monotonic timestamps')
            if b.flags&0x40:
                self.consecutive_errors+=1;self.corrupt_frames+=1
                if self.consecutive_errors>3:raise RuntimeError('Four consecutive corrupt camera frames')
                return None
            payload=self.maps[b.index][:b.bytesused]
            image=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR)
            if image is None:raise RuntimeError('MJPEG frame decode failed')
            self.consecutive_errors=0
            return b.sequence,b.timestamp.sec+b.timestamp.usec/1e6,image
        finally:ioctl(self.fd,15,b)

    def close(self):
        if self.streaming:
            ioctl(self.fd,19,C.c_int(1),False);self.streaming=False
        for m in self.maps:m.close()
        self.maps=[]
        if self.fd>=0:os.close(self.fd);self.fd=-1
