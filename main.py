import os
import argparse
import dpkt
import math
import numpy as np

def read_uint8(data, idx):
  return data[idx]


def read_sint8(data, idx):
  val = read_uint8(data, idx)
  return val-256 if val > 127 else val


def read_uint16(data, idx):
  return data[idx] + data[idx+1]*256


def read_sint16(data, idx):
  val = read_uint16(data, idx)
  return val-2**16 if val > 2**15-1 else val


def read_uint32(data, idx):
  return data[idx] + data[idx+1]*256 + data[idx+2]*256*256 + data[idx+3]*256*256*256

_RotSinTab = {}
_RotCosTab = {}

for i in range(360*100):
  _RotSinTab[i/100] = math.sin(math.radians(i/100))
  _RotCosTab[i/100] = math.cos(math.radians(i/100))

class Point():
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.z = 0.
        self.intensity = 0.
        self.laser_id = 0.
        self.timestamp = 0.

class Lidar():
    def __init__(self):
        self.point_cloud = []

    def process_data_frame(self, data, frame_idx):
        raise NotImplementedError("Subclasses should implement this!")

    def process_position_frame(self, data, frame_idx):
        raise NotImplementedError("Subclasses should implement this!")


class VelodyneVLP16(Lidar):
    def __init__(self):
        super(VelodyneVLP16, self).__init__()

    def process_data_frame(self, data, frame_idx):

        # veldyne has 12 blocks each 100 bytes data
        # data-legth = 1206 bytes
        blocks = data[0:1200].reshape(12, 100)
        timestamp = data[1200:1204]
        factory = data[1204:]

        # iteratie through each block
        for blk in blocks:
            self.read_firing_data(blk)

    def process_position_frame(self, data, frame_idx):
        raise NotImplementedError("Subclasses should implement this!")

    def read_firing_data(self, data):
        idx = 0
        block_id = read_uint16(data, idx)
        # 0xeeff is upper block
        assert block_id == 0xeeff

        idx += 2
        azimuth = read_uint16(data, idx) / 100.
        idx += 2

        # read two firing sequences
        for i in range(32):
            dist = read_uint16(data, idx)
            idx += 2
            intensity = read_uint8(data, idx)
            idx += 1
            # upper laser block ids range from 0 to 31, lower from 32 to 63
            laser_idx = i + (32 if block_id == 0xddff else 0)
            self.calc_point(laser_idx, azimuth, dist, intensity)

    def calc_point(self, laser_idx, azimuth, dist, intensity):
        # calc point 3d-geom.
        point = Point()
        return point

GPS_PORT = 8308
DATA_PORT = 2368

def main(args):
    path = args['path']
    with open(path, 'rb') as f:
        reader = dpkt.pcap.Reader(f)

        lidar = VelodyneVLP16()

        # itearte through each data packet and timestamps
        for idx, (ts, buf) in enumerate(reader):
            eth = dpkt.ethernet.Ethernet(buf)
            data = eth.data.data.data
            data = np.frombuffer(data, dtype=np.uint8).astype(np.uint32)

            if eth.data.data.sport == GPS_PORT:
                lidar.process_position_frame(data, idx)
            elif  eth.data.data.sport == DATA_PORT:
                lidar.process_data_frame(data, idx)

            # extracting datablcoks each 100 bytes
            idx = 0
            for i in range(12):
                block_id = read_uint16(data, idx)
                assert block_id == 0xeeff or block_id == 0xddff
                idx += 2
                azimuth = read_uint16(data, idx) / 100
                idx += 2
                for l in range(32):
                    dist = read_uint16(data, idx)
                    idx += 2
                    idx += 2
                    intensity = read_uint8(data, idx)
                    idx += 1

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--path', help="Path to the pcap file", required=True)

    args = vars(parser.parse_args())
    main(args)