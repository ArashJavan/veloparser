import os
from pathlib import Path
import dpkt
import datetime
import numpy as np
import pypcd
from  pypcd import pypcd
import lidar


def create_point_cloud(X, Y, Z):
    return None


def write_point_cloud_to_file(path, index, X, Y, Z, intensities, latitudes, longitudes, timestamps):
    pass

class VelodyneManager():

    def __init__(self, type, pcap_path, out_path, params):
        self.pcap_path = Path(pcap_path)
        self.params = params
        self.lidar_type = type
        self.out_path = Path("{}/{}".format(out_path, self.lidar_type.lower()))
        self.lidar = None
        if "velodynevlp16" == type.lower():
            self.lidar = lidar.VelodyneVLP16()

        if params['text']:
            txt_path = "{}/pcl_{}.txt".format(self.out_path, datetime.datetime.now())
            self.dframe_txt_writer = PointCloudToText(txt_path)

    def run(self):

        # open pcap file
        try:
            fpcap = open(self.pcap_path, 'rb')
            self.lidar_reader = dpkt.pcap.Reader(fpcap)
        except Exception as ex:
            print(str(ex))
            return

        # create output folder hierarchy
        self.create_folders()

        # itearte through each data packet and timestamps
        for idx, (ts, buf) in enumerate(self.lidar_reader):

            if idx < self.params['from']:
                continue
            if self.params['to'] > 0 and idx > self.params['to']:
                break

            eth = dpkt.ethernet.Ethernet(buf)
            data = eth.data.data.data

            data = np.frombuffer(data, dtype=np.uint8).astype(np.uint32)

            if self.params['gps']:
                if eth.data.data.sport == self.params['gps-port']:
                    self.lidar.process_position_frame(data, idx)

            if eth.data.data.sport == self.params['data-port']:
                self.process_data_frame(data, ts, idx)

    def process_data_frame(self, data, timestamp, index):
        X, Y, Z, intensities, latitudes, timestamps = self.lidar.process_data_frame(data, index)

        # number of sequences
        n_seq = int(len(X) / self.lidar.count_lasers)

        indicies = np.repeat(np.arange(self.lidar.count_lasers), n_seq)
        longitudes = np.repeat(self.lidar.omega, n_seq)

        # wrtie point cloud as a text-file
        self.dframe_txt_writer.write(indicies, X, Y, Z, timestamps, intensities, latitudes, longitudes)

    def create_folders(self):
        try:
            os.makedirs(self.outpath.absolute())
        except Exception as ex:
            print(str(ex))
            return

        # create point cloud path
        self.pcl_path = Path("{}/{}".format(self.outpath, "pointlcouds"))
        try:
            os.makedirs(self.pcl_path.absolute())
        except Exception as ex:
            print(str(ex))
            return


class PointCloudToText():
    def __init__(self, path):
        self.path = path
        self.initialised = False


    def initialise(self):

        header = "Id, X [m], Y, Z, Time [musec], Intensity, Latitude [Deg], Longitudes"
        try:
            self.fp = open(self.path, 'w')
            np.savetxt(self.fp, [], delimiter=',', header=header)
        except Exception as ex:
            print(str(ex))
            return

        self.initialised = True

    def write(self, laser_id, X, Y, Z,  timestamps, intensities=None, latitudes=None, longitudes=None):

        if not self.initialised:
            self.initialise()

        T = np.hstack((laser_id, X, Y, Z, timestamps))

        if intensities:
            T = np.vstack((T, intensities))
        if latitudes:
            T = np.vstack((T, intensities))
        if longitudes:
            T = np.vstack((T, intensities))

        np.savetxt(self.fp, T, delimiter=',')

    def close(self):
        self.fp.close()





