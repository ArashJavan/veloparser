import os
from pathlib import Path
import dpkt
import datetime
import numpy as np

from gps import GprmcMessage
import lidar


def create_point_cloud(X, Y, Z):
    return None


def write_point_cloud_to_file(path, index, X, Y, Z, intensities, latitudes, longitudes, timestamps):
    pass


class VelodyneManager():

    def __init__(self, type, pcap_path, out_root, params):
        self.pcap_path = Path(pcap_path)
        self.params = params
        self.lidar_type = type
        self.lidar = None
        self.out_root = out_root

        self.pos_X = None
        self.pos_Y = None
        self.pos_Z = None
        self.intensities = None
        self.latitudes = None
        self.timestamps = None
        self.distances = None
        self.indicies = None
        self.longitudes = None

        self.frame_nr = 0
        self.cur_azimuth = None
        self.last_azimuth = None
        self.date = None

        if "velodynevlp16" == type.lower():
            self.lidar = lidar.VelodyneVLP16()

    def run(self):
        """
        Exteractis point clouds from pcap file
        :return:
        """

        # open pcap file
        try:
            fpcap = open(self.pcap_path, 'rb')
            self.lidar_reader = dpkt.pcap.Reader(fpcap)
        except Exception as ex:
            print(str(ex))
            return

        # create output folder hierarchy
        if not self.create_folders():
            return

        # create frame text writer
        if self.params['text']:
            self.frame_writer = PointCloudTextWriter(self.txt_path, index=self.frame_nr)

        # itearte through each data packet and timestamps
        for idx, (ts, buf) in enumerate(self.lidar_reader):

            if idx < self.params['from']:
                continue
            if 0 < self.params['to'] < idx:
                break

            if self.date is None:
                self.date = datetime.datetime.utcfromtimestamp(ts).date()

            eth = dpkt.ethernet.Ethernet(buf)
            data = eth.data.data.data

            # handle Position-Frame (GPS-Data)
            if self.params['gps']:
                if eth.data.data.sport == self.params['gps-port']:
                    self.process_gps_frame(data, ts, idx)

            # Handle Data-Frame (Point clouds)
            if eth.data.data.sport == self.params['data-port']:
                self.process_data_frame(data, ts, idx)

    def process_data_frame(self, data, timestamp, index):
        cur_X, cur_Y, cur_Z, cur_intensities, cur_latitudes, cur_timestamps, cur_distances = self.lidar.process_data_frame(data, index)

        # number of sequences
        n_seq = int(len(cur_X) / self.lidar.count_lasers)

        cur_indicies = np.tile(np.arange(self.lidar.count_lasers), n_seq)
        cur_longitudes = np.tile(self.lidar.omega, n_seq)

        # initilaise states
        if index == 0 or self.pos_X is None:
            self.pos_X = cur_X
            self.pos_Y = cur_Y
            self.pos_Z = cur_Z
            self.intensities = cur_distances
            self.latitudes = cur_latitudes
            self.timestamps = cur_timestamps
            self.distances = cur_distances
            self.indicies = cur_indicies
            self.longitudes = cur_longitudes


        if self.cur_azimuth is None:
            self.cur_azimuth = cur_latitudes
            self.last_azimuth = cur_latitudes

        # update current azimuth before checking for roll over
        self.cur_azimuth = cur_latitudes

        # check if a frame is finished
        idx_rollovr = self.is_roll_over()

        # handle rollover (full 360° frame)
        if idx_rollovr is not None:
            time = self.time_from_lidar(self.timestamps[0])

            if idx_rollovr > 0:
                self.pos_X = np.hstack((self.pos_X, cur_X[0:idx_rollovr+1]))
                self.pos_Y = np.hstack((self.pos_Y, cur_Y[0:idx_rollovr+1]))
                self.pos_Z = np.hstack((self.pos_Z, cur_Z[0:idx_rollovr+1]))
                self.intensities = np.hstack((self.intensities, cur_intensities[0:idx_rollovr+1]))
                self.latitudes = np.hstack((self.latitudes, cur_latitudes[0:idx_rollovr+1]))
                self.timestamps = np.hstack((self.timestamps, cur_timestamps[0:idx_rollovr+1]))
                self.distances = np.hstack((self.distances, cur_distances[0:idx_rollovr+1]))
                self.indicies = np.hstack((self.indicies, cur_indicies[0:idx_rollovr+1]))
                self.longitudes = np.hstack((self.longitudes, cur_longitudes[0:idx_rollovr+1]))

            if self.params['text']:

                fpath = "{}/{}_data_frame.txt".format(self.txt_path, index)
                write_pcl_txt(self.txt_path, date, self.timestamps[0], self.frame_nr, self.timestamps,
                              self.pos_X, self.pos_Y, self.pos_Z, self.indicies,
                              self.intensities, self.latitudes, self.longitudes, self.distances)
            if idx_rollovr > 0:
                self.pos_X = cur_X[idx_rollovr+1:]
                self.pos_Y = cur_Y[idx_rollovr+1:]
                self.pos_Z = cur_Z[idx_rollovr+1:]
                self.intensities = cur_intensities[idx_rollovr+1:]
                self.latitudes = cur_latitudes[idx_rollovr+1:]
                self.timestamps = cur_timestamps[idx_rollovr+1:]
                self.distances = cur_distances[idx_rollovr+1:]
                self.indicies = cur_indicies[idx_rollovr+1:]
                self.longitudes = cur_longitudes[idx_rollovr+1:]
            else:
                self.pos_X = cur_X
                self.pos_Y = cur_Y
                self.pos_Z = cur_Z
                self.intensities = cur_intensities
                self.latitudes = cur_latitudes
                self.timestamps = cur_timestamps
                self.distances = cur_distances
                self.indicies = cur_indicies
                self.longitudes = cur_longitudes

            self.frame_nr += 1

            # reset roll over check
            self.cur_azimuth = None
            return

        self.pos_X = np.hstack((self.pos_X, cur_X))
        self.pos_Y = np.hstack((self.pos_Y, cur_Y))
        self.pos_Z = np.hstack((self.pos_Z, cur_Z))
        self.intensities = np.hstack((self.intensities, cur_intensities))
        self.latitudes = np.hstack((self.latitudes, cur_latitudes))
        self.timestamps = np.hstack((self.timestamps, cur_timestamps))
        self.distances = np.hstack((self.distances, cur_distances))
        self.indicies = np.hstack((self.indicies, cur_indicies))
        self.longitudes = np.hstack((self.longitudes, cur_longitudes))

        self.last_azimuth = cur_latitudes

    def process_gps_frame(self, data, timestamp, index):
        gps_msg = self.lidar.process_position_frame(data, index)

        if self.params['text']:
            # write point cloud as a text-file
            write_gps_txt(self.txt_path, timestamp, index, gps_msg)

    def is_roll_over(self):

        diff_cur = self.cur_azimuth[0:-1] - self.cur_azimuth[1:]
        diff_cur_last = self.cur_azimuth - self.last_azimuth

        res_cur = np.where(diff_cur > 0.)[0]
        res_cur_last =  np.where(diff_cur_last < 0.)[0]
        if res_cur.size > 0:
            index = res_cur[0]
            return index
        elif res_cur_last.size > 0:
            index = res_cur_last[0]
            return index
        else:
            return None

    def time_from_lidar(self, timestamp):
        pass

    def create_folders(self):
        self.out_path = Path("{}/{}".format(self.out_root, self.lidar_type.lower()))

        # creating output dir
        try:
            os.makedirs(self.out_path.absolute())
        except Exception as ex:
            print(str(ex))
            return False

        # create point cloud dirs
        self.pcl_path = Path("{}/{}".format(self.out_path, "frames_pcl"))
        try:
            os.makedirs(self.pcl_path.absolute())
        except Exception as ex:
            print(str(ex))
            return False

        # if text-files are desired, create text-file dir
        if self.params['text']:
            self.txt_path = Path("{}/{}".format(self.out_path, "frames_text"))
            try:
                os.makedirs(self.txt_path.absolute())
            except Exception as ex:
                print(str(ex))
                return False
        return True


def write_gps_txt(path, ts_frame, index, gps_msg):
    fpath = "{}/{}_{}_{}.txt".format(path, index, "gps", str(datetime.datetime.utcfromtimestamp(ts_frame)))
    header = "UTC-Time, Week, Seconds [sow], Status, Latitude [Deg], Longitudes [Deg], Velocity [m/s]\n"
    txt = "{}, {}, {}, {}, {} {}, {} {}, {}\n".format(gps_msg.datetime, gps_msg.weeks, gps_msg.seconds,
                                                      gps_msg.status, gps_msg.lat, gps_msg.lat_ori,
                                                      gps_msg.long, gps_msg.lat_ori, gps_msg.velocity)
    with open(fpath, 'w') as fp:
        fp.write(header)
        fp.write(txt)


def write_pcl_txt(path, ts_frame, index, timestamps, X, Y, Z,  laser_id, intensities=None, latitudes=None, longitudes=None, distances=None):
    header = "Time [musec], X [m], Y [m], Z [m], ID, Intensity, Latitude [Deg], Longitudes [Deg], Distance [m]"
    fpath = "{}/{}_data_frame.txt".format(path,  index)

    try:
        fp = open(fpath, 'w')
        np.savetxt(fp, [], delimiter=', ', header=header)
    except Exception as ex:
        print(str(ex))
        return

    M = np.vstack((timestamps, X, Y, Z, laser_id))

    if intensities is not None:
        M = np.vstack((M, intensities))
    if latitudes is not None:
        M = np.vstack((M, latitudes))
    if longitudes is not None:
        M = np.vstack((M, longitudes))
    if distances is not None:
        M = np.vstack((M, distances))

    np.savetxt(fp, M.T, fmt=('%d', '%.6f', '%.6f', '%.6f', '%d', '%d', '%.3f', '%.3f', '%.3f'), delimiter=', ')
    fp.close()


class PointCloudTextWriter():
    def __init__(self, path, index):
        self.path = path
        self.initialised = False
        self.index = index

    def initialise(self):
        header = "Time [musec], X [m], Y [m], Z [m], ID, Intensity, Latitude [Deg], Longitudes [Deg], Distance [m]"
        fpath = "{}/{}_{}.txt".format(self.path, self.index, "data_frame")

        try:
            self.fp = open(self.fpath, 'w')
            np.savetxt(self.fp, [], delimiter=', ', header=header)
        except Exception as ex:
            print(str(ex))
            return

        self.initialised = True

    def write(self, timestamps, X, Y, Z,  laser_id, intensities=None, latitudes=None, longitudes=None, distances=None):

        if not self.initialised:
            self.initialise()

        M = np.vstack((timestamps, X, Y, Z, laser_id))

        if intensities is not None:
            M = np.vstack((M, intensities))
        if latitudes is not None:
            M = np.vstack((M, latitudes))
        if longitudes is not None:
            M = np.vstack((M, longitudes))
        if distances is not None:
            M = np.vstack((M, distances))

        np.savetxt(fp, M.T, fmt=('%d', '%.6f', '%.6f', '%.6f', '%d', '%d', '%.3f', '%.3f', '%.3f'), delimiter=', ')

    def close(self):
        self.fp.close()





