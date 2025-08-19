"""
This script plots the phase difference and SSS intensity of recorded binary data
By default it parse the file "log_20m_1000samples.bin" and visualize the intensity of first 1000 pings
and plot the phase difference at ping 200.
"""

import numpy as np 
import matplotlib.pyplot as plt 
import struct 
import sys 
file_ = "/root/ros2_ws/src/isss_ros2_driver/isss_data/"# 1727263582.524461.bin"
# file_ = "log_20m_1000samples.bin"
f = open(file_+sys.argv[1], mode="rb")
data = f.read()
# print(type(data))
# print(len(bytes))

num_samples = 1000
bytes_per_ping = 12 + 2 *3 *num_samples
# print(len(list(data))//bytes_per_ping)

total_nbr_pings = len(list(data))//bytes_per_ping
nbr_pings  = int(sys.argv[2])
print(f"total_nbr_pings: {total_nbr_pings} ")
print(f"plot the first: {nbr_pings} pings")
# assert False
sss_port = []
sss_stbd = []
sss_phase_port = []
sss_phase_stbd = []
sss_phase_port1 = []
sss_phase_stbd1 = []

"""
// Header
       0 // 0xFE
       1 // 0x05
       2 // size packet[15:8]  // 3007
       3 // size packet[ 7:0]
       4 // Type ISSS=0xE1
       5 // time[31:2]  [ms]
       6 // time[23:1]  [ms]
       7 // time[15:8]  [ms]
       8 // time[ 7:0]  [ms]
       9 // 0x01            // FREQ id 
       10 // decimation_i[15:8]
       11 // decimation_i[ 7:0]

    // 1000 loop
        // Channel 0 
        // phase[15:8]
        // phase[ 7:0]
        // RSS

        // Channel 1
        // phase[15:8]
        // phase[ 7:0]
        // RSS
    // End loop
"""
index_list = []
for i in range(bytes_per_ping*nbr_pings):
    if int(data[i])==254 and int(data[i+1]==5) and int(data[i+4])==225:
        index_list.append(i)
print(len(index_list), nbr_pings)
print(f'offset: {index_list[0]}')
# assert len(index_list)==nbr_pings

start_ping_num=0
end_ping_num=nbr_pings




for (ping_num,i) in enumerate(index_list):
    if ping_num < start_ping_num or ping_num>end_ping_num :
        continue
    sonar_data = data[i+12:i+12+2*3*num_samples]
    phase_15_8_port = sonar_data[0::6]
    phase_7_0_port = sonar_data[1::6]
    rss_port = sonar_data[2::6]

    phase_15_8_stbd = sonar_data[3::6]
    phase_7_0_stbd = sonar_data[4::6]

    rss_stbd = sonar_data[5::6]
    # plt.figure()
    # plt.scatter(np.arange(1000), list(rss_stbd),s=1)
    # plt.show()
    # assert False

    sss_port.append(list(rss_port))
    sss_phase_port.append(list(phase_7_0_port))
    sss_phase_port1.append(list(phase_15_8_port))


    sss_stbd.append(list(rss_stbd))
    sss_phase_stbd.append(list(phase_7_0_stbd))
    sss_phase_stbd1.append(list(phase_15_8_stbd))
nbr_pings = len(sss_port)
sss_port = np.array(sss_port[:nbr_pings-1])
sss_phase_port = np.array(sss_phase_port[:nbr_pings-1])
sss_phase_port1 = np.array(sss_phase_port1[:nbr_pings-1])

sss_stbd = np.array(sss_stbd[:nbr_pings-1])
sss_phase_stbd = np.array(sss_phase_stbd[:nbr_pings-1])
sss_phase_stbd1 = np.array(sss_phase_stbd1[:nbr_pings-1])

SSS_phase_port = 256 * sss_phase_port1 + sss_phase_port
SSS_phase_stbd = 256 * sss_phase_stbd1 + sss_phase_stbd

SSS_phase_port[SSS_phase_port>65536//2]-=65535
SSS_phase_stbd[SSS_phase_stbd>65536//2]-=65535


# sss_port[:,:10]=0
# sss_stbd[:,:10]=0


plt.figure()
plt.imshow(np.concatenate((np.fliplr(sss_stbd), sss_port), axis=1), cmap="gray", vmax=180)
plt.title("sss")
#Arka modifications
plt.savefig('plot2.png')
print(f"Saved plot to 'plot2.png'")
# plt.figure()
# plt.imshow(sss_stbd, cmap="gray", vmax=sss_stbd.max())
# plt.title("sss_stbd")

idx = int(sys.argv[3])
plt.figure()
plt.scatter(np.arange(num_samples*2), np.concatenate((np.fliplr(SSS_phase_stbd), SSS_phase_port),axis=1)[idx], s=1)
plt.title(f"SSS port phase difference at ping {idx}")

# plt.figure()
# plt.scatter(np.arange(num_samples), SSS_phase_stbd[idx], s=1)
# plt.title(f"SSS stbd phase difference at ping {idx}")

plt.show()