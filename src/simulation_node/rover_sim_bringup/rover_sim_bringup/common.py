def gz_time(header): return header.stamp.sec+header.stamp.nsec*1e-9
def set_stamp(stamp,t):
    stamp.sec=int(t); stamp.nanosec=int(round((t-int(t))*1e9))
    if stamp.nanosec>=1_000_000_000: stamp.sec+=1; stamp.nanosec-=1_000_000_000
