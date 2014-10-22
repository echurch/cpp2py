from ROOT import *
d = DaqFile("/data/echurch/test_dir/annex-29Apr2014-INT-trig-1301-1.dat")

gSystem.Load("libudata_types.so")

e = d.GetEventObj(1) # 1st event!
