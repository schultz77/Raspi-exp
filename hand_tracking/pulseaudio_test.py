import pulsectl

with pulsectl.Pulse('volume') as pulse:
    for sink in pulse.sink_list():
        print(sink)
        pulse.volume_change_all_chans(sink, -0.1)
        print(sink)
