# RTKLIB-EX on TEX-CUP

Follow [RUN.md](RUN.md) for the public source revision, build, GNSS processing,
output conversion and final collection. This method has completed the current
4,040-epoch protocol; [results](../../results/final/statistics.md) are saved in
the shared final folder.

The input is locally generated rover/base RINEX plus separately attributed
broadcast navigation. [SBF processing instructions](../../data/PROCESSING.md)
provide direct publisher downloads, pinned `convbin` commands and the
[rover](config/texcup_rover_sbf.json)/[base](config/texcup_base_sbf.json)
conversion profiles. The positioning
[config](config/texcup_rtk_demo5.conf) uses GPS 1C/2W, Galileo 1C/7Q and BeiDou 2I,
with GLONASS disabled. Native output is antenna ECEF/GPST; the converter changes
time to UTC without translating the antenna. No source patch is needed.
The validated source conversion covers 4,040 epochs at 1 Hz and does not
claim byte-for-byte recreation of the historical full-recording inputs.

The source is pinned to the public v2.5.1 revision in the run guide. RTKLIB's
native position solution is also the position input to VINS-Fusion and IC-GVINS.
