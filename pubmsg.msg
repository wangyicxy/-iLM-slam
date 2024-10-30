Header header
float32 gpsweek
float32 gpstime
float32 heading
float32 pitch
float32 roll
float64 lat
float64 lon
float64 alt
float64 error
int16 status #天线、GPS标志位，若Snwm<15 或 hdop>5 则status = 0，表示掉线，否则status = 1,表示正常
