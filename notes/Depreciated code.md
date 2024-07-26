```
  struct NMEA_GNGGA{
    char GNGGA[6];
    float time;
    float lat;
    char NS;
    float lon;
    char EW;
    int quality;
    int numSV;
    float HDOP;
    float alt;
    char altUnit;
    float sep;
    char sepUnit;
    int diffAge;
    int diffStation;
    unsigned char cksm;
  }
  struct NMEA_GNGGA GPSstruct;
```