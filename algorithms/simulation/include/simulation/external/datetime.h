//    datetime.h
//
//     date and time routines for GPS Processing
//     This is a modfied version of the original routines created by
//     Charles Schwarz and Gordon Adams.  The functions were modified to
//     use the Remondi Date/Time algorithms (see Hilla & Jackson (2000),
//     The GPS Toolbox: The Remondi Date/Time Algorithms, GPS Solutions,
//     3(4), 71-74 ).
//
//  DateTime provides the following formats for representing a GPS-time:
//         1. GPS time (GPS week and second of week)
//         2. Modified Julian Date (MJD and fraction-of-a-day)
//         3. Year, day-of-year, hour, minute, seconds
//         4. Year, Month, day, hour, minute, seconds
//
// An object of the class DateTime may be initialized (constructed) from
//   any of the representations and viewed in any other representation.
//

#if !defined(DATETIME_)
#define DATETIME_

#if !defined(IOSTREAM_)
#include <iostream>
#define IOSTREAM_
#endif

namespace NGSdatetime {

using std::ostream;

typedef struct {
  long GPSWeek;
  double secsOfWeek;
} GPSTime;

typedef struct {
  long mjd;
  double fracOfDay;
} MJD;

typedef struct {
  long year;
  long dayOfYear;
  long hour;
  long min;
  double sec;
} YDOYHMS;

typedef struct {
  long year;
  long month;
  long day;
  long hour;
  long min;
  double sec;
} YMDHMS;

class DateTime {
  friend ostream& operator<<(ostream& output, DateTime& dt);

 public:
  // constructors
  DateTime();
  DateTime(GPSTime gpstime);
  DateTime(MJD mjd);
  DateTime(YDOYHMS yearDoyHMS);
  DateTime(YMDHMS yearMonthDayHMS);
  DateTime(long year, long month, long day, long hour, long min, double sec);

  // destructor
  ~DateTime();

  // initializers
  void SetGPSTime(GPSTime gpstime);
  void SetMJD(MJD mjd);
  void SetYDOYHMS(YDOYHMS yearDoyHMS);
  void SetYMDHMS(YMDHMS yearMonthDayHMS);
  void SetYMDHMS(
      long year, long month, long day, long hour, long min, double sec);

  // selectors
  GPSTime GetGPSTime();
  MJD GetMJD();
  YDOYHMS GetYDOYHMS();
  YMDHMS GetYMDHMS();

  // manipulators
  const DateTime& operator=(const DateTime& DT2);
  DateTime* operator&(DateTime input);

  DateTime operator+(const double days);
  double operator-(const DateTime& DT2);

  bool operator==(const DateTime& DT2);
  bool operator!=(const DateTime& DT2);
  bool operator>(const DateTime& DT2);
  bool operator>=(const DateTime& DT2);
  bool operator<(const DateTime& DT2);
  bool operator<=(const DateTime& DT2);

 private:
  long mjd;
  double fractionOfDay;
};

}  // namespace NGSdatetime

#endif
