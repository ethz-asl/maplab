//    datetime.cpp - This is a modified version of the Date/Time class
//             originally created by Charles Schwarz and Gordon Adams
//             -- Steve Hilla, 1 May 2000.
//
// 0005.01, SAH, Change code to use mjd, fmjd as the private data. The
//              difference between two DateTime objects will be in "days".
// 0012.27, SAH, Change LeapMonths[] and NormalMonths[] to hold 13 elements.
// 0103.13, SAH, modify the >, >=, <, <= manipulator member functions.
// 0104.11, SAH, Add an output operator function ( << ).
// 0105.08, SAH, add normalize function to prevent HMS output where sec = 60.0

#include "simulation/external/datetime.h"

#if !defined(MATH_)
#include <math.h>
#define MATH_
#endif

#if !defined(IOSTREAM_)
#include <iostream>
#define IOSTREAM_
#endif

#if !defined(IOMANIP_)
#include <iomanip>
#define IOMANIP_
#endif

namespace NGSdatetime {

using namespace std;

const long JAN61980 = 44244;
const long JAN11901 = 15385;
const double SECPERDAY = 86400.0;

const long LeapMonths[13] = {0,   31,  60,  91,  121, 152, 182,
                             213, 244, 274, 305, 335, 366};

const long NormalMonths[13] = {0,   31,  59,  90,  120, 151, 181,
                               212, 243, 273, 304, 334, 365};

// Constructors

DateTime::DateTime() {
  mjd = 0;
  fractionOfDay = 0.0;
}

DateTime::DateTime(GPSTime gpstime) {
  mjd = gpstime.GPSWeek * 7 + gpstime.secsOfWeek / SECPERDAY + JAN61980;
  fractionOfDay = fmod(gpstime.secsOfWeek, SECPERDAY) / SECPERDAY;
}

DateTime::DateTime(MJD MJD2) {
  mjd = MJD2.mjd;
  fractionOfDay = MJD2.fracOfDay;
}

DateTime::DateTime(YDOYHMS yearDoyHMS) {
  mjd = ((yearDoyHMS.year - 1901) / 4) * 1461 +
        ((yearDoyHMS.year - 1901) % 4) * 365 + yearDoyHMS.dayOfYear - 1 +
        JAN11901;

  fractionOfDay =
      ((yearDoyHMS.sec / 60.0 + yearDoyHMS.min) / 60.0 + yearDoyHMS.hour) /
      24.0;
}

DateTime::DateTime(YMDHMS yearMonthDayHMS) {
  long doy;

  if (yearMonthDayHMS.year % 4 == 0)  // good until the year 2100
    doy = LeapMonths[yearMonthDayHMS.month - 1] + yearMonthDayHMS.day;
  else
    doy = NormalMonths[yearMonthDayHMS.month - 1] + yearMonthDayHMS.day;

  mjd = ((yearMonthDayHMS.year - 1901) / 4) * 1461 +
        ((yearMonthDayHMS.year - 1901) % 4) * 365 + doy - 1 + JAN11901;

  fractionOfDay = ((yearMonthDayHMS.sec / 60.0 + yearMonthDayHMS.min) / 60.0 +
                   yearMonthDayHMS.hour) /
                  24.0;
}

DateTime::DateTime(
    long year, long month, long day, long hour, long min, double sec) {
  long doy;

  if (year % 4 == 0)  // good until the year 2100
    doy = LeapMonths[month - 1] + day;
  else
    doy = NormalMonths[month - 1] + day;

  mjd = ((year - 1901) / 4) * 1461 + ((year - 1901) % 4) * 365 + doy - 1 +
        JAN11901;

  fractionOfDay = ((sec / 60.0 + min) / 60.0 + hour) / 24.0;
}

// Destructor

DateTime::~DateTime() {
  // no need to delete mjd and fractionOfDay
  // since they were not created using new.
}

// Initializers - used to set the value of an existing variable.

void DateTime::SetGPSTime(GPSTime gpstime2) {
  *this = DateTime(gpstime2);
}

void DateTime::SetMJD(MJD inputMJD) {
  *this = DateTime(inputMJD);
}

void DateTime::SetYDOYHMS(YDOYHMS yearDoyHMS) {
  *this = DateTime(yearDoyHMS);
}

void DateTime::SetYMDHMS(YMDHMS yearMonthDayHMS) {
  *this = DateTime(yearMonthDayHMS);
}

void DateTime::SetYMDHMS(
    long year, long month, long monthday, long hour, long min, double sec) {
  *this = DateTime(year, month, monthday, hour, min, sec);
}

// Selectors - return the contents of a variable.

GPSTime DateTime::GetGPSTime() {
  GPSTime tt;

  tt.GPSWeek = (mjd - JAN61980) / 7;
  tt.secsOfWeek =
      ((mjd - JAN61980) - tt.GPSWeek * 7 + fractionOfDay) * SECPERDAY;
  return tt;
}

MJD DateTime::GetMJD() {
  MJD mm;

  mm.mjd = mjd;
  mm.fracOfDay = fractionOfDay;

  return mm;
}

YDOYHMS DateTime::GetYDOYHMS() {
  YDOYHMS yy;
  long daysFromJan11901, deltaYears, numberFourYears;
  long yearsSoFar, daysLeft;

  daysFromJan11901 = mjd - JAN11901;
  numberFourYears = daysFromJan11901 / 1461;
  yearsSoFar = 1901 + 4 * numberFourYears;
  daysLeft = daysFromJan11901 - 1461 * numberFourYears;
  deltaYears = daysLeft / 365 - daysLeft / 1460;

  yy.year = yearsSoFar + deltaYears;
  yy.dayOfYear = daysLeft - 365 * deltaYears + 1;
  yy.hour = fractionOfDay * 24.0;
  yy.min = fractionOfDay * 1440.0 - yy.hour * 60.0;
  yy.sec = fractionOfDay * 86400.0 - yy.hour * 3600.0 - yy.min * 60.0;

  return yy;
}

YMDHMS DateTime::GetYMDHMS() {
  YMDHMS ymd;
  long daysFromJan11901, deltaYears, numberFourYears;
  long more, guess, doy, yearsSoFar, daysLeft;

  daysFromJan11901 = mjd - JAN11901;
  numberFourYears = daysFromJan11901 / 1461;
  yearsSoFar = 1901 + 4 * numberFourYears;
  daysLeft = daysFromJan11901 - 1461 * numberFourYears;
  deltaYears = daysLeft / 365 - daysLeft / 1460;

  ymd.year = yearsSoFar + deltaYears;
  doy = daysLeft - 365 * deltaYears + 1;
  ymd.hour = fractionOfDay * 24.0;
  ymd.min = fractionOfDay * 1440.0 - ymd.hour * 60.0;
  ymd.sec = fractionOfDay * 86400.0 - ymd.hour * 3600.0 - ymd.min * 60.0;

  guess = doy * 0.032;
  more = 0;
  if (ymd.year % 4 == 0)  // good until the year 2100
  {
    if ((doy - LeapMonths[guess + 1]) > 0)
      more = 1;
    ymd.month = guess + more + 1;
    ymd.day = doy - LeapMonths[guess + more];
  } else {
    if ((doy - NormalMonths[guess + 1]) > 0)
      more = 1;
    ymd.month = guess + more + 1;
    ymd.day = doy - NormalMonths[guess + more];
  }

  return ymd;
}

// Operators

DateTime* DateTime::operator&(DateTime input) {
  return &input;
}

// const return avoids: (a1 = a2 ) = a3
const DateTime& DateTime::operator=(const DateTime& DT2) {
  if (&DT2 != this)  // avoids self assignment
  {
    mjd = DT2.mjd;
    fractionOfDay = DT2.fractionOfDay;
  }
  return *this;
}

DateTime DateTime::operator+(const double days)  // DT + "d.fff"
{
  DateTime DT;
  DT.mjd = mjd;
  DT.fractionOfDay = fractionOfDay + days;

  if (DT.fractionOfDay > 1.0) {
    double wholeDays = floor(DT.fractionOfDay);  // use normjd *******
    DT.mjd = DT.mjd + (long)wholeDays;
    DT.fractionOfDay = DT.fractionOfDay - wholeDays;
  }
  return DT;
}

double DateTime::operator-(const DateTime& DT2)  // find difference in days
// this method will allow a negative difference
{
  return (
      (((double)mjd - (double)DT2.mjd) + fractionOfDay) - DT2.fractionOfDay);
}

bool DateTime::operator==(const DateTime& DT2) {
  return (mjd == DT2.mjd && fractionOfDay == DT2.fractionOfDay);
}

bool DateTime::operator!=(const DateTime& DT2) {
  return (mjd != DT2.mjd || fractionOfDay != DT2.fractionOfDay);
}

bool DateTime::operator>(const DateTime& DT2) {
  if (mjd > DT2.mjd || (mjd == DT2.mjd && fractionOfDay > DT2.fractionOfDay)) {
    return true;
  }
  return false;
}

bool DateTime::operator>=(const DateTime& DT2) {
  if (mjd > DT2.mjd || (mjd == DT2.mjd && fractionOfDay >= DT2.fractionOfDay)) {
    return true;
  }
  return false;
}

bool DateTime::operator<(const DateTime& DT2) {
  if (mjd < DT2.mjd || (mjd == DT2.mjd && fractionOfDay < DT2.fractionOfDay)) {
    return true;
  }
  return false;
}

bool DateTime::operator<=(const DateTime& DT2) {
  if (mjd < DT2.mjd || (mjd == DT2.mjd && fractionOfDay <= DT2.fractionOfDay)) {
    return true;
  }
  return false;
}

ostream& operator<<(ostream& output, DateTime& dt) {
  YMDHMS ymdhms = dt.GetYMDHMS();
  output.setf(ios::fixed, ios::floatfield);
  output << setw(4) << ymdhms.year << " " << setw(2) << ymdhms.month << " "
         << setw(2) << ymdhms.day << "  " << setw(2) << ymdhms.hour << ":"
         << setw(2) << ymdhms.min << ":" << setw(9) << setprecision(6)
         << ymdhms.sec;

  return output;  // enables chaining
}

}  // namespace NGSdatetime
