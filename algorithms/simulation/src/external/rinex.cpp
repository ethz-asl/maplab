// rinex.cpp
// C++ classes for Reading and Writing RINEX files
// ver. 200302.12
//
// Authors:  Steve Hilla
//           Gordon Adams
//           National Geodetic Survey, NOAA
//           Silver Spring, Maryland  20910
//
// 12Feb03, test to see if there is a time tag in columns 2-26
//          of the EPOCH/SAT record (in RINEX Observation Files).

#include "simulation/external/rinex.h"

#if !defined(IOSTREAM_)
#include <iostream>
#define IOSTREAM_
#endif

#if !defined(FSTREAM_)
#include <fstream>
#define FSTREAM_
#endif

#if !defined(SSTREAM_)
#include <sstream>
#define SSTREAM_
#endif

#if !defined(IOMANIP_)
#include <iomanip>
#define IOMANIP_
#endif

#if !defined(STRING_)
#include <string>
#define STRING_
#endif

#if !defined(LIMITS_)
#include <limits>
#define LIMITS_
#endif

#if !defined(ALGORITHM_)
#include <algorithm>  // for replace() function
#define ALGORITHM_
#endif

#if !defined(CSTRING_)
#include <cstring>
#define CSTRING_
#endif

#if !defined(CSTDLIB_)
#include <cstdlib>
#define CSTDLIB_
#endif

#if !defined(CMATH_)
#include <cmath>
#define CMATH_
#endif

#if !defined(CCTYPE_)
#include <cctype>
#define CCTYPE_
#endif

#if !defined(DATETIME_H_)
#include "datetime.h"
#define DATETIME_H_
#endif

namespace NGSrinex {

using namespace std;

//===================== ObsEpoch Class ========================================

ObsEpoch::ObsEpoch()  // default Constructor
{
  //    Initialize the OBS epoch structure with NULL values.  For strings NULL
  //    is an empty string.  For numeric values the NULL is chosen as a value
  //    that can be stored in the data type (short) but which is outside of the
  //    allowed range for the data element.  NULLs are typically 9999

  initializeData();
}

ObsEpoch::~ObsEpoch()  // Destructor
{}

// Initializers
bool ObsEpoch::setEpochTime(DateTime input) {
  DateTime Jan1_1980, Dec31_2079;

  Jan1_1980.SetYMDHMS(1980, 1, 1, 0, 0, 0.00000001);
  Dec31_2079.SetYMDHMS(2079, 12, 31, 23, 59, 59.99999999);

  if (input >= Jan1_1980 && input <= Dec31_2079) {
    epochTime = input;
    return true;
  } else {
    epochTime.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
    return false;
  }
}

bool ObsEpoch::setEpochFlag(unsigned short input) {
  if (input < 7) {
    epochFlag = input;
    return true;
  } else {
    epochFlag = 9999;
    return false;
  }
}

bool ObsEpoch::setNumSat(unsigned short input) {
  if (input <= 999) {
    numSat = input;
    return true;
  } else {
    numSat = 9999;
    return false;
  }
}

bool ObsEpoch::setSatListElement(
    SatObsAtEpoch input, unsigned short numObsTypes, int i) {
  bool problemsFound = false;

  // Set the satCode member
  if (input.satCode == 'G' || input.satCode == ' ' || input.satCode == 'R') {
    satList[i].satCode = input.satCode;
  } else {
    satList[i].satCode = ' ';
    problemsFound = true;
  }

  // Set the satNum member
  if (input.satNum < MAXPRNID) {
    satList[i].satNum = input.satNum;
  } else {
    satList[i].satNum = 9999;
    problemsFound = true;
  }

  // Set the obsList[] member
  for (int j = 0; j < numObsTypes; j++) {
    if ((input.obsList[j].obsType == C1 || input.obsList[j].obsType == L1 ||
         input.obsList[j].obsType == L2 || input.obsList[j].obsType == P1 ||
         input.obsList[j].obsType == P2 || input.obsList[j].obsType == D1 ||
         input.obsList[j].obsType == D2 || input.obsList[j].obsType == S1 ||
         input.obsList[j].obsType == S2) &&
        (input.obsList[j].LLI <= 7) && (input.obsList[j].sigStrength <= 9)) {
      satList[i].obsList[j].obsPresent = input.obsList[j].obsPresent;
      satList[i].obsList[j].observation = input.obsList[j].observation;
      satList[i].obsList[j].obsType = input.obsList[j].obsType;
      satList[i].obsList[j].LLI = input.obsList[j].LLI;
      satList[i].obsList[j].sigStrength = input.obsList[j].sigStrength;
    } else {
      satList[i].obsList[j].obsPresent = false;
      satList[i].obsList[j].observation = 0.0;  // blank = 0.0
      satList[i].obsList[j].obsType = NOOBS;
      satList[i].obsList[j].LLI = 9999;
      satList[i].obsList[j].sigStrength = 9999;
      problemsFound = true;
    }
  }  // j-loop over MAXOBSTYPES

  if (problemsFound)
    return false;
  else
    return true;
}

bool ObsEpoch::setRecClockOffset(double input) {
  if (input > -100.0 && input < 100.0) {
    recClockOffset = input;
    return true;
  } else {
    recClockOffset = 9999.0;
    return false;
  }
}

bool ObsEpoch::appendToEpochHeaderRecords(string input) {
  epochHeaderRecords.append(input);
  epochHeaderRecords.append("\n");
  return true;
}

bool ObsEpoch::initializeData() {
  //    Initializes the OBS epoch structure with NULL values.  For strings NULL
  //    is an empty string.  For numeric values the NULL is chosen as a value
  //    that can be stored in the data type (short) but which is outside of the
  //    allowed range for the data element.  NULLs are typically 9999

  epochTime.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
  epochFlag = 9999;
  numSat = 9999;

  for (int i = 0; i < MAXSATPEREPOCH; i++) {
    satList[i].satCode = ' ';
    satList[i].satNum = 9999;

    for (int j = 0; j < MAXOBSTYPES; j++) {
      satList[i].obsList[j].obsPresent = false;
      satList[i].obsList[j].observation = 0.0;  // blank = 0.0
      satList[i].obsList[j].obsType = NOOBS;
      satList[i].obsList[j].LLI = 9999;
      satList[i].obsList[j].sigStrength = 9999;
    }
  }

  recClockOffset = 9999.0;
  epochHeaderRecords = "";
  return true;
}

// Selectors
DateTime ObsEpoch::getEpochTime() {
  return epochTime;
}

unsigned short ObsEpoch::getEpochFlag() {
  return epochFlag;
}

unsigned short ObsEpoch::getNumSat() {
  return numSat;
}

SatObsAtEpoch ObsEpoch::getSatListElement(int i) {
  return satList[i];
}

double ObsEpoch::getRecClockOffset() {
  return recClockOffset;
}

string ObsEpoch::getEpochHeaderRecords() {
  return epochHeaderRecords;
}

//===================== MetEpoch Class ====================================

MetEpoch::MetEpoch()  // Default Constructor
{
  epochTime.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
  for (int i = 0; i < MAXMETTYPES; i++) {
    metList[i].obsPresent = false;
    metList[i].observation = 0.0;  // blank = 0.0
    metList[i].metType = NOMET;
  }
}

MetEpoch::~MetEpoch()  // Destructor
{}

// Initializers
bool MetEpoch::setEpochTime(DateTime input) {
  DateTime Jan1_1980, Dec31_2079;

  Jan1_1980.SetYMDHMS(1980, 1, 1, 0, 0, 0.00000001);
  Dec31_2079.SetYMDHMS(2079, 12, 31, 23, 59, 59.99999999);

  if (input >= Jan1_1980 && input <= Dec31_2079) {
    epochTime = input;
    return true;
  } else {
    epochTime.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
    return false;
  }
}

bool MetEpoch::setMetListElement(MetSet input, int i) {
  if (input.metType == PR || input.metType == TD || input.metType == HR ||
      input.metType == ZW || input.metType == ZD || input.metType == ZT) {
    metList[i].obsPresent = input.obsPresent;
    metList[i].observation = input.observation;
    metList[i].metType = input.metType;
    return true;
  } else {
    metList[i].obsPresent = false;
    metList[i].observation = 0.0;  // blank = 0.0
    metList[i].metType = NOMET;
    return false;
  }
}

// Selectors
DateTime MetEpoch::getEpochTime() {
  return epochTime;
}

MetSet MetEpoch::getMetListElement(int i) {
  return metList[i];
}

//===================== ClkEpoch Class ====================================

ClkEpoch::ClkEpoch()  // Default Constructor
{
  clockDataType = NOCLK;
  recvrSatName = "";
  epochTime.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
  numberDataValues = 0;
  clockBias = 0.0;
  clockBiasSigma = 0.0;
  clockRate = 0.0;
  clockRateSigma = 0.0;
  clockAcceleration = 0.0;
  clockAccelSigma = 0.0;
}

ClkEpoch::~ClkEpoch()  // Destructor
{}

// Initializers
bool ClkEpoch::setClockDataType(enum CLKTYPE input) {
  if (input == AR || input == AS || input == CR || input == DR || input == MS) {
    clockDataType = input;
    return true;
  } else {
    clockDataType = NOCLK;
    return false;
  }
}

bool ClkEpoch::setRecvrSatName(string input) {
  recvrSatName = input;
  return true;
}

bool ClkEpoch::setEpochTime(DateTime input) {
  DateTime Jan1_1980, Dec31_2079;

  Jan1_1980.SetYMDHMS(1980, 1, 1, 0, 0, 0.00000001);
  Dec31_2079.SetYMDHMS(2079, 12, 31, 23, 59, 59.99999999);

  if (input >= Jan1_1980 && input <= Dec31_2079) {
    epochTime = input;
    return true;
  } else {
    epochTime.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
    return false;
  }
}

bool ClkEpoch::setNumberDataValues(unsigned short input) {
  numberDataValues = input;
  return true;
}

bool ClkEpoch::setClockBias(double input) {
  clockBias = input;
  return true;
}

bool ClkEpoch::setClockBiasSigma(double input) {
  clockBiasSigma = input;
  return true;
}

bool ClkEpoch::setClockRate(double input) {
  clockRate = input;
  return true;
}

bool ClkEpoch::setClockRateSigma(double input) {
  clockRateSigma = input;
  return true;
}

bool ClkEpoch::setClockAcceleration(double input) {
  clockAcceleration = input;
  return true;
}

bool ClkEpoch::setClockAccelSigma(double input) {
  clockAccelSigma = input;
  return true;
}

// Selectors
CLKTYPE ClkEpoch::getClockDataType() {
  return clockDataType;
}

string ClkEpoch::getRecvrSatName() {
  return recvrSatName;
}

DateTime ClkEpoch::getEpochTime() {
  return epochTime;
}

unsigned short ClkEpoch::getNumberDataValues() {
  return numberDataValues;
}

double ClkEpoch::getClockBias() {
  return clockBias;
}

double ClkEpoch::getClockBiasSigma() {
  return clockBiasSigma;
}

double ClkEpoch::getClockRate() {
  return clockRate;
}

double ClkEpoch::getClockRateSigma() {
  return clockRateSigma;
}

double ClkEpoch::getClockAcceleration() {
  return clockAcceleration;
}

double ClkEpoch::getClockAccelSigma() {
  return clockAccelSigma;
}

//===================== PRNBlock Class ====================================

PRNBlock::PRNBlock()  // Default Constructor
{
  satellitePRN = 0;
  tocYear = 9999;
  tocMonth = 1;
  tocDay = 1;
  tocHour = 0;
  tocMin = 0;
  tocSec = 0.0;
  clockBias = 0.0;
  clockDrift = 0.0;
  clockDriftRate = 0.0;
  iode = 0.0;
  crs = 0.0;
  deltan = 0.0;
  mo = 0.0;
  cuc = 0.0;
  eEccen = 0.0;
  cus = 0.0;
  sqrtA = 0.0;
  toe = 0.0;
  cic = 0.0;
  bigOmega = 0.0;
  cis = 0.0;
  io = 0.0;
  crc = 0.0;
  lilOmega = 0.0;
  bigOmegaDot = 0.0;
  idot = 0.0;
  codesOnL2 = 0.0;
  toeGPSWeek = 0.0;
  pDataFlagL2 = 0.0;
  svAccur = 0.0;
  svHealth = 0.0;
  tgd = 0.0;
  iodc = 0.0;
  transmTime = 0.0;
  fitInterval = 0.0;
  spare1 = 0.0;
  spare2 = 0.0;
}

PRNBlock::~PRNBlock()  // Destructor
{}

// Initializers
bool PRNBlock::setSatellitePRN(unsigned short input) {
  if (input <= MAXPRNID) {
    satellitePRN = input;
    return true;
  } else {
    satellitePRN = 0;
    return false;
  }
}

bool PRNBlock::setTocYear(unsigned short input) {
  if (input >= 1980 && input <= 2079) {
    tocYear = input;
    return true;
  } else {
    tocYear = 9999;
    return false;
  }
}

bool PRNBlock::setTocMonth(unsigned short input) {
  if (input >= 1 && input <= 12) {
    tocMonth = input;
    return true;
  } else {
    tocMonth = 1;
    return false;
  }
}

bool PRNBlock::setTocDay(unsigned short input) {
  if (input >= 1 && input <= 31) {
    tocDay = input;
    return true;
  } else {
    tocDay = 1;
    return false;
  }
}

bool PRNBlock::setTocHour(unsigned short input) {
  if (input <= 24) {
    tocHour = input;
    return true;
  } else {
    tocHour = 0;
    return false;
  }
}

bool PRNBlock::setTocMin(unsigned short input) {
  if (input <= 60) {
    tocMin = input;
    return true;
  } else {
    tocMin = 0;
    return false;
  }
}

bool PRNBlock::setTocSec(double input) {
  if (input >= 0.0 && input <= 60.0) {
    tocSec = input;
    return true;
  } else {
    tocSec = 0.0;
    return false;
  }
}

bool PRNBlock::setClockBias(double input) {
  clockBias = input;
  return true;
}

bool PRNBlock::setClockDrift(double input) {
  clockDrift = input;
  return true;
}

bool PRNBlock::setClockDriftRate(double input) {
  clockDriftRate = input;
  return true;
}

bool PRNBlock::setIode(double input) {
  iode = input;
  return true;
}
bool PRNBlock::setCrs(double input) {
  crs = input;
  return true;
}
bool PRNBlock::setDeltan(double input) {
  deltan = input;
  return true;
}
bool PRNBlock::setMo(double input) {
  mo = input;
  return true;
}

bool PRNBlock::setCuc(double input) {
  cuc = input;
  return true;
}
bool PRNBlock::setEccen(double input) {
  eEccen = input;
  return true;
}
bool PRNBlock::setCus(double input) {
  cus = input;
  return true;
}
bool PRNBlock::setSqrtA(double input) {
  sqrtA = input;
  return true;
}

bool PRNBlock::setToe(double input) {
  toe = input;
  return true;
}
bool PRNBlock::setCic(double input) {
  cic = input;
  return true;
}
bool PRNBlock::setBigOmega(double input) {
  bigOmega = input;
  return true;
}
bool PRNBlock::setCis(double input) {
  cis = input;
  return true;
}

bool PRNBlock::setIo(double input) {
  io = input;
  return true;
}
bool PRNBlock::setCrc(double input) {
  crc = input;
  return true;
}
bool PRNBlock::setLilOmega(double input) {
  lilOmega = input;
  return true;
}
bool PRNBlock::setBigOmegaDot(double input) {
  bigOmegaDot = input;
  return true;
}

bool PRNBlock::setIdot(double input) {
  idot = input;
  return true;
}
bool PRNBlock::setCodesOnL2(double input) {
  codesOnL2 = input;
  return true;
}
bool PRNBlock::setToeGPSWeek(double input) {
  toeGPSWeek = input;
  return true;
}
bool PRNBlock::setPDataFlagL2(double input) {
  pDataFlagL2 = input;
  return true;
}

bool PRNBlock::setSvAccur(double input) {
  svAccur = input;
  return true;
}
bool PRNBlock::setSvHealth(double input) {
  svHealth = input;
  return true;
}
bool PRNBlock::setTgd(double input) {
  tgd = input;
  return true;
}
bool PRNBlock::setIodc(double input) {
  iodc = input;
  return true;
}

bool PRNBlock::setTransmTime(double input) {
  transmTime = input;
  return true;
}
bool PRNBlock::setFitInterval(double input) {
  fitInterval = input;
  return true;
}
bool PRNBlock::setSpare1(double input) {
  spare1 = input;
  return true;
}
bool PRNBlock::setSpare2(double input) {
  spare2 = input;
  return true;
}

// Selectors
unsigned short PRNBlock::getSatellitePRN() {
  return satellitePRN;
}
unsigned short PRNBlock::getTocYear() {
  return tocYear;
}
unsigned short PRNBlock::getTocMonth() {
  return tocMonth;
}
unsigned short PRNBlock::getTocDay() {
  return tocDay;
}
unsigned short PRNBlock::getTocHour() {
  return tocHour;
}
unsigned short PRNBlock::getTocMin() {
  return tocMin;
}
double PRNBlock::getTocSec() {
  return tocSec;
}
double PRNBlock::getClockBias() {
  return clockBias;
}
double PRNBlock::getClockDrift() {
  return clockDrift;
}
double PRNBlock::getClockDriftRate() {
  return clockDriftRate;
}

double PRNBlock::getIode() {
  return iode;
}
double PRNBlock::getCrs() {
  return crs;
}
double PRNBlock::getDeltan() {
  return deltan;
}
double PRNBlock::getMo() {
  return mo;
}

double PRNBlock::getCuc() {
  return cuc;
}
double PRNBlock::getEccen() {
  return eEccen;
}
double PRNBlock::getCus() {
  return cus;
}
double PRNBlock::getSqrtA() {
  return sqrtA;
}

double PRNBlock::getToe() {
  return toe;
}
double PRNBlock::getCic() {
  return cic;
}
double PRNBlock::getBigOmega() {
  return bigOmega;
}
double PRNBlock::getCis() {
  return cis;
}

double PRNBlock::getIo() {
  return io;
}
double PRNBlock::getCrc() {
  return crc;
}
double PRNBlock::getLilOmega() {
  return lilOmega;
}
double PRNBlock::getBigOmegaDot() {
  return bigOmegaDot;
}

double PRNBlock::getIdot() {
  return idot;
}
double PRNBlock::getCodesOnL2() {
  return codesOnL2;
}
double PRNBlock::getToeGPSWeek() {
  return toeGPSWeek;
}
double PRNBlock::getPDataFlagL2() {
  return pDataFlagL2;
}

double PRNBlock::getSvAccur() {
  return svAccur;
}
double PRNBlock::getSvHealth() {
  return svHealth;
}
double PRNBlock::getTgd() {
  return tgd;
}
double PRNBlock::getIodc() {
  return iodc;
}

double PRNBlock::getTransmTime() {
  return transmTime;
}
double PRNBlock::getFitInterval() {
  return fitInterval;
}
double PRNBlock::getSpare1() {
  return spare1;
}
double PRNBlock::getSpare2() {
  return spare2;
}

//=============== GlonassEphemEpoch Class =================================

GlonassEphemEpoch::GlonassEphemEpoch()  // Default Constructor
{
  satelliteAlmanacNumber = 0;
  epochYear = 9999;
  epochMonth = 1;
  epochDay = 1;
  epochHour = 0;
  epochMin = 0;
  epochSec = 0.0;
  svClockBias = 0.0;
  svRelFreqBias = 0.0;
  messageFrameTime = 0.0;
  posX = 0.0;
  velX = 0.0;
  accX = 0.0;
  svHealth = 0.0;
  posY = 0.0;
  velY = 0.0;
  accY = 0.0;
  freqNumber = 0.0;
  posZ = 0.0;
  velZ = 0.0;
  accZ = 0.0;
  ageOfOperation = 0.0;
}

GlonassEphemEpoch::~GlonassEphemEpoch()  // Destructor
{}

// Initializers
bool GlonassEphemEpoch::setSatelliteAlmanacNumber(unsigned short input) {
  if (input <= MAXPRNID) {
    satelliteAlmanacNumber = input;
    return true;
  } else {
    satelliteAlmanacNumber = 0;
    return false;
  }
}

bool GlonassEphemEpoch::setEpochYear(unsigned short input) {
  if (input >= 1980 && input <= 2079) {
    epochYear = input;
    return true;
  } else {
    epochYear = 9999;
    return false;
  }
}

bool GlonassEphemEpoch::setEpochMonth(unsigned short input) {
  if (input >= 1 && input <= 12) {
    epochMonth = input;
    return true;
  } else {
    epochMonth = 1;
    return false;
  }
}

bool GlonassEphemEpoch::setEpochDay(unsigned short input) {
  if (input >= 1 && input <= 31) {
    epochDay = input;
    return true;
  } else {
    epochDay = 1;
    return false;
  }
}

bool GlonassEphemEpoch::setEpochHour(unsigned short input) {
  if (input <= 24) {
    epochHour = input;
    return true;
  } else {
    epochHour = 0;
    return false;
  }
}

bool GlonassEphemEpoch::setEpochMin(unsigned short input) {
  if (input <= 60) {
    epochMin = input;
    return true;
  } else {
    epochMin = 0;
    return false;
  }
}

bool GlonassEphemEpoch::setEpochSec(double input) {
  if (input >= 0.0 && input <= 60.0) {
    epochSec = input;
    return true;
  } else {
    epochSec = 0.0;
    return false;
  }
}

bool GlonassEphemEpoch::setSvClockBias(double input) {
  svClockBias = input;
  return true;
}

bool GlonassEphemEpoch::setSvRelFreqBias(double input) {
  svRelFreqBias = input;
  return true;
}

bool GlonassEphemEpoch::setMessageFrameTime(double input) {
  messageFrameTime = input;
  return true;
}

bool GlonassEphemEpoch::setPosX(double input) {
  posX = input;
  return true;
}
bool GlonassEphemEpoch::setVelX(double input) {
  velX = input;
  return true;
}
bool GlonassEphemEpoch::setAccX(double input) {
  accX = input;
  return true;
}
bool GlonassEphemEpoch::setSvHealth(double input) {
  svHealth = input;
  return true;
}

bool GlonassEphemEpoch::setPosY(double input) {
  posY = input;
  return true;
}
bool GlonassEphemEpoch::setVelY(double input) {
  velY = input;
  return true;
}
bool GlonassEphemEpoch::setAccY(double input) {
  accY = input;
  return true;
}
bool GlonassEphemEpoch::setFreqNumber(double input) {
  freqNumber = input;
  return true;
}

bool GlonassEphemEpoch::setPosZ(double input) {
  posZ = input;
  return true;
}
bool GlonassEphemEpoch::setVelZ(double input) {
  velZ = input;
  return true;
}
bool GlonassEphemEpoch::setAccZ(double input) {
  accZ = input;
  return true;
}
bool GlonassEphemEpoch::setAgeOfOperation(double input) {
  ageOfOperation = input;
  return true;
}

// Selectors
unsigned short GlonassEphemEpoch::getSatelliteAlmanacNumber() {
  return satelliteAlmanacNumber;
}
unsigned short GlonassEphemEpoch::getEpochYear() {
  return epochYear;
}
unsigned short GlonassEphemEpoch::getEpochMonth() {
  return epochMonth;
}
unsigned short GlonassEphemEpoch::getEpochDay() {
  return epochDay;
}
unsigned short GlonassEphemEpoch::getEpochHour() {
  return epochHour;
}
unsigned short GlonassEphemEpoch::getEpochMin() {
  return epochMin;
}
double GlonassEphemEpoch::getEpochSec() {
  return epochSec;
}

double GlonassEphemEpoch::getSvClockBias() {
  return svClockBias;
}
double GlonassEphemEpoch::getSvRelFreqBias() {
  return svRelFreqBias;
}
double GlonassEphemEpoch::getMessageFrameTime() {
  return messageFrameTime;
}

double GlonassEphemEpoch::getPosX() {
  return posX;
}
double GlonassEphemEpoch::getVelX() {
  return velX;
}
double GlonassEphemEpoch::getAccX() {
  return accX;
}
double GlonassEphemEpoch::getSvHealth() {
  return svHealth;
}

double GlonassEphemEpoch::getPosY() {
  return posY;
}
double GlonassEphemEpoch::getVelY() {
  return velY;
}
double GlonassEphemEpoch::getAccY() {
  return accY;
}
double GlonassEphemEpoch::getFreqNumber() {
  return freqNumber;
}

double GlonassEphemEpoch::getPosZ() {
  return posZ;
}
double GlonassEphemEpoch::getVelZ() {
  return velZ;
}
double GlonassEphemEpoch::getAccZ() {
  return accZ;
}
double GlonassEphemEpoch::getAgeOfOperation() {
  return ageOfOperation;
}

//=============== GeostationaryEphemEpoch Class ===============================

GeostationaryEphemEpoch::GeostationaryEphemEpoch()  // Default Constructor
{
  satelliteNumber = 0;
  epochYear = 9999;
  epochMonth = 1;
  epochDay = 1;
  epochHour = 0;
  epochMin = 0;
  epochSec = 0.0;
  svClockBias = 0.0;
  svRelFreqBias = 0.0;
  messageFrameTime = 0.0;
  posX = 0.0;
  velX = 0.0;
  accX = 0.0;
  svHealth = 0.0;
  posY = 0.0;
  velY = 0.0;
  accY = 0.0;
  accurCode = 0.0;
  posZ = 0.0;
  velZ = 0.0;
  accZ = 0.0;
  spare = 0.0;
}

GeostationaryEphemEpoch::~GeostationaryEphemEpoch()  // Destructor
{}

// Initializers
bool GeostationaryEphemEpoch::setSatelliteNumber(unsigned short input) {
  if (input <= MAXPRNID) {
    satelliteNumber = input;
    return true;
  } else {
    satelliteNumber = 0;
    return false;
  }
}

bool GeostationaryEphemEpoch::setEpochYear(unsigned short input) {
  if (input >= 1980 && input <= 2079) {
    epochYear = input;
    return true;
  } else {
    epochYear = 9999;
    return false;
  }
}

bool GeostationaryEphemEpoch::setEpochMonth(unsigned short input) {
  if (input >= 1 && input <= 12) {
    epochMonth = input;
    return true;
  } else {
    epochMonth = 1;
    return false;
  }
}

bool GeostationaryEphemEpoch::setEpochDay(unsigned short input) {
  if (input >= 1 && input <= 31) {
    epochDay = input;
    return true;
  } else {
    epochDay = 1;
    return false;
  }
}

bool GeostationaryEphemEpoch::setEpochHour(unsigned short input) {
  if (input <= 24) {
    epochHour = input;
    return true;
  } else {
    epochHour = 0;
    return false;
  }
}

bool GeostationaryEphemEpoch::setEpochMin(unsigned short input) {
  if (input <= 60) {
    epochMin = input;
    return true;
  } else {
    epochMin = 0;
    return false;
  }
}

bool GeostationaryEphemEpoch::setEpochSec(double input) {
  if (input >= 0.0 && input <= 60.0) {
    epochSec = input;
    return true;
  } else {
    epochSec = 0.0;
    return false;
  }
}

bool GeostationaryEphemEpoch::setSvClockBias(double input) {
  svClockBias = input;
  return true;
}

bool GeostationaryEphemEpoch::setSvRelFreqBias(double input) {
  svRelFreqBias = input;
  return true;
}

bool GeostationaryEphemEpoch::setMessageFrameTime(double input) {
  messageFrameTime = input;
  return true;
}

bool GeostationaryEphemEpoch::setPosX(double input) {
  posX = input;
  return true;
}
bool GeostationaryEphemEpoch::setVelX(double input) {
  velX = input;
  return true;
}
bool GeostationaryEphemEpoch::setAccX(double input) {
  accX = input;
  return true;
}
bool GeostationaryEphemEpoch::setSvHealth(double input) {
  svHealth = input;
  return true;
}

bool GeostationaryEphemEpoch::setPosY(double input) {
  posY = input;
  return true;
}
bool GeostationaryEphemEpoch::setVelY(double input) {
  velY = input;
  return true;
}
bool GeostationaryEphemEpoch::setAccY(double input) {
  accY = input;
  return true;
}
bool GeostationaryEphemEpoch::setAccurCode(double input) {
  accurCode = input;
  return true;
}

bool GeostationaryEphemEpoch::setPosZ(double input) {
  posZ = input;
  return true;
}
bool GeostationaryEphemEpoch::setVelZ(double input) {
  velZ = input;
  return true;
}
bool GeostationaryEphemEpoch::setAccZ(double input) {
  accZ = input;
  return true;
}
bool GeostationaryEphemEpoch::setSpare(double input) {
  spare = input;
  return true;
}

// Selectors
unsigned short GeostationaryEphemEpoch::getSatelliteNumber() {
  return satelliteNumber;
}
unsigned short GeostationaryEphemEpoch::getEpochYear() {
  return epochYear;
}
unsigned short GeostationaryEphemEpoch::getEpochMonth() {
  return epochMonth;
}
unsigned short GeostationaryEphemEpoch::getEpochDay() {
  return epochDay;
}
unsigned short GeostationaryEphemEpoch::getEpochHour() {
  return epochHour;
}
unsigned short GeostationaryEphemEpoch::getEpochMin() {
  return epochMin;
}
double GeostationaryEphemEpoch::getEpochSec() {
  return epochSec;
}

double GeostationaryEphemEpoch::getSvClockBias() {
  return svClockBias;
}
double GeostationaryEphemEpoch::getSvRelFreqBias() {
  return svRelFreqBias;
}
double GeostationaryEphemEpoch::getMessageFrameTime() {
  return messageFrameTime;
}

double GeostationaryEphemEpoch::getPosX() {
  return posX;
}
double GeostationaryEphemEpoch::getVelX() {
  return velX;
}
double GeostationaryEphemEpoch::getAccX() {
  return accX;
}
double GeostationaryEphemEpoch::getSvHealth() {
  return svHealth;
}

double GeostationaryEphemEpoch::getPosY() {
  return posY;
}
double GeostationaryEphemEpoch::getVelY() {
  return velY;
}
double GeostationaryEphemEpoch::getAccY() {
  return accY;
}
double GeostationaryEphemEpoch::getAccurCode() {
  return accurCode;
}

double GeostationaryEphemEpoch::getPosZ() {
  return posZ;
}
double GeostationaryEphemEpoch::getVelZ() {
  return velZ;
}
double GeostationaryEphemEpoch::getAccZ() {
  return accZ;
}
double GeostationaryEphemEpoch::getSpare() {
  return spare;
}

//===================== HeaderRecord Class ====================================

// Constructors
HeaderRecord::HeaderRecord() {
  first60 = "";
  label = "";
}

HeaderRecord::HeaderRecord(string input)  // input is the entire header record
{
  first60 = input.substr(0, 60);
  label = input.substr(60, 20);
}

HeaderRecord::~HeaderRecord() {}

void HeaderRecord::SetHeaderRecord(string input) {
  first60 = input.substr(0, 60);
  label = input.substr(60, 20);
}

void HeaderRecord::SetFirst60(string input) {
  first60 = input.substr(0, 60);
}

void HeaderRecord::SetLabel(string input) {
  label = input.substr(0, 20);
}

string HeaderRecord::GetFirst60() {
  return (first60);
}

string HeaderRecord::GetLabel() {
  return (label);
}

ostream& operator<<(ostream& os, const HeaderRecord& input) {
  os << input.first60 << input.label << endl;
  return os;
}

istream& operator>>(istream& is, HeaderRecord& output) {
  string temp;
  getline(is, temp);
  output.SetHeaderRecord(temp);
  return is;
}

HeaderRecord& HeaderRecord::operator=(const HeaderRecord& input) {
  first60 = input.first60;
  label = input.label;
  return *this;
}

HeaderRecord* HeaderRecord::operator&(HeaderRecord input) {
  return &input;
}

//========================== RinexHeader Class ================================

// Constructors
RinexHeader::RinexHeader()  // default constructor
{}

RinexHeader::RinexHeader(list<HeaderRecord> inputImage)  // copy constructor
{
  // Copy an existing linked list of type HeaderRecord into headerImage.
  headerImage = inputImage;
}

// Destructor
RinexHeader::~RinexHeader() {}

// Initializers
void RinexHeader::appendHeaderRecord(HeaderRecord addedRec) {
  // Append addedRec to the end of the headerImage linked list.
  headerImage.push_back(addedRec);
}

void RinexHeader::insertHeaderRecBeforeLabel(
    string inputLabel, HeaderRecord addedRec) {
  // InsertaddedRec before the first occurance of inputLabel.
  list<HeaderRecord>::iterator iter = headerImage.begin();

  for (; iter != headerImage.end(); ++iter) {
    if (iter->GetLabel() == inputLabel.substr(0, iter->GetLabel().length())) {
      headerImage.insert(iter, addedRec);
      break;
    }
  }
}

void RinexHeader::insertHeaderRecAfterLabel(
    string inputLabel, HeaderRecord addedRec) {
  // InsertaddedRec after the first occurance of inputLabel.
  list<HeaderRecord>::iterator iter = headerImage.begin();

  for (; iter != headerImage.end(); ++iter) {
    if (iter->GetLabel() == inputLabel.substr(0, iter->GetLabel().length())) {
      headerImage.insert(iter++, addedRec);
      break;
    }
  }
}

void RinexHeader::overwriteHeaderRecord(string inputLabel, string newFirst60) {
  // Overwrite the first occurance of inputLabel using newFirst60.
  list<HeaderRecord>::iterator iter = headerImage.begin();

  for (; iter != headerImage.end(); ++iter) {
    if (iter->GetLabel() == inputLabel.substr(0, iter->GetLabel().length())) {
      iter->SetFirst60(newFirst60);
      break;
    }
  }
}

void RinexHeader::deleteHeaderRecord(string inputLabel) {
  // Delete the first occurance of the HeaderRecord matching inputLabel.
  // If no match is found in the headerImage linked list, nothing is done.
  list<HeaderRecord>::iterator iter = headerImage.begin();

  for (; iter != headerImage.end(); ++iter) {
    if (iter->GetLabel() == inputLabel.substr(0, iter->GetLabel().length())) {
      headerImage.erase(iter);
      break;
    }
  }
}

void RinexHeader::setHeaderImage(list<HeaderRecord> inputImage) {
  // Copy an existing linked list of type HeaderRecord into headerImage
  headerImage = inputImage;
}

// Selectors
HeaderRecord RinexHeader::getHeaderRecord(string inputLabel) {
  // Get the HeaderRecord corresponding to the first occurance of inputLabel.
  list<HeaderRecord>::iterator iter = headerImage.begin();

  for (; iter != headerImage.end(); ++iter) {
    if (iter->GetLabel() == inputLabel.substr(0, iter->GetLabel().length())) {
      return *iter;
    }
  }

  // If inputLabel is not found, return an empty HeaderRecord using
  // the default constructor for class HeaderRecord.
  HeaderRecord emptyHeaderRecord;
  return emptyHeaderRecord;
}

void RinexHeader::writeHeaderImage(ofstream& outputStream) {
  // Write headerImage to an output file using an ofstream.
  list<HeaderRecord>::iterator iter = headerImage.begin();
  for (; iter != headerImage.end(); ++iter) {
    outputStream << iter->GetFirst60() << iter->GetLabel() << endl;
  }
}

//========================== RinexFile Class ==================================

RinexFile::RinexFile()  // this is a base class
{
  pathFilename = "nofilename.out";
  fileMode = ios::out;
}

RinexFile::RinexFile(string inputFilePath, ios::openmode mode) {
  string record;
  pathFilename = inputFilePath;
  fileMode = mode;

  if (fileMode == ios::in) {
    inputStream.open(pathFilename.c_str(), fileMode);
    if (!inputStream) {
      tempStream << "Error: In RinexFile constructor, unable to open file:"
                 << endl
                 << pathFilename << " using mode: " << fileMode << endl;
      appendToErrorMessages(tempStream.str());

      RinexFileException excep(tempStream.str());
      throw excep;
    }
    readFileTypeAndProgramName();  // read the RINEX VERSION/TYPE and
                                   // PGM/RUN BY/DATE header records
    inputStream.close();
    inputStream.open(pathFilename.c_str(), fileMode);  // reset the file at
                                                       // its beginning.
    numberLinesRead = 0;
    numberWarnings = 0;
    numberErrors = 0;
  } else {
    outputStream.open(pathFilename.c_str(), fileMode);
    if (!outputStream) {
      tempStream << "Error: In RinexFile constructor, unable to open file:"
                 << endl
                 << pathFilename << " using mode: " << fileMode << endl;
      appendToErrorMessages(tempStream.str());

      RinexFileException excep(tempStream.str());
      throw excep;
    }
  }
}

// Destructor
RinexFile::~RinexFile() {
  if (fileMode == ios::in) {
    inputStream.close();
  } else {
    outputStream.close();
  }
  pathFilename = "nofilename.out";
  fileMode = ios::out;
}

// Initializers
void RinexFile::setPathFilenameMode(string inputFilePath, ios::openmode mode) {
  pathFilename = inputFilePath;
  fileMode = mode;

  if (fileMode == ios::in)  // input file
  {
    // aforster \{
    inputStream.close();
    // \}
    inputStream.open(pathFilename.c_str(), fileMode);
    if (!inputStream) {
      tempStream << "Error: In setPathFilenameMode, unable to open file:"
                 << endl
                 << pathFilename << " using mode: " << fileMode << endl;
      appendToErrorMessages(tempStream.str());

      RinexFileException excep(tempStream.str());
      throw excep;
    }
    readFileTypeAndProgramName();  // read the RINEX VERSION/TYPE and
                                   // PGM/RUN BY/DATE header records
    inputStream.close();
    inputStream.open(pathFilename.c_str(), fileMode);  // reset the file at
                                                       // its beginning.
    numberLinesRead = 0;
    numberWarnings = 0;
    numberErrors = 0;
  } else  // output file
  {
    outputStream.open(pathFilename.c_str(), fileMode);
    if (!outputStream) {
      tempStream << "Error: In setPathFilenameMode, unable to open file:"
                 << endl
                 << pathFilename << " using mode: " << fileMode << endl;
      appendToErrorMessages(tempStream.str());

      RinexFileException excep(tempStream.str());
      throw excep;
    }
  }
}

bool RinexFile::setRinexHeaderImage(list<HeaderRecord> input) {
  rinexHeaderImage.setHeaderImage(input);
  return true;
}

bool RinexFile::setFormatVersion(float input) {
  if (input > 0.99 && input < 2.11) {
    formatVersion = input;
    return true;
  } else {
    formatVersion = 9999;
    return false;
  }
}

bool RinexFile::setRinexFileType(string input) {
  rinexFileType = input[0];
  return true;
}

bool RinexFile::setSatSystem(string input) {
  satSystem = input[0];
  return true;
}

bool RinexFile::setRinexProgram(string input) {
  rinexProgram = input.substr(0, 20);
  return true;
}

bool RinexFile::setCreatedByAgency(string input) {
  createdByAgency = input.substr(0, 20);
  return true;
}

bool RinexFile::setDateFileCreated(string input) {
  dateFileCreated = input.substr(0, 20);
  return true;
}

bool RinexFile::incrementNumberErrors(unsigned long n) {
  numberErrors = numberErrors + n;
  return true;
}

bool RinexFile::incrementNumberWarnings(unsigned long n) {
  numberWarnings = numberWarnings + n;
  return true;
}

bool RinexFile::incrementNumberLinesRead(unsigned long n) {
  numberLinesRead = numberLinesRead + n;
  return true;
}

bool RinexFile::setCurrentEpoch(DateTime input) {
  DateTime Jan1_1980, Dec31_2079;

  Jan1_1980.SetYMDHMS(1980, 1, 1, 0, 0, 0.00000001);
  Dec31_2079.SetYMDHMS(2079, 12, 31, 23, 59, 59.99999999);

  if (input >= Jan1_1980 && input <= Dec31_2079) {
    currentEpoch = input;
    return true;
  } else {
    currentEpoch.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
    return false;
  }
}

void RinexFile::appendToErrorMessages(string errMessage) {
  string temp = "---------------------------------------";  // 39 chars

  errorMessages << temp << temp << endl << errMessage << endl;
  numberErrors++;

  // Since an error has occured, output all warnings and
  // errors that have been logged up to this point.
  cerr << "An error has occurred for file: " << getPathFilename() << endl
       << endl
       << "Here are the Warning Messages for this file:" << endl
       << getWarningMessages() << endl
       << endl
       << "Here are the Error Messages for this file:" << endl
       << getErrorMessages() << endl
       << endl;
}

void RinexFile::appendToWarningMessages(string warnMessage) {
  string tempd = "---------------------------------------";  // 39 chars
  const string resetStr = " \0";

  warningMessages << tempd << tempd << endl << warnMessage << endl;
  numberWarnings++;
  tempStream.str(resetStr);  // clear tempStream for use by the next warning
}

// Selectors
string RinexFile::getPathFilename() {
  return (pathFilename);
}

ios::openmode RinexFile::getFileMode() {
  return (fileMode);
}

RinexHeader RinexFile::getRinexHeaderImage() {
  return (rinexHeaderImage);
}

float RinexFile::getFormatVersion() {
  return formatVersion;
}
char RinexFile::getRinexFileType() {
  return rinexFileType;
}
char RinexFile::getSatSystem() {
  return satSystem;
}
string RinexFile::getRinexProgram() {
  return rinexProgram;
}
string RinexFile::getCreatedByAgency() {
  return createdByAgency;
}
string RinexFile::getDateFileCreated() {
  return dateFileCreated;
}
unsigned long RinexFile::getNumberErrors() {
  return numberErrors;
}
unsigned long RinexFile::getNumberWarnings() {
  return numberWarnings;
}
unsigned long RinexFile::getNumberLinesRead() {
  return numberLinesRead;
}
DateTime RinexFile::getCurrentEpoch() {
  return currentEpoch;
}

string RinexFile::getErrorMessages() {
  return (errorMessages.str());
}

string RinexFile::getWarningMessages() {
  return (warningMessages.str());
}

void RinexFile::readFileTypeAndProgramName() {
  string inputRec;
  string recordReadIn;
  bool endOfHeaderFound = false;

  if (!validFirstLine(recordReadIn)) {
    tempStream << "Error: First Line is incorrect in File: "
               << getPathFilename() << endl
               << recordReadIn << endl;
    appendToErrorMessages(tempStream.str());
    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  // read all Header lines from line 2 to END OF HEADER
  // until the PGM / RUN BY / DATE record is found
  while (!endOfHeaderFound) {
    if (!getline(inputStream, inputRec, '\n')) {
      // Error reading a header line of the RINEX File
      tempStream << "Error reading a header line of file:" << endl
                 << getPathFilename() << endl;
      appendToErrorMessages(tempStream.str());
      RinexReadingException excep(tempStream.str());
      throw excep;
    }

    if (blankString(inputRec)) {
      if (formatVersion < 2.0) {
        endOfHeaderFound = true;
        break;  // end the while loop
      } else {
        continue;  // go to read the next record
      }
    }

    if (inputRec.find("END OF HEADER") == 60) {
      endOfHeaderFound = true;
      break;
    } else if (inputRec.find("PGM / RUN BY / DATE") == 60) {
      rinexProgram = inputRec.substr(0, 20);
      createdByAgency = inputRec.substr(20, 20);
      dateFileCreated = inputRec.substr(40, 20);
      endOfHeaderFound = true;
      break;
    } else {
      continue;  // go to read the next record
    }

  }  // end of while loop over all header records
}

// private methods
bool RinexFile::blankString(string inputStr) {
  for (int i = 0; i < (unsigned short)inputStr.length(); i++) {
    if (inputStr[i] != ' ') {
      return false;
    }
  }
  return true;
}

void RinexFile::makeRecordLength80(string& inputRec) {
  string BlankString("                                        ");
  size_t slen;

  BlankString.append("                                        ");
  if (inputRec.length() > RINEXRECSIZE) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << " the Rinex record has " << inputRec.length()
               << " characters. This record will be truncated to 80 characters."
               << endl;
    appendToWarningMessages(tempStream.str());
    inputRec = inputRec.substr(0, 80);
  } else if (inputRec.length() > 80 && inputRec.length() <= RINEXRECSIZE) {
    inputRec = inputRec.substr(0, 80);
  } else if (inputRec.length() < 80) {
    slen = 80 - inputRec.length();
    inputRec.append(BlankString, 0, slen);
  }
}

void RinexFile::truncateHeaderRec(string& inputRec) {
  string BlankString("                                        ");
  size_t slen;

  BlankString.append("                                        ");
  if (inputRec.length() > RINEXRECSIZE) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << " the Rinex record has " << inputRec.length()
               << " characters. This record will be truncated to 80 characters."
               << endl;
    appendToWarningMessages(tempStream.str());
    inputRec = inputRec.substr(0, 80);
  } else if (inputRec.length() > 80 && inputRec.length() <= RINEXRECSIZE) {
    inputRec = inputRec.substr(0, 80);
  }
}

bool RinexFile::alphasInString(string inputStr) {
  for (int i = 0; i < (unsigned short)inputStr.length(); i++) {
    if (isalpha(inputStr[i])) {
      return true;
    }
  }
  return false;
}

bool RinexFile::validFirstLine(string& recordReadIn) {
  double tempD;
  string inputRec;
  string temp;

  if (getline(inputStream, inputRec, '\n')) {
    incrementNumberLinesRead(1);
    recordReadIn = inputRec;
    bool problemsFound = false;

    if (inputRec.find("RINEX VERSION / TYPE") == 60) {
      temp = inputRec.substr(0, 9);
      if (getDouble(temp, tempD))
        formatVersion = tempD;
      if (formatVersion < 1.0 || formatVersion > 2.1) {
        tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                   << inputRec << endl
                   << "Format version is  incorrect:" << temp << "." << endl;
        appendToWarningMessages(tempStream.str());
        problemsFound = true;
      }
      rinexFileType = inputRec.substr(20, 1)[0];
      if (rinexFileType != 'O' && rinexFileType != 'N' &&
          rinexFileType != 'M' && rinexFileType != 'G' &&
          rinexFileType != 'H' && rinexFileType != 'C') {
        tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                   << inputRec << endl
                   << "RINEX File Type is unacceptable:" << temp << "." << endl
                   << "File Type must be O, N, M, G, H, or C !" << endl;
        appendToWarningMessages(tempStream.str());
        problemsFound = true;
      }
      if (rinexFileType == 'O') {
        satSystem = inputRec.substr(40, 1)[0];
        // Satellite System "S" and "T" are currently not supported
        if (satSystem != 'G' && satSystem != ' ' && satSystem != 'R' &&
            satSystem != 'M') {
          tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                     << inputRec << endl
                     << "Satellite System is unacceptable:" << temp << "."
                     << endl
                     << "Satellite System must be G, R, M, or blank !" << endl
                     << "Sat System = >" << satSystem << "<  " << endl;
          appendToWarningMessages(tempStream.str());
          problemsFound = true;
        }
      }  // satSystem is only used for OBS files
    } else {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "First line is not RINEX VERSION / TYPE ." << endl;
      appendToWarningMessages(tempStream.str());
      problemsFound = true;
    }

    if (problemsFound)
      return false;
    else
      return true;
  } else {
    // Error reading the first line of the OBS File
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "Error reading the first line of file:" << getPathFilename()
               << endl;
    appendToErrorMessages(tempStream.str());

    RinexReadingException excep(tempStream.str());
    throw excep;
  }
}

bool RinexFile::getDouble(string input, double& output) {
  char* p;
  bool blank_string = true;

  output = strtod(input.c_str(), &p);
  int l = strlen(p);

  // HUGE_VAL should be defined in math.h, if not, define it in rinex.h
  if (output == HUGE_VAL) {
    tempStream << "On line #" << getNumberLinesRead() << "," << endl
               << "overflow error in getDouble() reading string: >" << input
               << "<" << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  } else if (l > 0) {
    for (int i = 0; i < l; i++)
      if (p[i] != ' ')
        blank_string = false;

    if (blank_string)
      return true;
    else {
      tempStream << "On line # " << getNumberLinesRead() << ","
                 << " warning from function RinexFile::getDouble()." << endl
                 << "Illegal characters found while reading string: >" << input
                 << "< " << endl
                 << "The illegal characters are >" << p << "< " << endl;
      appendToWarningMessages(tempStream.str());
      return false;
    }
  } else
    return true;
}

bool RinexFile::getLong(string input, long& output) {
  char* p;
  bool blank_string = true;

  output = strtol(input.c_str(), &p, 10);
  int l = strlen(p);

  // LONG_MAX should be defined in _lim.h, if not, define it in rinex.h
  if (output == LONG_MAX) {
    tempStream << "On line # " << getNumberLinesRead() << "," << endl
               << "overflow error in getLong() reading string: " << input
               << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }
  // LONG_MIN should be defined in _lim.h, if not, define it in rinex.h
  else if (output == LONG_MIN) {
    tempStream << "On line # " << getNumberLinesRead() << "," << endl
               << "underflow error in getLong() reading string: " << input
               << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  } else if (l > 0) {
    for (int i = 0; i < l; i++)
      if (p[i] != ' ')
        blank_string = false;

    if (blank_string)
      return true;
    else {
      tempStream << "On line # " << getNumberLinesRead() << ","
                 << " warning from function RinexFile::getLong()." << endl
                 << "Illegal characters found while reading string: >" << input
                 << "< " << endl
                 << "The illegal characters are >" << p << "< " << endl;
      appendToWarningMessages(tempStream.str());
      return false;
    }
  } else
    return true;
}

bool RinexFile::validYMDHMS(
    long year, long month, long day, long hour, long minute, double second,
    string& warningString) {
  bool problemsFound = false;
  int daysInMonth[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  warningString = "";

  if (year < 1980 || year > 2079) {
    warningString.append("The year is incorrect \n");
    problemsFound = true;
  }
  if (month < 1 || month > 12) {
    warningString.append("The month is incorrect \n");
    problemsFound = true;
  }
  if (day < 1 || day > daysInMonth[month - 1]) {
    warningString.append("The day is incorrect \n");
    problemsFound = true;
  }
  if (hour < 0 || hour > 24) {
    warningString.append("The hour is incorrect \n");
    problemsFound = true;
  }
  if (minute < 0 || minute > 60) {
    warningString.append("The minute is incorrect \n");
    problemsFound = true;
  }
  if (second < 0.0 || second > 60.00000001) {
    warningString.append("The seconds are incorrect \n");
    problemsFound = true;
  }

  if (problemsFound)
    return false;
  else
    return true;
}

//========================== RinexObsFile Class ===============================

// Initialize static data member
unsigned int RinexObsFile::numberObsFiles = 0;  // no objects yet

// Constructors
RinexObsFile::RinexObsFile()
    : RinexFile()  // calls the base class constructor
{
  initializeData();
  numberObsEpochs = 0;
  numberObsFiles++;
}

RinexObsFile::RinexObsFile(string inputFilePath, ios::openmode mode)
    : RinexFile(inputFilePath, mode) {
  initializeData();
  numberObsEpochs = 0;
  numberObsFiles++;
}

// Destructor
RinexObsFile::~RinexObsFile() {
  --numberObsFiles;
}

// Initilizers
bool RinexObsFile::setMarkerName(string input) {
  markerName = input.substr(0, 60);
  return true;
}

bool RinexObsFile::setMarkerNumber(string input) {
  markerNumber = input.substr(0, 20);
  return true;
}

bool RinexObsFile::setObserverName(string input) {
  observerName = input.substr(0, 20);
  return true;
}

bool RinexObsFile::setObserverAgency(string input) {
  observerAgency = input.substr(0, 40);
  return true;
}

bool RinexObsFile::setReceiverNumber(string input) {
  receiverNumber = input.substr(0, 20);
  return true;
}

bool RinexObsFile::setReceiverType(string input) {
  receiverType = input.substr(0, 20);
  return true;
}

bool RinexObsFile::setReceiverFirmwareVersion(string input) {
  receiverFirmwareVersion = input.substr(0, 20);
  return true;
}

bool RinexObsFile::setAntennaNumber(string input) {
  antennaNumber = input.substr(0, 20);
  return true;
}

bool RinexObsFile::setAntennaType(string input) {
  antennaType = input.substr(0, 20);
  return true;
}

bool RinexObsFile::setApproxX(double input) {
  if (fabs(input) < 6400000.0) {
    approxX = input;
    return true;
  } else {
    approxX = 0.0;
    return false;
  }
}

bool RinexObsFile::setApproxY(double input) {
  if (fabs(input) < 6400000.0) {
    approxY = input;
    return true;
  } else {
    approxY = 0.0;
    return false;
  }
}

bool RinexObsFile::setApproxZ(double input) {
  if (fabs(input) < 6400000.0) {
    approxZ = input;
    return true;
  } else {
    approxZ = 0.0;
    return false;
  }
}

bool RinexObsFile::setAntennaDeltaH(double input) {
  if (input > 0.0 && input < 6400000.0)  // No negative antenna heights
  {
    antennaDeltaH = input;
    return true;
  } else {
    antennaDeltaH = 0.0;
    return false;
  }
}

bool RinexObsFile::setAntennaDeltaE(double input) {
  if (fabs(input) < 6400000.0) {
    antennaDeltaE = input;
    return true;
  } else {
    antennaDeltaE = 0.0;
    return false;
  }
}

bool RinexObsFile::setAntennaDeltaN(double input) {
  if (fabs(input) < 6400000.0) {
    antennaDeltaN = input;
    return true;
  } else {
    antennaDeltaN = 0.0;
    return false;
  }
}

bool RinexObsFile::setDefWaveLenFactorL1(unsigned short input) {
  if (input < 3) {
    defWaveLenFactorL1 = input;
    return true;
  } else {
    defWaveLenFactorL1 = 9999;
    return false;
  }
}

bool RinexObsFile::setDefWaveLenFactorL2(unsigned short input) {
  if (input < 3) {
    defWaveLenFactorL2 = input;
    return true;
  } else {
    defWaveLenFactorL2 = 9999;
    return false;
  }
}

bool RinexObsFile::setNumWaveLenPRN(unsigned short input) {
  if (input < MAXPRNID) {
    numWaveLenPRN = input;
    return true;
  } else {
    numWaveLenPRN = 0;
    return false;
  }
}

bool RinexObsFile::setNumWaveLenRecords(unsigned short input) {
  if (input < MAXPRNID) {
    numWaveLenRecords = input;
    return true;
  } else {
    numWaveLenRecords = 0;
    return false;
  }
}

bool RinexObsFile::setAllWaveLenRecordsElement(OneWaveLenRec input, int i) {
  allWaveLenRecords[i] = input;
  return true;
}

bool RinexObsFile::setNumObsTypes(unsigned short input) {
  if (input <= MAXOBSTYPES) {
    numObsTypes = input;
    return true;
  } else {
    numObsTypes = 9999;
    return false;
  }
}

bool RinexObsFile::setObsTypeListElement(enum OBSTYPE input, int i) {
  obsTypeList[i] = input;
  return true;
}

bool RinexObsFile::setObsInterval(float input) {
  if (input > 0.0 && input <= 86400.0) {
    obsInterval = input;
    return true;
  } else {
    obsInterval = -9999;
    return false;
  }
}

bool RinexObsFile::setFirstObs(YMDHMS input) {
  DateTime Jan1_1980, Dec31_2079, inputDate;

  inputDate.SetYMDHMS(input);
  Jan1_1980.SetYMDHMS(1980, 1, 1, 0, 0, 0.00000001);
  Dec31_2079.SetYMDHMS(2079, 12, 31, 23, 59, 59.99999999);

  if (inputDate >= Jan1_1980 && inputDate <= Dec31_2079) {
    firstObs = input;
    return true;
  } else {
    inputDate.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
    firstObs = inputDate.GetYMDHMS();
    return false;
  }
}

bool RinexObsFile::setFirstObsTimeSystem(string input) {
  firstObsTimeSystem = input.substr(0, 3);
  return true;
}

bool RinexObsFile::setLastObs(YMDHMS input) {
  DateTime Jan1_1980, Dec31_2079, inputDate;

  inputDate.SetYMDHMS(input);
  Jan1_1980.SetYMDHMS(1980, 1, 1, 0, 0, 0.00000001);
  Dec31_2079.SetYMDHMS(2079, 12, 31, 23, 59, 59.99999999);

  if (inputDate >= Jan1_1980 && inputDate <= Dec31_2079) {
    lastObs = input;
    return true;
  } else {
    inputDate.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
    lastObs = inputDate.GetYMDHMS();
    return false;
  }
}

bool RinexObsFile::setLastObsTimeSystem(string input) {
  lastObsTimeSystem = input.substr(0, 3);
  return true;
}

bool RinexObsFile::setNumberLeapSec(unsigned short input) {
  numberLeapSec = input;
  return true;
}

bool RinexObsFile::setRcvrClockApplied(unsigned short input) {
  if (input == 0 || input == 1) {
    rcvrClockApplied = input;
    return true;
  } else {
    rcvrClockApplied = 9999;
    return true;
  }
}

bool RinexObsFile::setNumberOfSat(unsigned short input) {
  if (input <= MAXPRNID) {
    numberOfSat = input;
    return true;
  } else {
    numberOfSat = 9999;
    return true;
  }
}

bool RinexObsFile::setSatObsTypeListElement(ObsCountForPRN input, int i) {
  satObsTypeList[i] = input;
  return true;
}

bool RinexObsFile::setNextSat(unsigned short input) {
  nextSat = input;
  return true;
}

void RinexObsFile::incrementNumberObsEpochs(unsigned int n) {
  numberObsEpochs = numberObsEpochs + n;  // n is usually one
}

void RinexObsFile::initializeData() {
  //    Initializes the OBS private data with NULL values.  For strings NULL
  //    is an empty string.  For numeric values the NULL is chosen as a value
  //    that can be stored in the data type (short) but which is outside of the
  //    allowed range for the data element.  NULLs are typically all 9999s

  unsigned int i;
  unsigned int j;

  formatVersion = 0.0;
  rinexFileType = ' ';
  satSystem = ' ';
  rinexProgram = "";
  createdByAgency = "";
  dateFileCreated = "";

  markerName = "";
  markerNumber = "";
  observerName = "";
  observerAgency = "";
  receiverNumber = "";
  receiverType = "";
  receiverFirmwareVersion = "";
  antennaNumber = "";
  antennaType = "";
  approxX = 0.0;
  approxY = 0.0;
  approxZ = 0.0;
  antennaDeltaH = 0.0;
  antennaDeltaE = 0.0;
  antennaDeltaN = 0.0;
  defWaveLenFactorL1 = 9999;
  defWaveLenFactorL2 = 9999;
  numWaveLenPRN = 0;
  numWaveLenRecords = 0;
  numberLeapSec = 9999;
  rcvrClockApplied = 9999;

  for (i = 0; i < MAXPRNID; i++) {
    allWaveLenRecords[i].L1Factor = 9999;
    allWaveLenRecords[i].L2Factor = 9999;
    allWaveLenRecords[i].numSatInRecord = 0;
    for (j = 0; j < 7; j++)
      allWaveLenRecords[i].satsInRecord[j] = "";
  }

  numObsTypes = 9999;

  for (i = 0; i < MAXOBSTYPES; i++) {
    obsTypeList[i] = NOOBS;
  }

  obsInterval = -9999.0;
  firstObs.year = 9999;
  firstObs.month = 1;
  firstObs.day = 1;
  firstObs.hour = 0;
  firstObs.min = 0;
  firstObs.sec = 0.0;
  firstObsTimeSystem = "";
  lastObs.year = 9999;
  lastObs.month = 1;
  lastObs.day = 1;
  lastObs.hour = 0;
  lastObs.min = 0;
  lastObs.sec = 0.0;
  lastObsTimeSystem = "";
  numberOfSat = 9999;

  for (i = 0; i < MAXPRNID; i++) {
    satObsTypeList[i].satCode = ' ';

    satObsTypeList[i].satNum = 9999;

    for (j = 0; j < MAXOBSTYPES; j++) {
      satObsTypeList[i].PRNObsCount[j] = 9999;
    }
  }

  for (i = 0; i < MAXOBSHEADERRECTYPES; i++) {
    headerRecs[i].numberPresent = 0;
  }

  headerRecs[0].recID = "RINEX VERSION / TYPE";
  headerRecs[0].required = true;

  headerRecs[1].recID = "PGM / RUN BY / DATE";
  headerRecs[1].required = true;

  headerRecs[2].recID = "COMMENT";
  headerRecs[2].required = false;

  headerRecs[3].recID = "MARKER NAME";
  headerRecs[3].required = true;

  headerRecs[4].recID = "MARKER NUMBER";
  headerRecs[4].required = false;

  headerRecs[5].recID = "OBSERVER / AGENCY";
  headerRecs[5].required = true;

  headerRecs[6].recID = "REC # / TYPE / VERS";
  headerRecs[6].required = true;

  headerRecs[7].recID = "ANT # / TYPE";
  headerRecs[7].required = true;

  headerRecs[8].recID = "APPROX POSITION XYZ";
  headerRecs[8].required = true;

  headerRecs[9].recID = "ANTENNA: DELTA H/E/N";
  headerRecs[9].required = true;

  headerRecs[10].recID = "WAVELENGTH FACT L1/2";
  headerRecs[10].required = true;

  headerRecs[11].recID = "# / TYPES OF OBSERV";
  headerRecs[11].required = true;

  headerRecs[12].recID = "INTERVAL";
  headerRecs[12].required = false;

  headerRecs[13].recID = "TIME OF FIRST OBS";
  headerRecs[13].required = true;

  headerRecs[14].recID = "TIME OF LAST OBS";
  headerRecs[14].required = false;

  headerRecs[15].recID = "RCV CLOCK OFFS APPL";
  headerRecs[15].required = false;

  headerRecs[16].recID = "LEAP SECONDS";
  headerRecs[16].required = false;

  headerRecs[17].recID = "# OF SATELLITES";
  headerRecs[17].required = false;

  headerRecs[18].recID = "PRN / # OF OBS";
  headerRecs[18].required = false;

  headerRecs[19].recID = "END OF HEADER";
  headerRecs[19].required = true;

  nextSat = 0;
}

unsigned short RinexObsFile::readHeader() {
  unsigned short i;
  unsigned short headerRecordFlag;
  string inputRec;
  string recordReadIn;
  HeaderRecord headerRec;
  ostringstream sstemp;
  bool endOfHeaderFound = false;
  unsigned long numberRequiredErrors = 0;
  enum { VersionRec };

  if (validFirstLine(recordReadIn)) {
    headerRecs[VersionRec].numberPresent++;
    headerRec.SetHeaderRecord(recordReadIn);
    rinexHeaderImage.appendHeaderRecord(recordReadIn);
  } else {
    tempStream << "Error: First Line is incorrect in File: "
               << getPathFilename() << endl
               << recordReadIn << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  // read all Header lines from line 2 until the end of header
  while (!endOfHeaderFound) {
    if (!getline(inputStream, inputRec, '\n')) {
      // Error reading a header line of the OBS File
      tempStream << "Error reading a header line of file:" << endl
                 << getPathFilename() << endl;
      appendToErrorMessages(tempStream.str());

      RinexReadingException excep(tempStream.str());
      throw excep;
    }
    incrementNumberLinesRead(1);

    if (blankString(inputRec)) {
      if (formatVersion < 2.0) {
        endOfHeaderFound = true;
        headerRec.SetHeaderRecord(inputRec);
        rinexHeaderImage.appendHeaderRecord(headerRec);
        break;  // end the while loop
      } else {
        tempStream << "Warning! On line #" << getNumberLinesRead() << ":"
                   << endl
                   << inputRec << endl
                   << " Blank header record found, skipping to next line. "
                   << endl;
        appendToWarningMessages(tempStream.str());
        continue;  // go to read the next record
      }
    }

    if (validHeaderRecord(inputRec)) {
      headerRec.SetHeaderRecord(inputRec);
      rinexHeaderImage.appendHeaderRecord(headerRec);
      if (inputRec.find("END OF HEADER") == 60) {
        endOfHeaderFound = true;
        break;
      }
      continue;  // go to read the next record
    }

    if (validEventFlagRecord(inputRec)) {
      tempStream
          << "Warning! On line #" << getNumberLinesRead() << ":" << endl
          << inputRec << endl
          << "EPOCH/SAT Flag Record found in Header, skipping to next line."
          << endl
          << "Check file for location of END OF HEADER record." << endl;
      appendToWarningMessages(tempStream.str());
      continue;  // go to read the next record
    }

    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "Unknown Record Type found in Header, skipping to next line."
               << endl;
    appendToWarningMessages(tempStream.str());

  }  // end of while loop over all header records

  // Check for the presence of all manditory header records.

  for (i = 0; i < MAXOBSHEADERRECTYPES; i++) {
    if (headerRecs[i].numberPresent == 0 && headerRecs[i].required) {
      sstemp << " missing Header record: " << headerRecs[i].recID << endl;
      numberRequiredErrors++;
    }
  }
  if (numberRequiredErrors > 0) {
    tempStream << getPathFilename() << " has the following missing Header "
               << "records." << endl
               << sstemp.str() << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }
  return (0);
}

bool RinexObsFile::validHeaderRecord(string inputRec) {
  unsigned short i;
  long tempL;
  double tempD;
  double distance;
  string temp;
  string warningString;
  unsigned short L1_fac, L2_fac, num_sat;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    MarkerNameRec,
    MarkerNumRec,
    ObserverRec,
    ReceiverRec,
    AntennaRec,
    ApproxPosRec,
    AntennaDeltaRec,
    WavelengthRec,
    ObsNumTypesRec,
    IntervalRec,
    FirstObsTimeRec,
    LastObsTimeRec,
    RcvClkApplRec,
    LeapSecRec,
    NumOfSatRec,
    PRNRec,
    EndHeaderRec
  };

  if (inputRec.find("COMMENT") == 60) {
    // all comment records are stored in the headerImage linked-list
    headerRecs[CommentRec].numberPresent++;
    return true;
  } else if (inputRec.find("PGM / RUN BY / DATE") == 60) {
    headerRecs[PgmRunByRec].numberPresent++;

    rinexProgram = inputRec.substr(0, 20);
    createdByAgency = inputRec.substr(20, 20);
    dateFileCreated = inputRec.substr(40, 20);
    return true;
  } else if (inputRec.find("MARKER NAME") == 60) {
    headerRecs[MarkerNameRec].numberPresent++;

    markerName = inputRec.substr(0, 60);
    return true;
  } else if (inputRec.find("MARKER NUMBER") == 60) {
    headerRecs[MarkerNumRec].numberPresent++;

    markerNumber = inputRec.substr(0, 20);
    return true;
  } else if (inputRec.find("OBSERVER / AGENCY") == 60) {
    headerRecs[ObserverRec].numberPresent++;

    observerName = inputRec.substr(0, 20);
    observerAgency = inputRec.substr(20, 40);
    return true;
  } else if (inputRec.find("REC # / TYPE / VERS") == 60) {
    headerRecs[ReceiverRec].numberPresent++;

    receiverNumber = inputRec.substr(0, 20);
    receiverType = inputRec.substr(20, 20);
    receiverFirmwareVersion = inputRec.substr(40, 20);
    return true;
  } else if (inputRec.find("ANT # / TYPE") == 60) {
    headerRecs[AntennaRec].numberPresent++;

    antennaNumber = inputRec.substr(0, 20);
    antennaType = inputRec.substr(20, 20);
    return true;
  } else if (inputRec.find("APPROX POSITION XYZ") == 60) {
    headerRecs[ApproxPosRec].numberPresent++;

    temp = inputRec.substr(0, 14);
    if (getDouble(temp, tempD))
      approxX = tempD;
    temp = inputRec.substr(14, 14);
    if (getDouble(temp, tempD))
      approxY = tempD;
    temp = inputRec.substr(28, 14);
    if (getDouble(temp, tempD))
      approxZ = tempD;
    distance = sqrt(approxX * approxX + approxY * approxY + approxZ * approxZ);
    // Check that the station coordinates make sense ...
    if (distance < 6000000.0 || distance > 6400000.0) {
      tempStream
          << "On line #" << getNumberLinesRead() << ":" << endl
          << inputRec << endl
          << "Approx Station coordinates are incorrect." << endl
          << " Distance of XYZ coords from origin = " << distance << endl
          << "6,000,000.0 < Allowed Distance from earth center < 6,400,000.0 "
          << " meters." << endl;
      appendToWarningMessages(tempStream.str());
    }
    return true;
  } else if (inputRec.find("ANTENNA: DELTA H/E/N") == 60) {
    headerRecs[AntennaDeltaRec].numberPresent++;

    temp = inputRec.substr(0, 14);
    if (getDouble(temp, tempD))
      antennaDeltaH = tempD;
    temp = inputRec.substr(14, 14);
    if (getDouble(temp, tempD))
      antennaDeltaE = tempD;
    temp = inputRec.substr(28, 14);
    if (getDouble(temp, tempD))
      antennaDeltaN = tempD;
    return true;
  } else if (inputRec.find("WAVELENGTH FACT L1/2") == 60) {
    string temp;
    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      L1_fac = static_cast<unsigned short>(tempL);
    temp = inputRec.substr(6, 6);
    if (getLong(temp, tempL))
      L2_fac = static_cast<unsigned short>(tempL);
    temp = inputRec.substr(12, 6);
    if (getLong(temp, tempL))
      num_sat = static_cast<unsigned short>(tempL);

    if (L1_fac > 2) {
      tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << " a bad L1 Wavelength factor was found: " << L1_fac << endl
                 << " The program will set L1_fac = 1 ." << endl;
      L1_fac = 1;
      appendToWarningMessages(tempStream.str());
    }
    if (L2_fac > 2) {
      tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << " a bad L2 Wavelength factor was found: " << L2_fac << endl
                 << " The program will set L2_fac = 1 ." << endl;
      L2_fac = 1;
      appendToWarningMessages(tempStream.str());
    }

    if (num_sat == 0) {
      defWaveLenFactorL1 = L1_fac;
      defWaveLenFactorL2 = L2_fac;
      numWaveLenPRN = 0;
      headerRecs[WavelengthRec].numberPresent++;
    }

    if (num_sat > 0) {
      numWaveLenPRN = (unsigned short)(numWaveLenPRN + num_sat);
      headerRecs[WavelengthRec].numberPresent++;
      allWaveLenRecords[numWaveLenRecords].L1Factor = L1_fac;
      allWaveLenRecords[numWaveLenRecords].L2Factor = L2_fac;
      allWaveLenRecords[numWaveLenRecords].numSatInRecord = num_sat;
      for (i = 0; i < num_sat; i++) {
        allWaveLenRecords[numWaveLenRecords].satsInRecord[i] =
            inputRec.substr(18 + (i * 6), 6);
      }
      numWaveLenRecords++;
    }
    return true;
  } else if (inputRec.find("# / TYPES OF OBSERV") == 60) {
    string obsType;
    string temp;

    headerRecs[ObsNumTypesRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      numObsTypes = static_cast<unsigned short>(tempL);

    temp = inputRec.substr(6, 54);
    istringstream nextObs(temp);

    for (i = 0; i < MAXOBSTYPES; i++) {
      obsTypeList[i] = NOOBS;
    }

    i = 0;
    while (nextObs >> obsType) {
      if (obsType.find("L1") != string::npos)
        obsTypeList[i] = L1;
      else if (obsType.find("L2") != string::npos)
        obsTypeList[i] = L2;
      else if (obsType.find("C1") != string::npos)
        obsTypeList[i] = C1;
      else if (obsType.find("P1") != string::npos)
        obsTypeList[i] = P1;
      else if (obsType.find("P2") != string::npos)
        obsTypeList[i] = P2;
      else if (obsType.find("D1") != string::npos)
        obsTypeList[i] = D1;
      else if (obsType.find("D2") != string::npos)
        obsTypeList[i] = D2;
      else if (obsType.find("T1") != string::npos)
        obsTypeList[i] = T1;
      else if (obsType.find("T2") != string::npos)
        obsTypeList[i] = T2;
      else if (obsType.find("S1") != string::npos)
        obsTypeList[i] = S1;
      else if (obsType.find("S2") != string::npos)
        obsTypeList[i] = S2;
      else {
        tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                   << inputRec << endl
                   << "Warning! Unrecognized Observation Type: " << obsType
                   << endl;
        appendToWarningMessages(tempStream.str());
      }
      i++;
    }
    return true;
  } else if (inputRec.find("INTERVAL") == 60) {
    headerRecs[IntervalRec].numberPresent++;

    temp = inputRec.substr(0, 10);
    if (getDouble(temp, tempD))
      obsInterval = tempD;
    if (obsInterval < 0.0001) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Negative Observation Interval: " << obsInterval << endl;
      appendToWarningMessages(tempStream.str());
    }
    return true;
  } else if (inputRec.find("LEAP SECONDS") == 60) {
    headerRecs[LeapSecRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      numberLeapSec = static_cast<unsigned short>(tempL);
    return true;
  } else if (inputRec.find("RCV CLOCK OFFS APPL") == 60) {
    headerRecs[RcvClkApplRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      rcvrClockApplied = static_cast<unsigned short>(tempL);
    // 1 = Yes, 0 = No
    if (rcvrClockApplied != 0 && rcvrClockApplied != 1) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Error reading RCV CLOCK OFFS APPL flag: "
                 << rcvrClockApplied << endl;
      appendToWarningMessages(tempStream.str());
    }
    return true;
  } else if (inputRec.find("TIME OF FIRST OBS") == 60) {
    headerRecs[FirstObsTimeRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      firstObs.year = tempL;

    temp = inputRec.substr(6, 6);
    if (getLong(temp, tempL))
      firstObs.month = tempL;

    temp = inputRec.substr(12, 6);
    if (getLong(temp, tempL))
      firstObs.day = tempL;

    temp = inputRec.substr(18, 6);
    if (getLong(temp, tempL))
      firstObs.hour = tempL;

    temp = inputRec.substr(24, 6);
    if (getLong(temp, tempL))
      firstObs.min = tempL;

    temp = inputRec.substr(30, 13);
    if (getDouble(temp, tempD))
      firstObs.sec = tempD;

    if (!validYMDHMS(
            firstObs.year, firstObs.month, firstObs.day, firstObs.hour,
            firstObs.min, firstObs.sec, warningString)) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Problems reading Date and Time of First Obs: " << endl
                 << warningString << endl;
      appendToWarningMessages(tempStream.str());
    }

    firstObsTimeSystem = inputRec.substr(48, 3);
    if (firstObsTimeSystem != "GPS" && firstObsTimeSystem != "GLO" &&
        firstObsTimeSystem != "   ") {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Unrecognized Time System for First Obs: "
                 << firstObsTimeSystem << endl;
      appendToWarningMessages(tempStream.str());
    }
    // Check for years that are not four digits

    if (firstObs.year < 1000) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Warning! Year in " << headerRecs[FirstObsTimeRec].recID
                 << " record is not four digits: " << firstObs.year << "."
                 << endl;
      appendToWarningMessages(tempStream.str());

      firstObs.year = firstObs.year % 100;

      if (firstObs.year > 79 && firstObs.year <= 99)
        firstObs.year = firstObs.year + 1900;
      if (firstObs.year >= 0 && firstObs.year <= 79)
        firstObs.year = firstObs.year + 2000;
    }
    return true;

  } else if (inputRec.find("TIME OF LAST OBS") == 60) {
    headerRecs[LastObsTimeRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      lastObs.year = tempL;

    temp = inputRec.substr(6, 6);
    if (getLong(temp, tempL))
      lastObs.month = tempL;

    temp = inputRec.substr(12, 6);
    if (getLong(temp, tempL))
      lastObs.day = tempL;

    temp = inputRec.substr(18, 6);
    if (getLong(temp, tempL))
      lastObs.hour = tempL;

    temp = inputRec.substr(24, 6);
    if (getLong(temp, tempL))
      lastObs.min = tempL;

    temp = inputRec.substr(30, 13);
    if (getDouble(temp, tempD))
      lastObs.sec = tempL;

    if (!validYMDHMS(
            lastObs.year, lastObs.month, lastObs.day, lastObs.hour, lastObs.min,
            lastObs.sec, warningString)) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Problems reading Date and Time of Last Obs: "
                 << warningString << endl;
      appendToWarningMessages(tempStream.str());
    }

    lastObsTimeSystem = inputRec.substr(48, 3);
    if (lastObsTimeSystem != "GPS" && lastObsTimeSystem != "GLO" &&
        lastObsTimeSystem != "   ") {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Unrecognized Time System for Last Obs: "
                 << lastObsTimeSystem << endl;
      appendToWarningMessages(tempStream.str());
    }
    // Check for years that are not four digits

    if (lastObs.year < 1000) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Warning! Year in " << headerRecs[LastObsTimeRec].recID
                 << " record is not four digits: " << lastObs.year << "."
                 << endl;
      appendToWarningMessages(tempStream.str());

      lastObs.year = lastObs.year % 100;

      if (lastObs.year > 79 && lastObs.year <= 99)
        lastObs.year = lastObs.year + 1900;
      if (lastObs.year >= 0 && lastObs.year <= 79)
        lastObs.year = lastObs.year + 2000;
    }
    return true;
  } else if (inputRec.find("# OF SATELLITES") == 60) {
    headerRecs[NumOfSatRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      numberOfSat = static_cast<unsigned short>(tempL);
    return true;
  } else if (inputRec.find("PRN / # OF OBS") == 60) {
    headerRecs[PRNRec].numberPresent++;

    if (nextSat < MAXPRNID) {
      temp = inputRec.substr(3, 1);
      satObsTypeList[nextSat].satCode = temp[0];

      temp = inputRec.substr(4, 2);
      if (getLong(temp, tempL))
        satObsTypeList[nextSat].satNum = static_cast<unsigned short>(tempL);

      for (i = 0; i < MAXOBSTYPES; i++) {
        temp = inputRec.substr((6 + i * 6), 6);

        if (temp == "      ") {
          break;
        }
        if (getLong(temp, tempL))
          satObsTypeList[nextSat].PRNObsCount[i] =
              static_cast<unsigned long>(tempL);
      }
    } else {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Number of PRN / # OF OBS records exceeds program" << endl
                 << "limit for maximum number of satellites. Record ignored."
                 << endl;
      appendToWarningMessages(tempStream.str());
    }
    nextSat++;
    return true;
  } else if (inputRec.find("END OF HEADER") == 60) {
    headerRecs[EndHeaderRec].numberPresent++;
    return true;
  } else {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << " there is an invalid header record." << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }
}

bool RinexObsFile::validEventFlagRecord(string inputRec) {
  // Note: an Event Flag record may have only 2 fields filled: the event flag
  // and the number of records to follow.

  string warningString;
  string cols71thru80;
  string cols2thru26;
  string temp;
  YMDHMS ymdhms;  // defined in datetime.h
  long tempL, col29 = -1;
  double tempD;

  makeRecordLength80(inputRec);

  cols71thru80 = inputRec.substr(70, 10);
  // there should be no letters in the rcvr clk offset field
  if (alphasInString(cols71thru80)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << " letters were found the Receiver Clock Offset field."
               << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }

  if (isdigit(inputRec[28])) {
    if (getLong(temp, tempL))
      col29 = tempL;
  }

  // The columns bewteen the YMDHMS values should always be blanks
  if (inputRec[0] != ' ' || inputRec[3] != ' ' || inputRec[6] != ' ' ||
      inputRec[9] != ' ' || inputRec[12] != ' ' || inputRec[15] != ' ' ||
      inputRec[26] != ' ' || inputRec[27] != ' ' || col29 < 0 || col29 > 6) {
    tempStream << " On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << " a formatting error was found in columns 1 through 29."
               << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }

  // Check the YMDHMS values if they are present
  cols2thru26 = inputRec.substr(1, 25);
  if (!blankString(cols2thru26)) {
    temp = inputRec.substr(1, 2);
    if (getLong(temp, tempL))
      ymdhms.year = tempL;
    if (ymdhms.year >= 80 && ymdhms.year <= 99)
      ymdhms.year = ymdhms.year + 1900;
    if (ymdhms.year >= 0 && ymdhms.year <= 79)
      ymdhms.year = ymdhms.year + 2000;
    temp = inputRec.substr(4, 2);
    if (getLong(temp, tempL))
      ymdhms.month = tempL;
    temp = inputRec.substr(7, 2);
    if (getLong(temp, tempL))
      ymdhms.day = tempL;
    temp = inputRec.substr(10, 2);
    if (getLong(temp, tempL))
      ymdhms.hour = tempL;
    temp = inputRec.substr(13, 2);
    if (getLong(temp, tempL))
      ymdhms.min = tempL;
    temp = inputRec.substr(15, 11);
    if (getDouble(temp, tempD))
      ymdhms.sec = tempD;
    if (!validYMDHMS(
            ymdhms.year, ymdhms.month, ymdhms.day, ymdhms.hour, ymdhms.min,
            ymdhms.sec, warningString)) {
      tempStream << " On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << warningString << endl;
      appendToWarningMessages(tempStream.str());
      return false;
    }
  }  // do only when a date/time is present in the Event Flag record

  return true;  // the input EPOCH/SAT/EVENT FLAG record is OK
}

bool RinexObsFile::validObservationsRecord(string inputRec) {
  double obs[5];
  string temp;
  double tempD;

  // Note: missing observations can cause a blank observation line
  if (blankString(inputRec))
    return true;

  // No letter-characters should be present in an observations record
  if (alphasInString(inputRec)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "Illegal alphabetic characters found in Data record." << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }

  makeRecordLength80(inputRec);

  if (inputRec[10] != '.' && inputRec[26] != '.' && inputRec[42] != '.' &&
      inputRec[58] != '.' && inputRec[74] != '.') {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "No decimal points found in Data record." << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }

  for (int i = 0; i < 5; i++) {
    temp = inputRec.substr(i * 16, 14);
    if (getDouble(temp, tempD))
      obs[i] = tempD;
    if (obs[i] > 9999999999.999 || obs[i] < -999999999.999)  // F14.3 field
    {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Illegal observation found in Data record: " << obs[i]
                 << endl;
      appendToWarningMessages(tempStream.str());
      return false;
    }
  }

  return true;
}

void RinexObsFile::writeHeaderImage(ofstream& outputStream) {
  rinexHeaderImage.writeHeaderImage(outputStream);
}

unsigned short RinexObsFile::readEpoch(ObsEpoch& inputEpoch) {
  unsigned short tempUS;
  ostringstream stringStream;
  unsigned short appendLen;
  string saveSatCode = "";
  unsigned short saveSatNum[MAXSATPEREPOCH];
  unsigned short i;
  string inputRec;
  short j;
  size_t slen;
  string temp;
  string tempLLI;
  string tempSigStrength;
  string working;
  string cols2thru26;
  YMDHMS ymdhms;
  DateTime tempDateTime;
  SatObsAtEpoch tempSatObsAtEpoch;
  double tempDouble, tempD;
  long tempL;
  bool eventFlagRecordOK = false;
  bool timeTagOK = false;
  inputEpoch.initializeData();

  // Find the next good "time-tag"/Event Flag record.
  while (!eventFlagRecordOK) {
    if (!getline(inputStream, inputRec, '\n')) {
      return (0);  // return 0 when end of file is encountered
    }
    incrementNumberLinesRead(1);
    makeRecordLength80(inputRec);  // missing fields have been filled in

    if (validEventFlagRecord(inputRec)) {
      eventFlagRecordOK = true;

      // Check the YMDHMS values if they are present
      cols2thru26 = inputRec.substr(1, 25);
      if (!blankString(cols2thru26)) {
        timeTagOK = true;

        temp = inputRec.substr(1, 2);
        if (temp == "  ") {
          tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                     << inputRec << endl
                     << "Warning ! Year in time tag is missing." << endl;
          appendToWarningMessages(tempStream.str());
          timeTagOK = false;
        }
        if (getLong(temp, tempL))
          ymdhms.year = tempL;
        if (ymdhms.year >= 80 && ymdhms.year <= 99)
          ymdhms.year = ymdhms.year + 1900;
        if (ymdhms.year >= 0 && ymdhms.year <= 79)
          ymdhms.year = ymdhms.year + 2000;

        temp = inputRec.substr(4, 2);
        if (temp == "  ") {
          tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                     << inputRec << endl
                     << "Warning ! Month in time tag is missing." << endl;
          appendToWarningMessages(tempStream.str());
          timeTagOK = false;
        }
        if (getLong(temp, tempL))
          ymdhms.month = tempL;

        temp = inputRec.substr(7, 2);
        if (temp == "  ") {
          tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                     << inputRec << endl
                     << "Warning ! Day in time tag is missing." << endl;
          appendToWarningMessages(tempStream.str());
          timeTagOK = false;
        }
        if (getLong(temp, tempL))
          ymdhms.day = tempL;

        temp = inputRec.substr(10, 2);
        if (temp == "  ") {
          tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                     << inputRec << endl
                     << "Warning ! Hour in time tag is missing." << endl;
          appendToWarningMessages(tempStream.str());
          timeTagOK = false;
        }
        if (getLong(temp, tempL))
          ymdhms.hour = tempL;

        temp = inputRec.substr(13, 2);
        if (temp == "  ") {
          tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                     << inputRec << endl
                     << "Warning ! Minute in time tag is missing." << endl;
          appendToWarningMessages(tempStream.str());
          timeTagOK = false;
        }
        if (getLong(temp, tempL))
          ymdhms.min = tempL;

        temp = inputRec.substr(15, 11);
        if (temp == "           ") {
          tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                     << inputRec << endl
                     << "Warning ! Seconds in time tag is missing." << endl;
          appendToWarningMessages(tempStream.str());
          timeTagOK = false;
        }
        if (getDouble(temp, tempD))
          ymdhms.sec = tempD;
        // Note: these ymdhms values have already
        // been validated in validEventFlagRecord()
        if (timeTagOK) {
          tempDateTime.SetYMDHMS(ymdhms);
          inputEpoch.setEpochTime(tempDateTime);
          ymdhms = inputEpoch.getEpochTime().GetYMDHMS();
        }
      }  // do this only if there is a time tag in columns 2 through 26

      temp = inputRec.substr(26, 3);
      if (getLong(temp, tempL))
        tempUS = static_cast<unsigned short>(tempL);
      if (!inputEpoch.setEpochFlag(tempUS)) {
        tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                   << inputRec << endl
                   << "   Bad epoch flag encountered: " << temp << "." << endl;
        appendToWarningMessages(tempStream.str());
      }

      temp = inputRec.substr(29, 3);
      if (getLong(temp, tempL))
        tempUS = static_cast<unsigned short>(tempL);
      if (!inputEpoch.setNumSat(tempUS)) {
        tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                   << inputRec << endl
                   << "Warning ! More than 999 satellites/#lines found."
                   << endl;
        appendToWarningMessages(tempStream.str());
      }

      break;  // exit this while loop once a good EPOCH/SAT record has been
              // found
    } else {
      tempStream << "Warning ! A valid EPOCH/SAT record has not been found yet:"
                 << endl
                 << inputRec << endl
                 << "Now searching ahead for the next good time tag line."
                 << endl;
      appendToWarningMessages(tempStream.str());
    }
  }  // end of while loop to search for next good Time Tag/Event Flag Record

  // Read event records.
  if (inputEpoch.getEpochFlag() >= 2 && inputEpoch.getEpochFlag() <= 5) {
    tempStream << "On line # " << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "Warning! An Event Flag has been found of type: "
               << inputEpoch.getEpochFlag() << "." << endl
               << inputEpoch.getEpochHeaderRecords();

    for (i = 0; i < inputEpoch.getNumSat(); i++) {
      if (!getline(inputStream, inputRec, '\n')) {
        // Error reading records following an Event Flag
        tempStream << "Error reading records after Event Flag in file:" << endl
                   << getPathFilename() << endl;
        appendToErrorMessages(tempStream.str());

        RinexReadingException excep(tempStream.str());
        throw excep;
      }
      incrementNumberLinesRead(1);
      makeRecordLength80(inputRec);
      inputEpoch.appendToEpochHeaderRecords(inputRec);
      tempStream << inputRec << endl;    // append epoch header records to warn.
      if (!validHeaderRecord(inputRec))  // update private data in RinexObsFile
      {
        tempStream << "Warning ! On line #" << getNumberLinesRead() << ":"
                   << endl
                   << inputRec << endl
                   << "  an invalid epoch header record was found." << endl;
        appendToWarningMessages(tempStream.str());
      }
    }  // end of loop over all records following event flag 2,3,4,5
    appendToWarningMessages(tempStream.str());

  }  // do the above when event flags 2,3,4,5 are encountered

  // Read the Observations
  if (inputEpoch.getEpochFlag() == 0 || inputEpoch.getEpochFlag() == 1 ||
      inputEpoch.getEpochFlag() == 6) {
    temp = inputRec.substr(68, 12);
    if (getDouble(temp, tempD)) {
      if (!inputEpoch.setRecClockOffset(tempD)) {
        tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                   << inputRec << endl
                   << "   bad receiver clock offset encountered: " << temp
                   << "." << endl;
        appendToWarningMessages(tempStream.str());
      }
    }
    if (inputEpoch.getNumSat() > MAXSATPEREPOCH) {
      tempStream << " On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Warning ! More than " << MAXSATPEREPOCH
                 << " Satellites: " << inputEpoch.getNumSat() << "." << endl;
      inputEpoch.setNumSat(12);  // reset numSat, try to skip to the next epoch
    }

    if (inputEpoch.getNumSat() <= 12) {
      for (i = 0; i < inputEpoch.getNumSat(); i++) {
        temp = inputRec.substr((32 + i * 3), 1);
        saveSatCode.append(temp);
        temp = inputRec.substr((32 + (i * 3) + 1), 2);
        if (getLong(temp, tempL))
          saveSatNum[i] = static_cast<unsigned short>(tempL);
      }
    } else {
      for (i = 0; i < 12; i++) {
        temp = inputRec.substr((32 + i * 3), 1);
        saveSatCode.append(temp);
        temp = inputRec.substr((32 + (i * 3) + 1), 2);
        if (getLong(temp, tempL))
          saveSatNum[i] = static_cast<unsigned short>(tempL);
      }
      temp = inputRec;
      if (!getline(inputStream, inputRec, '\n')) {
        // Error reading more than 12 PRNs following a EPOCH/SAT record
        tempStream << "Error reading more than 12 PRNs after record: " << temp
                   << endl
                   << "in file: " << getPathFilename() << endl;
        appendToErrorMessages(tempStream.str());

        RinexReadingException excep(tempStream.str());
        throw excep;
      }
      incrementNumberLinesRead(1);
      for (int k = 12; k < inputEpoch.getNumSat(); k++) {
        i = k - 12;
        temp = inputRec.substr((32 + i * 3), 1);
        saveSatCode.append(temp);
        temp = inputRec.substr((32 + (i * 3) + 1), 2);
        if (getLong(temp, tempL))
          saveSatNum[k] = static_cast<unsigned short>(tempL);
      }
    }

    if (inputEpoch.getEpochFlag() == 6) {
      tempStream << " On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "Warning ! An Event Flag has been found of type: "
                 << inputEpoch.getEpochFlag() << "." << endl;
    }

    for (i = 0; i < inputEpoch.getNumSat(); i++) {
      if (!getline(inputStream, inputRec, '\n')) {
        // Error reading observation/cycle slip records following an Epoch Flag
        tempStream << "Error reading data records after Epoch Flag, in file:"
                   << endl
                   << getPathFilename() << endl;
        appendToErrorMessages(tempStream.str());

        RinexReadingException excep(tempStream.str());
        throw excep;
      }
      incrementNumberLinesRead(1);
      working = inputRec;
      makeRecordLength80(working);

      if (inputEpoch.getEpochFlag() == 6)
        tempStream << working << endl;

      if (numObsTypes > 5) {
        if (!getline(inputStream, inputRec, '\n')) {
          // Error reading observation/cycle slip records when #obs > 5
          tempStream << "Error reading data records when #obs > 5, in file:"
                     << endl
                     << getPathFilename() << endl;
          appendToErrorMessages(tempStream.str());

          RinexReadingException excep(tempStream.str());
          throw excep;
        }
        incrementNumberLinesRead(1);
        makeRecordLength80(inputRec);
        working.append(inputRec);  // add second line to first line
      }

      for (j = 0; j < numObsTypes; j++) {
        tempDouble = 0.0;
        temp = working.substr((j * 16), 14);
        if (getDouble(temp, tempD))
          tempDouble = tempD;
        tempLLI = working.substr((j * 16) + 14, 1);
        tempSigStrength = working.substr((j * 16) + 15, 1);

        if (fabs(tempDouble) > 0.0001) {
          tempSatObsAtEpoch.obsList[j].obsPresent = true;
          tempSatObsAtEpoch.obsList[j].observation = tempDouble;
          tempSatObsAtEpoch.obsList[j].obsType = obsTypeList[j];

          if (tempLLI == " ")
            tempSatObsAtEpoch.obsList[j].LLI = 0;
          if (isdigit(tempLLI[0])) {
            if (getLong(tempLLI, tempL))
              tempSatObsAtEpoch.obsList[j].LLI =
                  static_cast<unsigned short>(tempL);
          }

          if (tempSigStrength == " ")
            tempSatObsAtEpoch.obsList[j].sigStrength = 0;
          if (isdigit(tempSigStrength[0])) {
            if (getLong(tempSigStrength, tempL))
              tempSatObsAtEpoch.obsList[j].sigStrength =
                  static_cast<unsigned short>(tempL);
          }
        } else {
          tempSatObsAtEpoch.obsList[j].obsPresent = false;
          tempSatObsAtEpoch.obsList[j].observation = 0.0;
          tempSatObsAtEpoch.obsList[j].obsType = obsTypeList[j];
          tempSatObsAtEpoch.obsList[j].LLI = 0;
          tempSatObsAtEpoch.obsList[j].sigStrength = 0;
        }
      }  // for ( j=0; ...

      tempSatObsAtEpoch.satCode = saveSatCode[i];
      tempSatObsAtEpoch.satNum = saveSatNum[i];
      if (!inputEpoch.setSatListElement(tempSatObsAtEpoch, numObsTypes, i)) {
        tempStream
            << "Warning ! On line #" << getNumberLinesRead()
            << "   bad SV name, observation, LLI, or S/N value encountered: "
            << endl
            << working << endl;
        appendToWarningMessages(tempStream.str());
      }

    }  // for ( i=0; ...

    if (inputEpoch.getEpochFlag() == 6)
      appendToWarningMessages(tempStream.str());

  }  // do the above when the Event Flag is 0,1,6

  return (1);  // return 1 when everything was read correctly
}

//---------------------------------------------------------------------------
// writeEpoch()
//    Writes a single "epoch" of data with time-tag record and all satellite
//    records to the output data file.

void RinexObsFile::writeEpoch(ofstream& outputOBS, ObsEpoch& outputEpoch) {
  unsigned short i, j;
  YMDHMS ymdhms;

  // Write the EPOCH/SAT record.
  ymdhms = outputEpoch.getEpochTime().GetYMDHMS();

  outputOBS.setf(ios::fixed);
  outputOBS.setf(ios::showpoint);

  if (ymdhms.year != 9999) {
    outputOBS << " " << setw(2) << setfill('0') << ymdhms.year % 100;
    outputOBS << " " << setw(2) << setfill(' ') << ymdhms.month;
    outputOBS << " " << setw(2) << ymdhms.day;
    outputOBS << " " << setw(2) << ymdhms.hour;
    outputOBS << " " << setw(2) << ymdhms.min;
    outputOBS << setw(11) << setprecision(7) << ymdhms.sec;
  } else
    outputOBS << "                          ";  // 26 blanks if no ymdhms

  if (outputEpoch.getEpochFlag() != 9999) {
    outputOBS << "  " << setw(1) << outputEpoch.getEpochFlag();
  } else {
    outputOBS << "   ";
  }

  if (outputEpoch.getNumSat() != 9999) {
    outputOBS << setw(3) << outputEpoch.getNumSat();
  } else {
    outputOBS << "   ";
  }

  for (i = 0; i < 12; i++)  // do all 12 so that spacers can be inserted
  {
    if (outputEpoch.getSatListElement(i).satNum != 9999) {
      outputOBS << setw(1) << outputEpoch.getSatListElement(i).satCode;
      outputOBS << setw(2) << outputEpoch.getSatListElement(i).satNum;
    } else  // output spacers for PRN-list when rcvr clk offset is present
    {
      if (outputEpoch.getRecClockOffset() != 9999.0 &&
          fabs(outputEpoch.getRecClockOffset()) > 0.0000000001)
        outputOBS << "   ";
    }
  }

  // Note: the recever clock offset is written to 9 decimal places (f12.9)
  // so test for values larger than zero or larger than 0.0000000001 .
  if (outputEpoch.getRecClockOffset() == 9999.0 ||
      fabs(outputEpoch.getRecClockOffset()) < 0.0000000001) {
    outputOBS << endl;
  } else {
    outputOBS.setf(ios::fixed);
    outputOBS.setf(ios::showpoint);
    outputOBS << setw(12) << setprecision(9) << outputEpoch.getRecClockOffset()
              << endl;
  }

  // Write either satellite obs records or header records, if there are any.

  if (outputEpoch.getNumSat() != (unsigned short)9999 &&
      (outputEpoch.getEpochFlag() == 0 || outputEpoch.getEpochFlag() == 1 ||
       outputEpoch.getEpochFlag() == 6)) {
    // Write PRN list extension line here, if necessary (no rcvr clock offset)
    if (outputEpoch.getNumSat() > 12) {
      outputOBS << "                                ";  // 32 blanks
      for (i = 12; i < 24; i++)                         // add up to 12 more svs
      {
        if (outputEpoch.getSatListElement(i).satNum != 9999) {
          outputOBS << setw(1) << outputEpoch.getSatListElement(i).satCode;
          outputOBS << setw(2) << outputEpoch.getSatListElement(i).satNum;
        }
      }
      outputOBS << endl;
    }

    for (i = 0; i < (unsigned short)outputEpoch.getNumSat(); i++) {
      // Write each observation value for each satellite.
      for (j = 0; j < getNumObsTypes(); j++) {
        if (j != 0 && j % 5 == 0)  // Max. 5 obs. per line
        {
          outputOBS << endl;
        }

        if (outputEpoch.getSatListElement(i).obsList[j].obsPresent) {
          outputOBS.setf(ios::fixed);
          outputOBS.setf(ios::showpoint);

          outputOBS << setw(14) << setprecision(3)
                    << outputEpoch.getSatListElement(i).obsList[j].observation;

          if (outputEpoch.getSatListElement(i).obsList[j].LLI ==
                  (unsigned short)9999 ||
              outputEpoch.getSatListElement(i).obsList[j].LLI == 0) {
            if (j != (getNumObsTypes() - 1) && ((j + 1) % 5) != 0)
              outputOBS << " ";
          } else {
            outputOBS << setw(1)
                      << outputEpoch.getSatListElement(i).obsList[j].LLI;
          }

          if (outputEpoch.getSatListElement(i).obsList[j].sigStrength ==
                  (unsigned short)9999 ||
              outputEpoch.getSatListElement(i).obsList[j].sigStrength == 0) {
            if (j != (getNumObsTypes() - 1) && ((j + 1) % 5) != 0)
              outputOBS << " ";
          } else {
            outputOBS
                << setw(1)
                << outputEpoch.getSatListElement(i).obsList[j].sigStrength;
          }
        } else {
          // If this type of obs. is missing from the record for this
          // satellite, fill it in with blanks (when obsPresent = false).
          if (j != (getNumObsTypes() - 1) && ((j + 1) % 5) != 0)
            outputOBS << "                ";
        }
      }  // j-loop over numObsTypes
      outputOBS << endl;
    }  // i-loop over numSat
  } else if (
      outputEpoch.getEpochFlag() >= 2 && outputEpoch.getEpochFlag() <= 5) {
    // output Event Flag records
    outputOBS << outputEpoch.getEpochHeaderRecords();
  }
}

// Selectors
string RinexObsFile::getMarkerName() {
  return markerName;
}
string RinexObsFile::getMarkerNumber() {
  return markerNumber;
}
string RinexObsFile::getObserverName() {
  return observerName;
}
string RinexObsFile::getObserverAgency() {
  return observerAgency;
}
string RinexObsFile::getReceiverNumber() {
  return receiverNumber;
}
string RinexObsFile::getReceiverType() {
  return receiverType;
}
string RinexObsFile::getReceiverFirmwareVersion() {
  return receiverFirmwareVersion;
}
string RinexObsFile::getAntennaNumber() {
  return antennaNumber;
}
string RinexObsFile::getAntennaType() {
  return antennaType;
}

double RinexObsFile::getApproxX() {
  return approxX;
}
double RinexObsFile::getApproxY() {
  return approxY;
}
double RinexObsFile::getApproxZ() {
  return approxZ;
}
double RinexObsFile::getAntennaDeltaH() {
  return antennaDeltaH;
}
double RinexObsFile::getAntennaDeltaE() {
  return antennaDeltaE;
}
double RinexObsFile::getAntennaDeltaN() {
  return antennaDeltaN;
}

unsigned short RinexObsFile::getDefWaveLenFactorL1() {
  return defWaveLenFactorL1;
}
unsigned short RinexObsFile::getDefWaveLenFactorL2() {
  return defWaveLenFactorL2;
}
unsigned short RinexObsFile::getNumWaveLenPRN() {
  return numWaveLenPRN;
}
unsigned short RinexObsFile::getNumWaveLenRecords() {
  return numWaveLenRecords;
}

OneWaveLenRec RinexObsFile::getAllWaveLenRecordsElement(int i) {
  return allWaveLenRecords[i];
}
unsigned short RinexObsFile::getNumObsTypes() {
  return numObsTypes;
}
enum OBSTYPE RinexObsFile::getObsTypeListElement(int i) {
  return obsTypeList[i];
}
float RinexObsFile::getObsInterval() {
  return obsInterval;
}
YMDHMS RinexObsFile::getFirstObs() {
  return firstObs;
}
string RinexObsFile::getFirstObsTimeSystem() {
  return firstObsTimeSystem;
}
YMDHMS RinexObsFile::getLastObs() {
  return lastObs;
}
string RinexObsFile::getLastObsTimeSystem() {
  return lastObsTimeSystem;
}
unsigned short RinexObsFile::getNumberLeapSec() {
  return numberLeapSec;
}
unsigned short RinexObsFile::getRcvrClockApplied() {
  return rcvrClockApplied;
}
unsigned short RinexObsFile::getNumberOfSat() {
  return numberOfSat;
}
ObsCountForPRN RinexObsFile::getSatObsTypeListElement(int i) {
  return satObsTypeList[i];
}
unsigned short RinexObsFile::getNextSat() {
  return nextSat;
}

unsigned int RinexObsFile::getNumberObsEpochs() {
  return (numberObsEpochs);
}

unsigned int RinexObsFile::getObsFilesCount() {
  return (numberObsFiles);
}

//========================== RinexNavFile Class ===============================

// Initialize static data member
unsigned int RinexNavFile::numberNavFiles = 0;  // no objects yet

// Constructors

RinexNavFile::RinexNavFile() : RinexFile() {
  initializeData();
  numberPRNBlocks = 0;
  numberNavFiles++;
}

RinexNavFile::RinexNavFile(string inputFilePath, ios::openmode mode)
    : RinexFile(inputFilePath, mode) {
  initializeData();
  numberPRNBlocks = 0;
  numberNavFiles++;
}

// Destructor
RinexNavFile::~RinexNavFile() {
  --numberNavFiles;
}

// Initializers
bool RinexNavFile::setA0(double input) {
  a0 = input;
  return true;
}
bool RinexNavFile::setA1(double input) {
  a1 = input;
  return true;
}
bool RinexNavFile::setA2(double input) {
  a2 = input;
  return true;
}
bool RinexNavFile::setA3(double input) {
  a3 = input;
  return true;
}

bool RinexNavFile::setB0(double input) {
  b0 = input;
  return true;
}
bool RinexNavFile::setB1(double input) {
  b1 = input;
  return true;
}
bool RinexNavFile::setB2(double input) {
  b2 = input;
  return true;
}
bool RinexNavFile::setB3(double input) {
  b3 = input;
  return true;
}

bool RinexNavFile::setUtcA0(double input) {
  utcA0 = input;
  return true;
}
bool RinexNavFile::setUtcA1(double input) {
  utcA1 = input;
  return true;
}

bool RinexNavFile::setUtcRefTime(long input) {
  utcRefTime = input;
  return true;
}
bool RinexNavFile::setUtcRefWeek(long input) {
  utcRefWeek = input;
  return true;
}
bool RinexNavFile::setLeapSec(unsigned short input) {
  leapSec = input;
  return true;
}

unsigned short RinexNavFile::readHeader() {
  unsigned short i;
  string inputRec;
  string temp;
  string recordReadIn;
  HeaderRecord headerRec;
  ostringstream sstemp;
  bool endOfHeaderFound = false;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    IonAlphaRec,
    IonBetaRec,
    DeltaUTCRec,
    LeapSecRec,
    EndHeaderRec
  };

  if (validFirstLine(recordReadIn)) {
    headerRecs[VersionRec].numberPresent++;
    headerRec.SetHeaderRecord(recordReadIn);
    rinexHeaderImage.appendHeaderRecord(recordReadIn);
  } else {
    tempStream << "Error: First Line is incorrect in File: "
               << getPathFilename() << endl
               << recordReadIn << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  // read from line 2 until the end of header
  while (!endOfHeaderFound) {
    if (!getline(inputStream, inputRec, '\n')) {
      // Error reading a header line of the NAV File
      tempStream << "Error reading a header line of file:" << endl
                 << getPathFilename() << endl;
      appendToErrorMessages(tempStream.str());

      RinexReadingException excep(tempStream.str());
      throw excep;
    }
    incrementNumberLinesRead(1);

    if (blankString(inputRec)) {
      if (formatVersion < 2.0) {
        endOfHeaderFound = true;
        break;  // exit the while loop
      } else {
        tempStream << "Warning! On line #" << getNumberLinesRead() << ":"
                   << endl
                   << inputRec << endl
                   << " Not a Valid Header record, skipping to next line. "
                   << endl;
        appendToWarningMessages(tempStream.str());
        continue;  // go to read the next record
      }
    }

    if (validHeaderRecord(inputRec)) {
      headerRec.SetHeaderRecord(inputRec);
      rinexHeaderImage.appendHeaderRecord(headerRec);
      if (inputRec.find("END OF HEADER") == 60)
        endOfHeaderFound = true;
      continue;  // go to read the next record
    }

  }  // end of while loop over all header records

  // Check for manditory records.

  int numberRequiredErrors = 0;
  for (i = 0; i < MAXNAVHEADERRECTYPES; i++) {
    if (headerRecs[i].numberPresent == 0 && headerRecs[i].required) {
      sstemp << " missing Header record: " << headerRecs[i].recID << endl;
      numberRequiredErrors++;
    }
  }
  if (numberRequiredErrors > 0) {
    tempStream << getPathFilename() << " has the following missing Header "
               << "records." << endl
               << sstemp.str() << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }
  return (0);
}

bool RinexNavFile::validHeaderRecord(string inputRec) {
  unsigned short i;
  string temp;
  string first60;
  HeaderRecord headerRec;
  ostringstream sstemp;
  long tempL;
  double tempD;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    IonAlphaRec,
    IonBetaRec,
    DeltaUTCRec,
    LeapSecRec,
    EndHeaderRec
  };

  if (inputRec.find("COMMENT") == 60) {
    // Comment records are optional, may be more than one.
    // all comments are stored in the headerImage linked-list
    headerRecs[CommentRec].numberPresent++;
    return true;
  } else if (inputRec.find("PGM / RUN BY / DATE") == 60) {
    headerRecs[PgmRunByRec].numberPresent++;

    rinexProgram = inputRec.substr(0, 20);
    createdByAgency = inputRec.substr(20, 20);
    dateFileCreated = inputRec.substr(40, 20);
    return true;
  } else if (inputRec.find("ION ALPHA") == 60) {
    headerRecs[IonAlphaRec].numberPresent++;
    first60 = inputRec.substr(0, 60);
    replace(first60.begin(), first60.end(), 'D', 'E');

    temp = first60.substr(2, 12);
    if (getDouble(temp, tempD))
      a0 = tempD;
    temp = first60.substr(14, 12);
    if (getDouble(temp, tempD))
      a1 = tempD;
    temp = first60.substr(26, 12);
    if (getDouble(temp, tempD))
      a2 = tempD;
    temp = first60.substr(38, 12);
    if (getDouble(temp, tempD))
      a3 = tempD;
    return true;
  } else if (inputRec.find("ION BETA") == 60) {
    headerRecs[IonBetaRec].numberPresent++;
    first60 = inputRec.substr(0, 60);
    replace(first60.begin(), first60.end(), 'D', 'E');

    temp = first60.substr(2, 12);
    if (getDouble(temp, tempD))
      b0 = tempD;
    temp = first60.substr(14, 12);
    if (getDouble(temp, tempD))
      b1 = tempD;
    temp = first60.substr(26, 12);
    if (getDouble(temp, tempD))
      b2 = tempD;
    temp = first60.substr(38, 12);
    if (getDouble(temp, tempD))
      b3 = tempD;
    return true;
  } else if (inputRec.find("DELTA-UTC: A0,A1,T,W") == 60) {
    headerRecs[DeltaUTCRec].numberPresent++;
    first60 = inputRec.substr(0, 60);
    replace(first60.begin(), first60.end(), 'D', 'E');

    temp = first60.substr(3, 19);
    if (getDouble(temp, tempD))
      utcA0 = tempD;
    temp = first60.substr(22, 19);
    if (getDouble(temp, tempD))
      utcA1 = tempD;
    temp = first60.substr(41, 9);
    if (getLong(temp, tempL))
      utcRefTime = tempL;
    temp = first60.substr(50, 9);
    if (getLong(temp, tempL))
      utcRefWeek = tempL;
    return true;
  } else if (inputRec.find("LEAP SECONDS") == 60) {
    headerRecs[LeapSecRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      leapSec = static_cast<unsigned short>(tempL);
    return true;
  } else if (inputRec.find("END OF HEADER") == 60) {
    headerRecs[EndHeaderRec].numberPresent++;
    return true;
  } else {
    tempStream << "Invalid NAV header record: " << endl << inputRec << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }
}

void RinexNavFile::initializeData() {
  int i;

  for (i = 0; i < MAXNAVHEADERRECTYPES; i++)
    headerRecs[i].numberPresent = 0;

  headerRecs[0].recID = "RINEX VERSION / TYPE";
  headerRecs[0].required = true;

  headerRecs[1].recID = "PGM / RUN BY / DATE";
  headerRecs[1].required = true;

  headerRecs[2].recID = "COMMENT";
  headerRecs[2].required = false;

  headerRecs[3].recID = "ION ALPHA";
  headerRecs[3].required = false;

  headerRecs[4].recID = "ION BETA";
  headerRecs[4].required = false;

  headerRecs[5].recID = "DELTA-UTC: A0,A1,T,W";
  headerRecs[5].required = false;

  headerRecs[6].recID = "LEAP SECONDS";
  headerRecs[6].required = false;

  headerRecs[7].recID = "END OF HEADER";
  headerRecs[7].required = true;

  a0 = 0.0;
  a1 = 0.0;
  a2 = 0.0;
  a3 = 0.0;
  b0 = 0.0;
  b1 = 0.0;
  b2 = 0.0;
  b3 = 0.0;
  utcA0 = 0.0;
  utcA1 = 0.0;
  utcRefTime = -9999;
  utcRefWeek = -9999;
  leapSec = 0;
}

unsigned short RinexNavFile::readPRNBlock(PRNBlock& prnBlock) {
  unsigned short tempUS, tyear;
  string inputRec;
  size_t l;
  string temp;
  string working;
  long tempL;
  double tempD;
  string BlankString("                                        ");
  BlankString.append("                                        ");

  // Read epoch "time-tag" record.

  if (!getline(inputStream, inputRec, '\n')) {
    return 0;  // EOF encountered
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(0, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!prnBlock.setSatellitePRN(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid Satellite PRN was found: " << temp << "."
               << endl;
    appendToWarningMessages(tempStream.str());
  }
  temp = inputRec.substr(3, 2);
  if (getLong(temp, tempL))
    tyear = static_cast<unsigned short>(tempL);
  if (tyear >= 80 && tyear <= 99)
    tyear = tyear + 1900;
  if (tyear <= 79)  // between 00 and 79
    tyear = tyear + 2000;
  if (!prnBlock.setTocYear(tyear)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid TOC year was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(6, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!prnBlock.setTocMonth(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid TOC month was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(9, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!prnBlock.setTocDay(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid TOC day was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(12, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!prnBlock.setTocHour(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid TOC hour was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(15, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!prnBlock.setTocMin(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  and invalid TOC minute was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(17, 5);
  if (getDouble(temp, tempD)) {
    if (!prnBlock.setTocSec(tempD)) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "  an invalid TOC second was found: " << temp << "."
                 << endl;
      appendToWarningMessages(tempStream.str());
    }
  }
  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    prnBlock.setClockBias(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    prnBlock.setClockDrift(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    prnBlock.setClockDriftRate(tempD);

  // Read the first line after the PRN number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the first data line of a PRN Block." << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    prnBlock.setIode(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    prnBlock.setCrs(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    prnBlock.setDeltan(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    prnBlock.setMo(tempD);

  // Read the 2nd line after the PRN number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the second data line of a PRN Block: "
               << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    prnBlock.setCuc(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    prnBlock.setEccen(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    prnBlock.setCus(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    prnBlock.setSqrtA(tempD);

  // Read the 3rd line after the PRN number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the third data line of a PRN Block." << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    prnBlock.setToe(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    prnBlock.setCic(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    prnBlock.setBigOmega(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    prnBlock.setCis(tempD);

  // Read the 4th line after the PRN number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the fourth data line of a PRN Block."
               << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    prnBlock.setIo(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    prnBlock.setCrc(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    prnBlock.setLilOmega(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    prnBlock.setBigOmegaDot(tempD);

  // Read the 5th line after the PRN number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the fifth data line of a PRN Block." << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    prnBlock.setIdot(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    prnBlock.setCodesOnL2(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    prnBlock.setToeGPSWeek(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    prnBlock.setPDataFlagL2(tempD);

  // Read the 6th line after the PRN number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning ! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the sixth data line of a PRN Block." << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    prnBlock.setSvAccur(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    prnBlock.setSvHealth(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    prnBlock.setTgd(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    prnBlock.setIodc(tempD);

  // Read the 7th line after the PRN number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the seventh data line of a PRN Block."
               << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    prnBlock.setTransmTime(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    prnBlock.setFitInterval(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    prnBlock.setSpare1(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    prnBlock.setSpare2(tempD);

  return 1;
}

// Selectors
double RinexNavFile::getA0() {
  return a0;
}
double RinexNavFile::getA1() {
  return a1;
}
double RinexNavFile::getA2() {
  return a2;
}
double RinexNavFile::getA3() {
  return a3;
}

double RinexNavFile::getB0() {
  return b0;
}
double RinexNavFile::getB1() {
  return b1;
}
double RinexNavFile::getB2() {
  return b2;
}
double RinexNavFile::getB3() {
  return b3;
}

double RinexNavFile::getUtcA0() {
  return utcA0;
}
double RinexNavFile::getUtcA1() {
  return utcA1;
}
long RinexNavFile::getUtcRefTime() {
  return utcRefTime;
}
long RinexNavFile::getUtcRefWeek() {
  return utcRefWeek;
}
unsigned short RinexNavFile::getLeapSec() {
  return leapSec;
}

void RinexNavFile::writeHeaderImage(ofstream& outputStream) {
  rinexHeaderImage.writeHeaderImage(outputStream);
}

void RinexNavFile::writePRNBlock(
    ofstream& outputStream, PRNBlock& outputPRNBlock) {
  ostringstream stringStream;
  string temp;

  // Write the PRN,"time-tag" record.

  stringStream.setf(ios::fixed, ios::floatfield);
  stringStream.setf(ios::showpoint);

  stringStream << setw(2) << outputPRNBlock.getSatellitePRN();
  stringStream << " " << setw(2) << setfill('0')
               << outputPRNBlock.getTocYear() % 100;
  stringStream << " " << setw(2) << setfill(' ')
               << outputPRNBlock.getTocMonth();
  stringStream << " " << setw(2) << outputPRNBlock.getTocDay();
  stringStream << " " << setw(2) << outputPRNBlock.getTocHour();
  stringStream << " " << setw(2) << outputPRNBlock.getTocMin();
  stringStream << setw(5) << setprecision(1)
               << (double)outputPRNBlock.getTocSec();

  stringStream.setf(ios::scientific, ios::floatfield);
  stringStream.setf(ios::uppercase);
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getClockBias();
  stringStream << setw(19) << setprecision(12)
               << outputPRNBlock.getClockDrift();
  stringStream << setw(19) << setprecision(12)
               << outputPRNBlock.getClockDriftRate() << endl;

  // 1st line
  stringStream << "   " << setw(19) << setprecision(12)
               << outputPRNBlock.getIode();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getCrs();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getDeltan();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getMo()
               << endl;

  // 2nd line
  stringStream << "   " << setw(19) << setprecision(12)
               << outputPRNBlock.getCuc();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getEccen();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getCus();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getSqrtA()
               << endl;

  // 3rd line
  stringStream << "   " << setw(19) << setprecision(12)
               << outputPRNBlock.getToe();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getCic();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getBigOmega();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getCis()
               << endl;

  // 4th line
  stringStream << "   " << setw(19) << setprecision(12)
               << outputPRNBlock.getIo();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getCrc();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getLilOmega();
  stringStream << setw(19) << setprecision(12)
               << outputPRNBlock.getBigOmegaDot() << endl;

  // 5th line
  stringStream << "   " << setw(19) << setprecision(12)
               << outputPRNBlock.getIdot();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getCodesOnL2();
  stringStream << setw(19) << setprecision(12)
               << outputPRNBlock.getToeGPSWeek();
  stringStream << setw(19) << setprecision(12)
               << outputPRNBlock.getPDataFlagL2() << endl;

  // 6th line
  stringStream << "   " << setw(19) << setprecision(12)
               << outputPRNBlock.getSvAccur();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getSvHealth();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getTgd();
  stringStream << setw(19) << setprecision(12) << outputPRNBlock.getIodc()
               << endl;

  // 7th line
  stringStream << "   " << setw(19) << setprecision(12)
               << outputPRNBlock.getTransmTime();
  stringStream << setw(19) << setprecision(12)
               << outputPRNBlock.getFitInterval();

  if (fabs(outputPRNBlock.getSpare1()) > numeric_limits<double>::min())
    stringStream << setw(19) << setprecision(12) << outputPRNBlock.getSpare1();

  if (fabs(outputPRNBlock.getSpare1()) > numeric_limits<double>::min())
    stringStream << setw(19) << setprecision(12) << outputPRNBlock.getSpare2();

  stringStream << endl;
  temp = stringStream.str();
  replace(temp.begin(), temp.end(), 'E', 'D');
  outputStream << temp;
}

void RinexNavFile::incrementNumberPRNBlocks(unsigned int n) {
  numberPRNBlocks = numberPRNBlocks + n;  // n is usually one
}

unsigned int RinexNavFile::getNumberPRNBlocks() {
  return (numberPRNBlocks);
}

unsigned int RinexNavFile::getNavFilesCount() {
  return (numberNavFiles);
}

//======================= GlonassNavFile Class =============================

// Initialize static data member
unsigned int GlonassNavFile::numberFiles = 0;  // no objects yet

// Constructors

GlonassNavFile::GlonassNavFile() : RinexFile() {
  initializeData();
  numberEpochs = 0;
  numberFiles++;
}

GlonassNavFile::GlonassNavFile(string inputFilePath, ios::openmode mode)
    : RinexFile(inputFilePath, mode) {
  initializeData();
  numberEpochs = 0;
  numberFiles++;
}

// Destructor
GlonassNavFile::~GlonassNavFile() {
  --numberFiles;
}

// Initializers
bool GlonassNavFile::setRefTimeYear(unsigned short input) {
  refTimeYear = input;
  return true;
}

bool GlonassNavFile::setRefTimeMonth(unsigned short input) {
  refTimeMonth = input;
  return true;
}

bool GlonassNavFile::setRefTimeDay(unsigned short input) {
  refTimeDay = input;
  return true;
}

bool GlonassNavFile::setTimeScaleCorr(double input) {
  timeScaleCorr = input;
  return true;
}

bool GlonassNavFile::setLeapSec(unsigned short input) {
  leapSec = input;
  return true;
}

unsigned short GlonassNavFile::readHeader() {
  unsigned short i;
  string inputRec;
  string temp;
  string recordReadIn;
  HeaderRecord headerRec;
  ostringstream sstemp;
  bool endOfHeaderFound = false;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    TimeCorrRec,
    LeapSecRec,
    EndHeaderRec
  };

  if (validFirstLine(recordReadIn)) {
    headerRecs[VersionRec].numberPresent++;
    truncateHeaderRec(recordReadIn);
    headerRec.SetHeaderRecord(recordReadIn);
    rinexHeaderImage.appendHeaderRecord(recordReadIn);
  } else {
    tempStream << "Error: First Line is incorrect in File: "
               << getPathFilename() << endl
               << recordReadIn << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  // read from line 2 until the end of header
  while (!endOfHeaderFound) {
    if (!getline(inputStream, inputRec, '\n')) {
      // Error reading a header line of the NAV File
      tempStream << "Error reading a header line of file:" << endl
                 << getPathFilename() << endl;
      appendToErrorMessages(tempStream.str());

      RinexReadingException excep(tempStream.str());
      throw excep;
    }
    truncateHeaderRec(inputRec);
    incrementNumberLinesRead(1);

    if (blankString(inputRec)) {
      if (formatVersion < 2.0) {
        endOfHeaderFound = true;
        break;  // exit the while loop
      } else {
        tempStream << "Warning! On line #" << getNumberLinesRead() << ":"
                   << endl
                   << inputRec << endl
                   << " Not a Valid Header record, skipping to next line. "
                   << endl;
        appendToWarningMessages(tempStream.str());
        continue;  // go to read the next record
      }
    }

    if (validHeaderRecord(inputRec)) {
      headerRec.SetHeaderRecord(inputRec);
      rinexHeaderImage.appendHeaderRecord(headerRec);
      if (inputRec.find("END OF HEADER") == 60)
        endOfHeaderFound = true;
      continue;  // go to read the next record
    }

  }  // end of while loop over all header records

  // Check for manditory records.

  int numberRequiredErrors = 0;
  for (i = 0; i < MAXGLONAVHEADERRECTYPES; i++) {
    if (headerRecs[i].numberPresent == 0 && headerRecs[i].required) {
      sstemp << " missing Header record: " << headerRecs[i].recID << endl;
      numberRequiredErrors++;
    }
  }
  if (numberRequiredErrors > 0) {
    tempStream << getPathFilename() << " has the following missing Header "
               << "records." << endl
               << sstemp.str() << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }
  return (0);
}

bool GlonassNavFile::validHeaderRecord(string inputRec) {
  unsigned short i;
  string temp;
  string first60;
  HeaderRecord headerRec;
  ostringstream sstemp;
  long tempL;
  double tempD;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    TimeCorrRec,
    LeapSecRec,
    EndHeaderRec
  };

  if (inputRec.find("COMMENT") == 60) {
    // Comment records are optional, may be more than one.
    // all comments are stored in the headerImage linked-list
    headerRecs[CommentRec].numberPresent++;
    return true;
  } else if (inputRec.find("PGM / RUN BY / DATE") == 60) {
    headerRecs[PgmRunByRec].numberPresent++;

    rinexProgram = inputRec.substr(0, 20);
    createdByAgency = inputRec.substr(20, 20);
    dateFileCreated = inputRec.substr(40, 20);
    return true;
  } else if (inputRec.find("CORR TO SYSTEM TIME") == 60) {
    headerRecs[TimeCorrRec].numberPresent++;
    first60 = inputRec.substr(0, 60);
    replace(first60.begin(), first60.end(), 'D', 'E');

    temp = first60.substr(0, 6);
    if (getLong(temp, tempL))
      refTimeYear = static_cast<unsigned short>(tempL);
    temp = first60.substr(6, 6);
    if (getLong(temp, tempL))
      refTimeMonth = static_cast<unsigned short>(tempL);
    temp = first60.substr(12, 6);
    if (getLong(temp, tempL))
      refTimeDay = static_cast<unsigned short>(tempL);

    temp = first60.substr(21, 19);
    if (getDouble(temp, tempD))
      timeScaleCorr = tempD;

    return true;
  } else if (inputRec.find("LEAP SECONDS") == 60) {
    headerRecs[LeapSecRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      leapSec = static_cast<unsigned short>(tempL);
    return true;
  } else if (inputRec.find("END OF HEADER") == 60) {
    headerRecs[EndHeaderRec].numberPresent++;
    return true;
  } else {
    tempStream << "Invalid NAV header record: " << endl << inputRec << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }
}

void GlonassNavFile::initializeData() {
  int i;

  for (i = 0; i < MAXGLONAVHEADERRECTYPES; i++)
    headerRecs[i].numberPresent = 0;

  headerRecs[0].recID = "RINEX VERSION / TYPE";
  headerRecs[0].required = true;

  headerRecs[1].recID = "PGM / RUN BY / DATE";
  headerRecs[1].required = true;

  headerRecs[2].recID = "COMMENT";
  headerRecs[2].required = false;

  headerRecs[3].recID = "CORR TO SYSTEM TIME";
  headerRecs[3].required = false;

  headerRecs[4].recID = "LEAP SECONDS";
  headerRecs[4].required = false;

  headerRecs[5].recID = "END OF HEADER";
  headerRecs[5].required = true;

  refTimeYear = 9999;
  refTimeMonth = 1;
  refTimeDay = 1;
  timeScaleCorr = 0.0;
  leapSec = 0;
}

unsigned short GlonassNavFile::readEphemEpoch(GlonassEphemEpoch& navEpoch) {
  unsigned short tempUS, tyear;
  string inputRec;
  size_t l;
  string temp;
  string working;
  long tempL;
  double tempD;
  string BlankString("                                        ");
  BlankString.append("                                        ");

  // Read epoch "time-tag" record.

  if (!getline(inputStream, inputRec, '\n')) {
    return 0;  // EOF encountered
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(0, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setSatelliteAlmanacNumber(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid Satellite Almanac Number was found: " << temp
               << "." << endl;
    appendToWarningMessages(tempStream.str());
  }
  temp = inputRec.substr(3, 2);
  if (getLong(temp, tempL))
    tyear = static_cast<unsigned short>(tempL);
  if (tyear >= 80 && tyear <= 99)
    tyear = tyear + 1900;
  if (tyear <= 79)  // between 00 and 79
    tyear = tyear + 2000;
  if (!navEpoch.setEpochYear(tyear)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid epoch year was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(6, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setEpochMonth(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid epoch month was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(9, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setEpochDay(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid epoch day was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(12, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setEpochHour(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid epoch hour was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(15, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setEpochMin(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  and invalid epoch minute was found: " << temp << "."
               << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(17, 5);
  if (getDouble(temp, tempD)) {
    if (!navEpoch.setEpochSec(tempD)) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "  an invalid epoch second was found: " << temp << "."
                 << endl;
      appendToWarningMessages(tempStream.str());
    }
  }
  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    navEpoch.setSvClockBias(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    navEpoch.setSvRelFreqBias(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    navEpoch.setMessageFrameTime(tempD);

  // Read the first line after the Satellite Almanac number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the first data line of a Nav Epoch." << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    navEpoch.setPosX(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    navEpoch.setVelX(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    navEpoch.setAccX(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    navEpoch.setSvHealth(tempD);

  // Read the 2nd line after the Satellite Almanac number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the second data line of a Nav Epoch: "
               << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    navEpoch.setPosY(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    navEpoch.setVelY(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    navEpoch.setAccY(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    navEpoch.setFreqNumber(tempD);

  // Read the 3rd line after the Satellite Almanac number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the third data line of a Nav Epoch." << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    navEpoch.setPosZ(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    navEpoch.setVelZ(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    navEpoch.setAccZ(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    navEpoch.setAgeOfOperation(tempD);

  return 1;
}

// Selectors
unsigned short GlonassNavFile::getRefTimeYear() {
  return refTimeYear;
}
unsigned short GlonassNavFile::getRefTimeMonth() {
  return refTimeMonth;
}
unsigned short GlonassNavFile::getRefTimeDay() {
  return refTimeDay;
}
double GlonassNavFile::getTimeScaleCorr() {
  return timeScaleCorr;
}
unsigned short GlonassNavFile::getLeapSec() {
  return leapSec;
}

void GlonassNavFile::writeHeaderImage(ofstream& outputStream) {
  rinexHeaderImage.writeHeaderImage(outputStream);
}

void GlonassNavFile::writeEphemEpoch(
    ofstream& outputStream, GlonassEphemEpoch& navEpoch) {
  ostringstream stringStream;
  string temp;

  // Write the PRN,"time-tag" record.

  stringStream.setf(ios::fixed, ios::floatfield);
  stringStream.setf(ios::showpoint);

  stringStream << setw(2) << navEpoch.getSatelliteAlmanacNumber();
  stringStream << " " << setw(2) << setfill('0')
               << navEpoch.getEpochYear() % 100;
  stringStream << " " << setw(2) << setfill(' ') << navEpoch.getEpochMonth();
  stringStream << " " << setw(2) << navEpoch.getEpochDay();
  stringStream << " " << setw(2) << navEpoch.getEpochHour();
  stringStream << " " << setw(2) << navEpoch.getEpochMin();
  stringStream << setw(5) << setprecision(1) << navEpoch.getEpochSec();

  stringStream.setf(ios::scientific, ios::floatfield);
  stringStream.setf(ios::uppercase);
  stringStream << setw(19) << setprecision(12) << navEpoch.getSvClockBias();
  stringStream << setw(19) << setprecision(12) << navEpoch.getSvRelFreqBias();
  stringStream << setw(19) << setprecision(12) << navEpoch.getMessageFrameTime()
               << endl;

  // 1st line
  stringStream << "   " << setw(19) << setprecision(12) << navEpoch.getPosX();
  stringStream << setw(19) << setprecision(12) << navEpoch.getVelX();
  stringStream << setw(19) << setprecision(12) << navEpoch.getAccX();
  stringStream << setw(19) << setprecision(12) << navEpoch.getSvHealth()
               << endl;

  // 2nd line
  stringStream << "   " << setw(19) << setprecision(12) << navEpoch.getPosY();
  stringStream << setw(19) << setprecision(12) << navEpoch.getVelY();
  stringStream << setw(19) << setprecision(12) << navEpoch.getAccY();
  stringStream << setw(19) << setprecision(12) << navEpoch.getFreqNumber()
               << endl;

  // 3rd line
  stringStream << "   " << setw(19) << setprecision(12) << navEpoch.getPosZ();
  stringStream << setw(19) << setprecision(12) << navEpoch.getVelZ();
  stringStream << setw(19) << setprecision(12) << navEpoch.getAccZ();
  stringStream << setw(19) << setprecision(12) << navEpoch.getAgeOfOperation()
               << endl;

  // output the entire string stream
  temp = stringStream.str();
  replace(temp.begin(), temp.end(), 'E', 'D');
  outputStream << temp;
}

void GlonassNavFile::incrementNumberEphemEpochs(unsigned int n) {
  numberEpochs = numberEpochs + n;  // n is usually one
}

unsigned int GlonassNavFile::getNumberEpochs() {
  return (numberEpochs);
}

unsigned int GlonassNavFile::getFilesCount() {
  return (numberFiles);
}

//===================== Geostationary NavFile Class ===========================

// Initialize static data member
unsigned int GeostationaryNavFile::numberFiles = 0;  // no objects yet

// Constructors

GeostationaryNavFile::GeostationaryNavFile() : RinexFile() {
  initializeData();
  numberEpochs = 0;
  numberFiles++;
}

GeostationaryNavFile::GeostationaryNavFile(
    string inputFilePath, ios::openmode mode)
    : RinexFile(inputFilePath, mode) {
  initializeData();
  numberEpochs = 0;
  numberFiles++;
}

// Destructor
GeostationaryNavFile::~GeostationaryNavFile() {
  --numberFiles;
}

// Initializers
bool GeostationaryNavFile::setRefTimeYear(unsigned short input) {
  refTimeYear = input;
  return true;
}

bool GeostationaryNavFile::setRefTimeMonth(unsigned short input) {
  refTimeMonth = input;
  return true;
}

bool GeostationaryNavFile::setRefTimeDay(unsigned short input) {
  refTimeDay = input;
  return true;
}

bool GeostationaryNavFile::setCorrToUTC(double input) {
  corrToUTC = input;
  return true;
}

bool GeostationaryNavFile::setLeapSec(unsigned short input) {
  leapSec = input;
  return true;
}

unsigned short GeostationaryNavFile::readHeader() {
  unsigned short i;
  string inputRec;
  string temp;
  string recordReadIn;
  HeaderRecord headerRec;
  ostringstream sstemp;
  bool endOfHeaderFound = false;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    TimeCorrRec,
    LeapSecRec,
    EndHeaderRec
  };

  if (validFirstLine(recordReadIn)) {
    headerRecs[VersionRec].numberPresent++;
    truncateHeaderRec(recordReadIn);
    headerRec.SetHeaderRecord(recordReadIn);
    rinexHeaderImage.appendHeaderRecord(recordReadIn);
  } else {
    tempStream << "Error: First Line is incorrect in File: "
               << getPathFilename() << endl
               << recordReadIn << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  // read from line 2 until the end of header
  while (!endOfHeaderFound) {
    if (!getline(inputStream, inputRec, '\n')) {
      // Error reading a header line of the NAV File
      tempStream << "Error reading a header line of file:" << endl
                 << getPathFilename() << endl;
      appendToErrorMessages(tempStream.str());

      RinexReadingException excep(tempStream.str());
      throw excep;
    }
    truncateHeaderRec(inputRec);
    incrementNumberLinesRead(1);

    if (blankString(inputRec)) {
      if (formatVersion < 2.0) {
        endOfHeaderFound = true;
        break;  // exit the while loop
      } else {
        tempStream << "Warning! On line #" << getNumberLinesRead() << ":"
                   << endl
                   << inputRec << endl
                   << " Not a Valid Header record, skipping to next line. "
                   << endl;
        appendToWarningMessages(tempStream.str());
        continue;  // go to read the next record
      }
    }

    if (validHeaderRecord(inputRec)) {
      headerRec.SetHeaderRecord(inputRec);
      rinexHeaderImage.appendHeaderRecord(headerRec);
      if (inputRec.find("END OF HEADER") == 60)
        endOfHeaderFound = true;
      continue;  // go to read the next record
    }

  }  // end of while loop over all header records

  // Check for manditory records.

  int numberRequiredErrors = 0;
  for (i = 0; i < MAXGEONAVHEADERRECTYPES; i++) {
    if (headerRecs[i].numberPresent == 0 && headerRecs[i].required) {
      sstemp << " missing Header record: " << headerRecs[i].recID << endl;
      numberRequiredErrors++;
    }
  }
  if (numberRequiredErrors > 0) {
    tempStream << getPathFilename() << " has the following missing Header "
               << "records." << endl
               << sstemp.str() << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }
  return (0);
}

bool GeostationaryNavFile::validHeaderRecord(string inputRec) {
  unsigned short i;
  string temp;
  string first60;
  HeaderRecord headerRec;
  ostringstream sstemp;
  long tempL;
  double tempD;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    TimeCorrRec,
    LeapSecRec,
    EndHeaderRec
  };

  if (inputRec.find("COMMENT") == 60) {
    // Comment records are optional, may be more than one.
    // all comments are stored in the headerImage linked-list
    headerRecs[CommentRec].numberPresent++;
    return true;
  } else if (inputRec.find("PGM / RUN BY / DATE") == 60) {
    headerRecs[PgmRunByRec].numberPresent++;

    rinexProgram = inputRec.substr(0, 20);
    createdByAgency = inputRec.substr(20, 20);
    dateFileCreated = inputRec.substr(40, 20);
    return true;
  } else if (inputRec.find("CORR TO SYSTEM TIME") == 60) {
    headerRecs[TimeCorrRec].numberPresent++;
    first60 = inputRec.substr(0, 60);
    replace(first60.begin(), first60.end(), 'D', 'E');

    temp = first60.substr(0, 6);
    if (getLong(temp, tempL))
      refTimeYear = static_cast<unsigned short>(tempL);
    temp = first60.substr(6, 6);
    if (getLong(temp, tempL))
      refTimeMonth = static_cast<unsigned short>(tempL);
    temp = first60.substr(12, 6);
    if (getLong(temp, tempL))
      refTimeDay = static_cast<unsigned short>(tempL);

    temp = first60.substr(21, 19);
    if (getDouble(temp, tempD))
      corrToUTC = tempD;

    return true;
  } else if (inputRec.find("LEAP SECONDS") == 60) {
    headerRecs[LeapSecRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      leapSec = static_cast<unsigned short>(tempL);
    return true;
  } else if (inputRec.find("END OF HEADER") == 60) {
    headerRecs[EndHeaderRec].numberPresent++;
    return true;
  } else {
    tempStream << "Invalid NAV header record: " << endl << inputRec << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }
}

void GeostationaryNavFile::initializeData() {
  int i;

  for (i = 0; i < MAXGEONAVHEADERRECTYPES; i++)
    headerRecs[i].numberPresent = 0;

  headerRecs[0].recID = "RINEX VERSION / TYPE";
  headerRecs[0].required = true;

  headerRecs[1].recID = "PGM / RUN BY / DATE";
  headerRecs[1].required = true;

  headerRecs[2].recID = "COMMENT";
  headerRecs[2].required = false;

  headerRecs[3].recID = "CORR TO SYSTEM TIME";
  headerRecs[3].required = false;

  headerRecs[4].recID = "LEAP SECONDS";
  headerRecs[4].required = false;

  headerRecs[5].recID = "END OF HEADER";
  headerRecs[5].required = true;

  refTimeYear = 9999;
  refTimeMonth = 1;
  refTimeDay = 1;
  corrToUTC = 0.0;
  leapSec = 0;
}

unsigned short GeostationaryNavFile::readEphemEpoch(
    GeostationaryEphemEpoch& navEpoch) {
  unsigned short tempUS, tyear;
  string inputRec;
  size_t l;
  string temp;
  string working;
  long tempL;
  double tempD;
  string BlankString("                                        ");
  BlankString.append("                                        ");

  // Read epoch "time-tag" record.

  if (!getline(inputStream, inputRec, '\n')) {
    return 0;  // EOF encountered
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(0, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setSatelliteNumber(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid Satellite Number was found: " << temp << "."
               << endl;
    appendToWarningMessages(tempStream.str());
  }
  temp = inputRec.substr(3, 2);
  if (getLong(temp, tempL))
    tyear = static_cast<unsigned short>(tempL);
  if (tyear >= 80 && tyear <= 99)
    tyear = tyear + 1900;
  if (tyear <= 79)  // between 00 and 79
    tyear = tyear + 2000;
  if (!navEpoch.setEpochYear(tyear)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid epoch year was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(6, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setEpochMonth(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid epoch month was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(9, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setEpochDay(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid epoch day was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(12, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setEpochHour(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  an invalid epoch hour was found: " << temp << "." << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(15, 2);
  if (getLong(temp, tempL))
    tempUS = static_cast<unsigned short>(tempL);
  if (!navEpoch.setEpochMin(tempUS)) {
    tempStream << "On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  and invalid epoch minute was found: " << temp << "."
               << endl;
    appendToWarningMessages(tempStream.str());
  }

  temp = inputRec.substr(17, 5);
  if (getDouble(temp, tempD)) {
    if (!navEpoch.setEpochSec(tempD)) {
      tempStream << "On line #" << getNumberLinesRead() << ":" << endl
                 << inputRec << endl
                 << "  an invalid epoch second was found: " << temp << "."
                 << endl;
      appendToWarningMessages(tempStream.str());
    }
  }
  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    navEpoch.setSvClockBias(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    navEpoch.setSvRelFreqBias(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    navEpoch.setMessageFrameTime(tempD);

  // Read the first line after the Satellite number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the first data line of a Nav Epoch." << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    navEpoch.setPosX(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    navEpoch.setVelX(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    navEpoch.setAccX(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    navEpoch.setSvHealth(tempD);

  // Read the 2nd line after the Satellite Almanac number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the second data line of a Nav Epoch: "
               << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    navEpoch.setPosY(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    navEpoch.setVelY(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    navEpoch.setAccY(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    navEpoch.setAccurCode(tempD);

  // Read the 3rd line after the Satellite Almanac number and time tag

  if (!getline(inputStream, inputRec, '\n')) {
    tempStream << "Warning! On line #" << getNumberLinesRead() << ":" << endl
               << inputRec << endl
               << "  error reading the third data line of a Nav Epoch." << endl;
    appendToWarningMessages(tempStream.str());
    return 0;
  }
  incrementNumberLinesRead(1);
  replace(inputRec.begin(), inputRec.end(), 'D', 'E');
  makeRecordLength80(inputRec);

  temp = inputRec.substr(3, 19);
  if (getDouble(temp, tempD))
    navEpoch.setPosZ(tempD);

  temp = inputRec.substr(22, 19);
  if (getDouble(temp, tempD))
    navEpoch.setVelZ(tempD);

  temp = inputRec.substr(41, 19);
  if (getDouble(temp, tempD))
    navEpoch.setAccZ(tempD);

  temp = inputRec.substr(60, 19);
  if (getDouble(temp, tempD))
    navEpoch.setSpare(tempD);

  return 1;
}

// Selectors
unsigned short GeostationaryNavFile::getRefTimeYear() {
  return refTimeYear;
}
unsigned short GeostationaryNavFile::getRefTimeMonth() {
  return refTimeMonth;
}
unsigned short GeostationaryNavFile::getRefTimeDay() {
  return refTimeDay;
}
double GeostationaryNavFile::getCorrToUTC() {
  return corrToUTC;
}
unsigned short GeostationaryNavFile::getLeapSec() {
  return leapSec;
}

void GeostationaryNavFile::writeHeaderImage(ofstream& outputStream) {
  rinexHeaderImage.writeHeaderImage(outputStream);
}

void GeostationaryNavFile::writeEphemEpoch(
    ofstream& outputStream, GeostationaryEphemEpoch& navEpoch) {
  ostringstream stringStream;
  string temp;

  // Write the PRN,"time-tag" record.

  stringStream.setf(ios::fixed, ios::floatfield);
  stringStream.setf(ios::showpoint);

  stringStream << setw(2) << navEpoch.getSatelliteNumber();
  stringStream << " " << setw(2) << setfill('0')
               << navEpoch.getEpochYear() % 100;
  stringStream << " " << setw(2) << setfill(' ') << navEpoch.getEpochMonth();
  stringStream << " " << setw(2) << navEpoch.getEpochDay();
  stringStream << " " << setw(2) << navEpoch.getEpochHour();
  stringStream << " " << setw(2) << navEpoch.getEpochMin();
  stringStream << setw(5) << setprecision(1) << navEpoch.getEpochSec();

  stringStream.setf(ios::scientific, ios::floatfield);
  stringStream.setf(ios::uppercase);
  stringStream << setw(19) << setprecision(12) << navEpoch.getSvClockBias();
  stringStream << setw(19) << setprecision(12) << navEpoch.getSvRelFreqBias();
  stringStream << setw(19) << setprecision(12) << navEpoch.getMessageFrameTime()
               << endl;

  // 1st line
  stringStream << "   " << setw(19) << setprecision(12) << navEpoch.getPosX();
  stringStream << setw(19) << setprecision(12) << navEpoch.getVelX();
  stringStream << setw(19) << setprecision(12) << navEpoch.getAccX();
  stringStream << setw(19) << setprecision(12) << navEpoch.getSvHealth()
               << endl;

  // 2nd line
  stringStream << "   " << setw(19) << setprecision(12) << navEpoch.getPosY();
  stringStream << setw(19) << setprecision(12) << navEpoch.getVelY();
  stringStream << setw(19) << setprecision(12) << navEpoch.getAccY();
  stringStream << setw(19) << setprecision(12) << navEpoch.getAccurCode()
               << endl;

  // 3rd line
  stringStream << "   " << setw(19) << setprecision(12) << navEpoch.getPosZ();
  stringStream << setw(19) << setprecision(12) << navEpoch.getVelZ();
  stringStream << setw(19) << setprecision(12) << navEpoch.getAccZ();
  stringStream << setw(19) << setprecision(12) << navEpoch.getSpare() << endl;

  // output the entire string stream
  temp = stringStream.str();
  replace(temp.begin(), temp.end(), 'E', 'D');
  outputStream << temp;
}

void GeostationaryNavFile::incrementNumberEphemEpochs(unsigned int n) {
  numberEpochs = numberEpochs + n;  // n is usually one
}

unsigned int GeostationaryNavFile::getNumberEpochs() {
  return (numberEpochs);
}

unsigned int GeostationaryNavFile::getFilesCount() {
  return (numberFiles);  //***Geostationary
}

//========================== RinexMetFile Class ===============================

// Initialize static data member
unsigned int RinexMetFile::numberMetFiles = 0;  // no objects yet

// Constructors
RinexMetFile::RinexMetFile() : RinexFile() {
  initializeData();
  numberMetEpochs = 0;
  numberMetFiles++;
}

RinexMetFile::RinexMetFile(string inputFilePath, ios::openmode mode)
    : RinexFile(inputFilePath, mode) {
  initializeData();
  numberMetEpochs = 0;
  numberMetFiles++;
}

// Destructor
RinexMetFile::~RinexMetFile() {
  --numberMetFiles;
}

// Initializers
bool RinexMetFile::setMarkerName(string input) {
  markerName = input;
  return true;
}
bool RinexMetFile::setMarkerNumber(string input) {
  markerNumber = input;
  return true;
}
bool RinexMetFile::setNumMetTypes(unsigned short input) {
  numMetTypes = input;
  return true;
}
bool RinexMetFile::setObsTypeListElement(enum METTYPE input, int i) {
  obsTypeList[i] = input;
  return true;
}
bool RinexMetFile::setSensorModAccurElement(SensorInfo input, int i) {
  sensorModAccur[i] = input;
  return true;
}
bool RinexMetFile::setSensorXYZhElement(SensorPosition input, int i) {
  sensorXYZh[i] = input;
  return true;
}

unsigned short RinexMetFile::readHeader() {
  unsigned short i;
  string inputRec;
  string temp;
  string recordReadIn;
  HeaderRecord headerRec;
  ostringstream sstemp;
  bool endOfHeaderFound = false;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    MarkerNameRec,
    MarkerNumRec,
    ObsNumTypesRec,
    ModTypeAccRec,
    XyzHgtRec,
    EndHeaderRec
  };

  if (validFirstLine(recordReadIn)) {
    headerRecs[VersionRec].numberPresent++;
    headerRec.SetHeaderRecord(recordReadIn);
    rinexHeaderImage.appendHeaderRecord(recordReadIn);
  } else {
    tempStream << "Error: First Line is incorrect in File: "
               << getPathFilename() << endl
               << recordReadIn << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  // read from line 2 until the end of header
  while (!endOfHeaderFound) {
    if (!getline(inputStream, inputRec, '\n')) {
      // Error reading a header line of the MET File
      tempStream << "Error, cannot read a header line from file:" << endl
                 << getPathFilename() << endl;
      appendToErrorMessages(tempStream.str());

      RinexReadingException excep(tempStream.str());
      throw excep;
    }
    incrementNumberLinesRead(1);

    if (blankString(inputRec)) {
      if (formatVersion < 2.0) {
        endOfHeaderFound = true;
        break;  // exit the while loop
      } else {
        tempStream << "Warning! On line: " << getNumberLinesRead() << ":"
                   << endl
                   << inputRec << endl
                   << " Not a Valid Header record, skipping to next line. "
                   << endl;
        appendToWarningMessages(tempStream.str());
        continue;  // go to read the next record
      }
    }

    if (validHeaderRecord(inputRec)) {
      headerRec.SetHeaderRecord(inputRec);
      rinexHeaderImage.appendHeaderRecord(headerRec);
      if (inputRec.find("END OF HEADER") == 60)
        endOfHeaderFound = true;
      continue;  // go to read the next record
    }

  }  // end of while loop over all header records

  // Check for manditory records.

  int numberRequiredErrors = 0;
  for (i = 0; i < MAXMETHEADERRECTYPES; i++) {
    if (headerRecs[i].numberPresent == 0 && headerRecs[i].required) {
      sstemp << " missing Header record: " << headerRecs[i].recID << endl;
      numberRequiredErrors++;
    }
  }
  if (numberRequiredErrors > 0) {
    tempStream << getPathFilename() << " has the following missing Header "
               << "records." << endl
               << sstemp.str() << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  return (0);
}

bool RinexMetFile::validHeaderRecord(string inputRec) {
  unsigned short i;
  string temp;
  HeaderRecord headerRec;
  int indexModTypeAccRecs = -1;
  int indexXyzHgtRecs = -1;
  ostringstream sstemp;
  long tempL;
  double tempD;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    MarkerNameRec,
    MarkerNumRec,
    ObsNumTypesRec,
    ModTypeAccRec,
    XyzHgtRec,
    EndHeaderRec
  };

  if (inputRec.find("COMMENT") == 60) {
    // Comment records are optional, may be more than one.
    // all comments are stored in the headerImage linked-list
    headerRecs[CommentRec].numberPresent++;
    return true;
  } else if (inputRec.find("PGM / RUN BY / DATE") == 60) {
    headerRecs[PgmRunByRec].numberPresent++;

    rinexProgram = inputRec.substr(0, 20);
    createdByAgency = inputRec.substr(20, 20);
    dateFileCreated = inputRec.substr(40, 20);
    return true;
  } else if (inputRec.find("MARKER NAME") == 60) {
    headerRecs[MarkerNameRec].numberPresent++;
    markerName = inputRec.substr(0, 60);
    return true;
  } else if (inputRec.find("MARKER NUMBER") == 60) {
    headerRecs[MarkerNumRec].numberPresent++;
    markerNumber = inputRec.substr(0, 20);
    return true;
  } else if (inputRec.find("# / TYPES OF OBSERV") == 60) {
    string obsType;

    headerRecs[ObsNumTypesRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      numMetTypes = static_cast<unsigned short>(tempL);

    temp = inputRec.substr(6, 54);
    istringstream nextObs(temp);

    for (i = 0; i < MAXMETTYPES; i++) {
      obsTypeList[i] = NOMET;
    }

    i = 0;
    while (nextObs >> obsType) {
      if (obsType.find("PR") != string::npos)
        obsTypeList[i] = PR;
      if (obsType.find("TD") != string::npos)
        obsTypeList[i] = TD;
      if (obsType.find("HR") != string::npos)
        obsTypeList[i] = HR;
      if (obsType.find("ZW") != string::npos)
        obsTypeList[i] = ZW;
      if (obsType.find("ZD") != string::npos)
        obsTypeList[i] = ZD;
      if (obsType.find("ZT") != string::npos)
        obsTypeList[i] = ZT;
      i++;
    }
    return true;

  } else if (inputRec.find("SENSOR MOD/TYPE/ACC") == 60) {
    indexModTypeAccRecs++;
    headerRecs[ModTypeAccRec].numberPresent++;

    sensorModAccur[indexModTypeAccRecs].model = inputRec.substr(0, 20);
    sensorModAccur[indexModTypeAccRecs].type = inputRec.substr(20, 20);
    temp = inputRec.substr(46, 7);
    if (getDouble(temp, tempD))
      sensorModAccur[indexModTypeAccRecs].accuracy = tempD;
    sensorModAccur[indexModTypeAccRecs].metObsType = inputRec.substr(57, 2);
    return true;
  } else if (inputRec.find("SENSOR POS XYZ/H") == 60) {
    indexXyzHgtRecs++;
    headerRecs[XyzHgtRec].numberPresent++;

    temp = inputRec.substr(0, 14);
    if (getDouble(temp, tempD))
      sensorXYZh[indexXyzHgtRecs].x = tempD;
    temp = inputRec.substr(14, 14);
    if (getDouble(temp, tempD))
      sensorXYZh[indexXyzHgtRecs].y = tempD;
    temp = inputRec.substr(28, 14);
    if (getDouble(temp, tempD))
      sensorXYZh[indexXyzHgtRecs].z = tempD;
    temp = inputRec.substr(42, 14);
    if (getDouble(temp, tempD))
      sensorXYZh[indexXyzHgtRecs].h = tempD;
    sensorXYZh[indexXyzHgtRecs].metObsType = inputRec.substr(57, 2);
    return true;
  } else if (inputRec.find("END OF HEADER") == 60) {
    headerRecs[EndHeaderRec].numberPresent++;
    return true;
  } else {
    tempStream << "Invalid MET header record: " << endl << inputRec << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }
}

void RinexMetFile::initializeData() {
  int i;

  markerName = "";
  markerNumber = "";
  numMetTypes = 0;

  for (i = 0; i < MAXMETTYPES; i++) {
    obsTypeList[i] = NOMET;
    sensorModAccur[i].model = "";
    sensorModAccur[i].type = "";
    sensorModAccur[i].accuracy = 0.0;
    sensorModAccur[i].metObsType = "";
    sensorXYZh[i].x = 0.0;
    sensorXYZh[i].y = 0.0;
    sensorXYZh[i].z = 0.0;
    sensorXYZh[i].h = 0.0;
    sensorXYZh[i].metObsType = "";
  }

  for (i = 0; i < MAXMETHEADERRECTYPES; i++)
    headerRecs[i].numberPresent = 0;

  headerRecs[0].recID = "RINEX VERSION / TYPE";
  headerRecs[0].required = true;

  headerRecs[1].recID = "PGM / RUN BY / DATE";
  headerRecs[1].required = true;

  headerRecs[2].recID = "COMMENT";
  headerRecs[2].required = false;

  headerRecs[3].recID = "MARKER NAME";
  headerRecs[3].required = true;

  headerRecs[4].recID = "MARKER NUMBER";
  headerRecs[4].required = false;

  headerRecs[5].recID = "# / TYPES OF OBSERV";
  headerRecs[5].required = true;

  headerRecs[6].recID = "SENSOR MOD/TYPE/ACC";
  headerRecs[6].required = false;

  headerRecs[7].recID = "SENSOR POS XYZ/H";
  headerRecs[7].required = false;

  headerRecs[8].recID = "END OF HEADER";
  headerRecs[8].required = true;
}

void RinexMetFile::initEpochInfo(MetEpoch& epoch) {
  int i;
  DateTime tempDateTime(9999, 1, 1, 0, 0, 0.0);
  MetSet tempMetSet;

  tempMetSet.obsPresent = false;
  tempMetSet.observation = 0.0;  // blank = 0.0
  tempMetSet.metType = NOMET;

  epoch.setEpochTime(tempDateTime);
  for (i = 0; i < MAXMETTYPES; i++)
    epoch.setMetListElement(tempMetSet, i);
}

unsigned short RinexMetFile::readEpoch(MetEpoch& epoch) {
  string warningString;
  string inputRec;
  size_t l;
  YMDHMS ymdhms;
  int i;
  string temp;
  string working;
  long tempL;
  double tempD;
  string BlankString("                                        ");
  BlankString.append("                                        ");

  // Read epoch "time-tag" record.

  if (!getline(inputStream, inputRec, '\n')) {
    return 0;  // EOF encountered
  }
  initEpochInfo(epoch);
  makeRecordLength80(inputRec);

  temp = inputRec.substr(1, 2);
  if (getLong(temp, tempL))
    ymdhms.year = tempL;
  if (ymdhms.year >= 80 && ymdhms.year <= 99)
    ymdhms.year = ymdhms.year + 1900;
  if (ymdhms.year >= 0 && ymdhms.year <= 79)
    ymdhms.year = ymdhms.year + 2000;

  temp = inputRec.substr(4, 2);
  if (getLong(temp, tempL))
    ymdhms.month = tempL;

  temp = inputRec.substr(7, 2);
  if (getLong(temp, tempL))
    ymdhms.day = tempL;

  temp = inputRec.substr(10, 2);
  if (getLong(temp, tempL))
    ymdhms.hour = tempL;

  temp = inputRec.substr(13, 2);
  if (getLong(temp, tempL))
    ymdhms.min = tempL;

  temp = inputRec.substr(16, 2);
  if (getDouble(temp, tempD))
    ymdhms.sec = tempD;

  if (!validYMDHMS(
          ymdhms.year, ymdhms.month, ymdhms.day, ymdhms.hour, ymdhms.min,
          ymdhms.sec, warningString)) {
    tempStream << "Problems reading Date and Time in MET record: "
               << warningString << inputRec << endl;
    appendToWarningMessages(tempStream.str());
  }

  DateTime tempDateTime(ymdhms);
  epoch.setEpochTime(tempDateTime);

  for (i = 0; i < numMetTypes; i++) {
    temp = inputRec.substr((18 + i * 7), 7);
    if (!blankString(temp) && getDouble(temp, tempD)) {
      MetSet tempMetSet;
      tempMetSet.obsPresent = true;
      tempMetSet.observation = tempD;
      tempMetSet.metType = obsTypeList[i];
      epoch.setMetListElement(tempMetSet, i);
    } else {
      MetSet tempMetSet;
      tempMetSet.obsPresent = false;
      tempMetSet.observation = 0.0;
      tempMetSet.metType = obsTypeList[i];
      epoch.setMetListElement(tempMetSet, i);
    }
  }

  return 1;
}

// Selectors
string RinexMetFile::getMarkerName() {
  return markerName;
}
string RinexMetFile::getMarkerNumber() {
  return markerNumber;
}
unsigned short RinexMetFile::getNumMetTypes() {
  return numMetTypes;
}
enum METTYPE RinexMetFile::getObsTypeListElement(int i) {
  return obsTypeList[i];
}
SensorInfo RinexMetFile::getSensorModAccurElement(int i) {
  return sensorModAccur[i];
}
SensorPosition RinexMetFile::getSensorXYZhElement(int i) {
  return sensorXYZh[i];
}

void RinexMetFile::writeHeaderImage(ofstream& outputStream) {
  rinexHeaderImage.writeHeaderImage(outputStream);
}

void RinexMetFile::writeEpoch(ofstream& outputStream, MetEpoch& outputEpoch) {
  unsigned short i;
  YMDHMS ymdhms;
  DateTime tempDateTime;

  // add 0.001 seconds to allow for round off error in ymdhms.sec
  tempDateTime = outputEpoch.getEpochTime() + (0.001 / 86400.0);
  outputEpoch.setEpochTime(tempDateTime);
  ymdhms = outputEpoch.getEpochTime().GetYMDHMS();

  outputStream << " " << setw(2) << setfill('0') << ymdhms.year % 100;
  outputStream << " " << setw(2) << setfill(' ') << ymdhms.month;
  outputStream << " " << setw(2) << ymdhms.day;
  outputStream << " " << setw(2) << ymdhms.hour;
  outputStream << " " << setw(2) << ymdhms.min;
  outputStream.setf(ios::fixed);
  outputStream.unsetf(ios::showpoint);
  outputStream << " " << setw(2) << setprecision(0) << ymdhms.sec;

  outputStream.setf(ios::fixed);
  outputStream.setf(ios::showpoint);
  for (i = 0; i < numMetTypes; i++) {
    outputStream << setw(7) << setprecision(1)
                 << outputEpoch.getMetListElement(i).observation;
  }
  outputStream << endl;
}

void RinexMetFile::incrementNumberMetEpochs(unsigned int n) {
  numberMetEpochs = numberMetEpochs + n;  // n is usually one
}

unsigned int RinexMetFile::getNumberMetEpochs() {
  return (numberMetEpochs);
}

unsigned int RinexMetFile::getMetFilesCount() {
  return (numberMetFiles);
}

//========================== ClockDataFile Class ===============================

// Initialize static data member
unsigned short ClockDataFile::numberClkFiles = 0;  // no objects yet

// Constructors
ClockDataFile::ClockDataFile() : RinexFile() {
  initializeData();
  numberClkEpochs = 0;
  numberClkFiles++;
}

ClockDataFile::ClockDataFile(string inputFilePath, ios::openmode mode)
    : RinexFile(inputFilePath, mode) {
  initializeData();
  numberClkEpochs = 0;
  numberClkFiles++;
}

// Destructor
ClockDataFile::~ClockDataFile() {
  --numberClkFiles;
}

// Initializers
bool ClockDataFile::setLeapSec(unsigned short input) {
  leapSec = input;
  return true;
}

bool ClockDataFile::setNumberClkTypes(unsigned short input) {
  numberClkTypes = input;
  return true;
}

bool ClockDataFile::setClkTypeListElement(enum CLKTYPE input, int i) {
  clkTypeList[i] = input;
  return true;
}

bool ClockDataFile::setStationName(string input) {
  stationName = input;
  return true;
}

bool ClockDataFile::setStationNumber(string input) {
  stationNumber = input;
  return true;
}

bool ClockDataFile::setStationClkRef(string input) {
  stationClkRef = input;
  return true;
}

bool ClockDataFile::setACDesignator(string input) {
  ACDesignator = input;
  return true;
}

bool ClockDataFile::setAnalysisCenterName(string input) {
  analysisCenterName = input;
  return true;
}

bool ClockDataFile::setNumberAnalysisClockRef(unsigned short input) {
  numberAnalysisClkRef = input;
  return true;
}

bool ClockDataFile::setAnalysisStartEpoch(DateTime input) {
  analysisStartEpoch = input;
  return true;
}

bool ClockDataFile::setAnalysisStopEpoch(DateTime input) {
  analysisStopEpoch = input;
  return true;
}

bool ClockDataFile::setNumberSolnSta(unsigned short input) {
  numberSolnSta = input;
  return true;
}

bool ClockDataFile::setTerrRefFrameOrSinex(string input) {
  terrRefFrameOrSinex = input;
  return true;
}

bool ClockDataFile::setNumberSolnSatellites(unsigned short input) {
  numberSolnSatellites = input;
  return true;
}

bool ClockDataFile::setClkRefListElement(AnalysisClkRefData input, int i) {
  clkRefList[i] = input;
  return true;
}

bool ClockDataFile::setSolnStaListElement(SolnStaNameData input, int i) {
  solnStaList[i] = input;
  return true;
}

bool ClockDataFile::setPrnListElement(string input, int i) {
  prnList[i] = input;
  return true;
}

unsigned short ClockDataFile::readHeader() {
  unsigned short i;
  string inputRec;
  string temp;
  string recordReadIn;
  HeaderRecord headerRec;
  ostringstream sstemp;
  bool endOfHeaderFound = false;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    LeapSecRec,
    DataTypeRec,
    StationNameRec,
    StaClkRefRec,
    ACNameRec,
    NumClkRefRec,
    AnalysisClkRec,
    NumSolnStaRec,
    SolnStaNameRec,
    NumSolSvsRec,
    PRNListRec,
    EndHeaderRec
  };

  if (validFirstLine(recordReadIn)) {
    headerRecs[VersionRec].numberPresent++;
    headerRec.SetHeaderRecord(recordReadIn);
    rinexHeaderImage.appendHeaderRecord(recordReadIn);
  } else {
    tempStream << "Error: First Line is incorrect in File: "
               << getPathFilename() << endl
               << recordReadIn << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  // read from line 2 until the end of header
  while (!endOfHeaderFound) {
    if (!getline(inputStream, inputRec, '\n')) {
      // Error reading a header line of the CLK File
      tempStream << "Error, cannot read a header line from file:" << endl
                 << getPathFilename() << endl;
      appendToErrorMessages(tempStream.str());

      RinexReadingException excep(tempStream.str());
      throw excep;
    }
    incrementNumberLinesRead(1);

    if (blankString(inputRec)) {
      if (formatVersion < 2.0) {
        endOfHeaderFound = true;
        break;  // exit the while loop
      } else {
        tempStream << "Warning! On line: " << getNumberLinesRead() << ":"
                   << endl
                   << inputRec << endl
                   << " Not a Valid Header record, skipping to next line. "
                   << endl;
        appendToWarningMessages(tempStream.str());
        continue;  // go to read the next record
      }
    }

    if (validHeaderRecord(inputRec)) {
      headerRec.SetHeaderRecord(inputRec);
      rinexHeaderImage.appendHeaderRecord(headerRec);
      if (inputRec.find("END OF HEADER") == 60)
        endOfHeaderFound = true;
      continue;  // go to read the next record
    }

  }  // end of while loop over all header records

  // Decide which header records need to be required based on clock data types
  for (i = 0; i < numberClkTypes; i++) {
    if (clkTypeList[i] == AR) {
      headerRecs[7].required = true;   // ANALYSIS CENTER
      headerRecs[8].required = true;   // # OF CLK REF
      headerRecs[9].required = true;   // ANALYSIS CLK REF
      headerRecs[10].required = true;  // # OF SOLN STA / TRF
      headerRecs[11].required = true;  // SOLN STA NAME / NUM
    } else if (clkTypeList[i] == AS) {
      headerRecs[7].required = true;   // ANALYSIS CENTER
      headerRecs[8].required = true;   // # OF CLK REF
      headerRecs[9].required = true;   // ANALYSIS CLK REF
      headerRecs[10].required = true;  // # OF SOLN STA / TRF
      headerRecs[11].required = true;  // SOLN STA NAME / NUM
      headerRecs[12].required = true;  // # OF SOLN SATS
      headerRecs[13].required = true;  // PRN LIST
    } else if (clkTypeList[i] == CR) {
      headerRecs[5].required = true;  // STATION NAME / NUM
      headerRecs[6].required = true;  // STATION CLK REF
    } else if (clkTypeList[i] == DR) {
      headerRecs[5].required = true;  // STATION NAME / NUM
    } else if (clkTypeList[i] == MS) {
      headerRecs[7].required = true;  // ANALYSIS CENTER
    } else {
      tempStream << getPathFilename()
                 << " has an illegal clock data type:" << endl
                 << clkTypeList[i] << endl;
      appendToErrorMessages(tempStream.str());
    }
  }

  // Check for ALL required header records (based on # / TYPES OF DATA)
  int numberRequiredErrors = 0;
  for (i = 0; i < MAXCLKHEADERRECTYPES; i++) {
    // Check for the universal required header records
    if (headerRecs[i].numberPresent == 0 && headerRecs[i].required) {
      sstemp << " missing Header record: " << headerRecs[i].recID << endl;
      numberRequiredErrors++;
    }
  }

  // Loop over data types to check for data-specfic required-header-records

  if (numberRequiredErrors > 0) {
    tempStream << getPathFilename() << " has the following missing Header "
               << "records." << endl
               << sstemp.str() << endl;
    appendToErrorMessages(tempStream.str());

    RequiredRecordMissingException excep(tempStream.str());
    throw excep;
  }

  return (0);
}

bool ClockDataFile::validHeaderRecord(string inputRec) {
  unsigned short i;
  string temp;
  string warningString;
  HeaderRecord headerRec;
  int indexAnalysisClkRefRecs = -1;
  int indexSolnStaRecs = -1;
  ostringstream sstemp;
  long tempL;
  double tempD;
  YMDHMS ymdhms;

  enum {
    VersionRec,
    PgmRunByRec,
    CommentRec,
    LeapSecRec,
    DataTypeRec,
    StationNameRec,
    StaClkRefRec,
    ACNameRec,
    NumClkRefRec,
    AnalysisClkRec,
    NumSolnStaRec,
    SolnStaNameRec,
    NumSolSvsRec,
    PRNListRec,
    EndHeaderRec
  };

  if (inputRec.find("COMMENT") == 60) {
    // Comment records are optional, may be more than one.
    // all comments are stored in the headerImage linked-list
    headerRecs[CommentRec].numberPresent++;
    return true;
  } else if (inputRec.find("PGM / RUN BY / DATE") == 60) {
    headerRecs[PgmRunByRec].numberPresent++;

    rinexProgram = inputRec.substr(0, 20);
    createdByAgency = inputRec.substr(20, 20);
    dateFileCreated = inputRec.substr(40, 20);
    return true;
  } else if (inputRec.find("LEAP SECONDS") == 60) {
    headerRecs[LeapSecRec].numberPresent++;
    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      leapSec = static_cast<unsigned short>(tempL);
    return true;
  } else if (inputRec.find("# / TYPES OF DATA") == 60) {
    string obsType;
    headerRecs[DataTypeRec].numberPresent++;

    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      numberClkTypes = static_cast<unsigned short>(tempL);

    temp = inputRec.substr(6, 54);
    istringstream nextObs(temp);

    for (i = 0; i < MAXCLKTYPES; i++) {
      clkTypeList[i] = NOCLK;
    }

    i = 0;
    while (nextObs >> obsType) {
      if (obsType.find("AR") != string::npos)
        clkTypeList[i] = AR;
      if (obsType.find("AS") != string::npos)
        clkTypeList[i] = AS;
      if (obsType.find("CR") != string::npos)
        clkTypeList[i] = CR;
      if (obsType.find("DR") != string::npos)
        clkTypeList[i] = DR;
      if (obsType.find("MS") != string::npos)
        clkTypeList[i] = MS;
      i++;
    }
    return true;

  } else if (inputRec.find("STATION NAME / NUM") == 60) {
    headerRecs[StationNameRec].numberPresent++;
    stationName = inputRec.substr(0, 4);
    stationNumber = inputRec.substr(5, 20);
    return true;
  } else if (inputRec.find("STATION CLK REF") == 60) {
    headerRecs[StaClkRefRec].numberPresent++;
    stationClkRef = inputRec.substr(0, 60);
    return true;
  } else if (inputRec.find("ANALYSIS CENTER") == 60) {
    headerRecs[ACNameRec].numberPresent++;
    ACDesignator = inputRec.substr(0, 3);
    analysisCenterName = inputRec.substr(5, 55);
    return true;
  } else if (inputRec.find("# OF CLK REF") == 60) {
    headerRecs[NumClkRefRec].numberPresent++;
    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      numberAnalysisClkRef = static_cast<unsigned short>(tempL);

    temp = inputRec.substr(7, 4);
    if (getLong(temp, tempL))
      ymdhms.year = tempL;

    temp = inputRec.substr(11, 3);
    if (getLong(temp, tempL))
      ymdhms.month = tempL;

    temp = inputRec.substr(14, 3);
    if (getLong(temp, tempL))
      ymdhms.day = tempL;

    temp = inputRec.substr(17, 3);
    if (getLong(temp, tempL))
      ymdhms.hour = tempL;

    temp = inputRec.substr(20, 3);
    if (getLong(temp, tempL))
      ymdhms.min = tempL;

    temp = inputRec.substr(23, 10);
    if (getDouble(temp, tempD))
      ymdhms.sec = tempD;

    if (!validYMDHMS(
            ymdhms.year, ymdhms.month, ymdhms.day, ymdhms.hour, ymdhms.min,
            ymdhms.sec, warningString)) {
      tempStream << "Problems reading Date & Time in Analysis Start Time:"
                 << endl
                 << warningString << endl
                 << inputRec << endl;
      appendToWarningMessages(tempStream.str());
    }

    temp = inputRec.substr(34, 4);
    if (getLong(temp, tempL))
      ymdhms.year = tempL;

    temp = inputRec.substr(38, 3);
    if (getLong(temp, tempL))
      ymdhms.month = tempL;

    temp = inputRec.substr(41, 3);
    if (getLong(temp, tempL))
      ymdhms.day = tempL;

    temp = inputRec.substr(44, 3);
    if (getLong(temp, tempL))
      ymdhms.hour = tempL;

    temp = inputRec.substr(47, 3);
    if (getLong(temp, tempL))
      ymdhms.min = tempL;

    temp = inputRec.substr(50, 10);
    if (getDouble(temp, tempD))
      ymdhms.sec = tempD;

    if (!validYMDHMS(
            ymdhms.year, ymdhms.month, ymdhms.day, ymdhms.hour, ymdhms.min,
            ymdhms.sec, warningString)) {
      tempStream << "Problems reading Date & Time in Analysis Stop Time:"
                 << endl
                 << warningString << endl
                 << inputRec << endl;
      appendToWarningMessages(tempStream.str());
    }

    clkRefList = new AnalysisClkRefData[numberAnalysisClkRef];

    return true;
  } else if (inputRec.find("ANALYSIS CLK REF") == 60) {
    indexAnalysisClkRefRecs++;
    headerRecs[AnalysisClkRec].numberPresent++;

    clkRefList[indexAnalysisClkRefRecs].rcvrSatName = inputRec.substr(0, 4);
    clkRefList[indexAnalysisClkRefRecs].refClockID = inputRec.substr(5, 20);
    temp = inputRec.substr(40, 19);
    if (getDouble(temp, tempD))
      clkRefList[indexAnalysisClkRefRecs].aprioriClkConstraint = tempD;
    return true;
  } else if (inputRec.find("# OF SOLN STA / TRF") == 60) {
    headerRecs[NumSolnStaRec].numberPresent++;
    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      numberSolnSta = static_cast<unsigned short>(tempL);
    terrRefFrameOrSinex = inputRec.substr(10, 50);
    solnStaList = new SolnStaNameData[numberSolnSta];
    return true;
  } else if (inputRec.find("SOLN STA NAME / NUM") == 60) {
    indexSolnStaRecs++;
    headerRecs[SolnStaNameRec].numberPresent++;

    solnStaList[indexSolnStaRecs].staRcvrName = inputRec.substr(0, 4);
    solnStaList[indexSolnStaRecs].staRcvrID = inputRec.substr(5, 20);

    temp = inputRec.substr(25, 11);
    if (getDouble(temp, tempD))
      solnStaList[indexSolnStaRecs].staX = tempD;
    temp = inputRec.substr(37, 11);
    if (getDouble(temp, tempD))
      solnStaList[indexSolnStaRecs].staY = tempD;
    temp = inputRec.substr(49, 11);
    if (getDouble(temp, tempD))
      solnStaList[indexSolnStaRecs].staZ = tempD;

    return true;
  } else if (inputRec.find("# OF SOLN SATS") == 60) {
    headerRecs[NumSolSvsRec].numberPresent++;
    temp = inputRec.substr(0, 6);
    if (getLong(temp, tempL))
      numberSolnSatellites = static_cast<unsigned short>(tempL);
    prnList = new string[numberSolnSatellites];

    return true;
  } else if (inputRec.find("PRN LIST") == 60) {
    headerRecs[PRNListRec].numberPresent++;

    if (numberSolnSatellites <= 15) {
      for (i = 0; i < numberSolnSatellites; i++) {
        prnList[i] = inputRec.substr(i * 3, 3);
      }
    } else if (
        numberSolnSatellites > 15 &&
        headerRecs[PRNListRec].numberPresent == 1) {
      for (i = 0; i < 15; i++) {
        prnList[i] = inputRec.substr(i * 3, 3);
      }
    } else if (
        numberSolnSatellites > 15 &&
        headerRecs[PRNListRec].numberPresent == 2) {
      for (int k = 15; k < numberSolnSatellites; k++) {
        i = k - 15;
        prnList[k] = inputRec.substr(i * 3, 3);
      }
    }

    return true;
  } else if (inputRec.find("END OF HEADER") == 60) {
    headerRecs[EndHeaderRec].numberPresent++;
    return true;
  } else {
    tempStream << "Invalid CLK header record: " << endl << inputRec << endl;
    appendToWarningMessages(tempStream.str());
    return false;
  }
}

void ClockDataFile::initializeData() {
  int i;

  leapSec = 0;
  numberClkTypes = 0;
  stationName = "";
  stationNumber = "";
  stationClkRef = "";
  ACDesignator = "";
  analysisCenterName = "";
  numberAnalysisClkRef = 0;
  analysisStartEpoch.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
  analysisStopEpoch.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
  numberSolnSta = 0;
  terrRefFrameOrSinex = "";
  numberSolnSatellites = 0;

  for (i = 0; i < MAXCLKTYPES; i++) {
    clkTypeList[i] = NOCLK;
  }

  for (i = 0; i < MAXCLKHEADERRECTYPES; i++)
    headerRecs[i].numberPresent = 0;

  headerRecs[0].recID = "RINEX VERSION / TYPE";
  headerRecs[0].required = true;

  headerRecs[1].recID = "PGM / RUN BY / DATE";
  headerRecs[1].required = true;

  headerRecs[2].recID = "COMMENT";
  headerRecs[2].required = false;

  headerRecs[3].recID = "LEAP SECONDS";
  headerRecs[3].required = false;

  headerRecs[4].recID = "# / TYPES OF DATA";
  headerRecs[4].required = true;

  headerRecs[5].recID = "STATION NAME / NUM";  // required for CR, DR
  headerRecs[5].required = false;

  headerRecs[6].recID = "STATION CLK REF";  // required for CR
  headerRecs[6].required = false;

  headerRecs[7].recID = "ANALYSIS CENTER";  // required for AR, AS, MS
  headerRecs[7].required = false;

  headerRecs[8].recID = "# OF CLK REF";  // required for AR, AS
  headerRecs[8].required = false;

  headerRecs[9].recID = "ANALYSIS CLK REF";  // required for AR, AS
  headerRecs[9].required = false;

  headerRecs[10].recID = "# OF SOLN STA / TRF";  // required for AR, AS
  headerRecs[10].required = false;

  headerRecs[11].recID = "SOLN STA NAME / NUM";  // required for AR, AS
  headerRecs[11].required = false;

  headerRecs[12].recID = "# OF SOLN SATS";  // required for AS
  headerRecs[12].required = false;

  headerRecs[13].recID = "PRN LIST";  // required for AS
  headerRecs[13].required = false;

  headerRecs[14].recID = "END OF HEADER";
  headerRecs[14].required = true;
}

void ClockDataFile::initEpochInfo(ClkEpoch& epoch) {
  DateTime tempdt;
  tempdt.SetYMDHMS(9999, 1, 1, 0, 0, 0.0);
  string tempStr = "";

  epoch.setClockDataType(NOCLK);
  epoch.setRecvrSatName(tempStr);
  epoch.setEpochTime(tempdt);
  epoch.setNumberDataValues(0);
  epoch.setClockBias(0.0);
  epoch.setClockBiasSigma(0.0);
  epoch.setClockRate(0.0);
  epoch.setClockRateSigma(0.0);
  epoch.setClockAcceleration(0.0);
  epoch.setClockAccelSigma(0.0);
}

unsigned short ClockDataFile::readEpoch(ClkEpoch& epoch) {
  string warningString;
  string inputRec;
  size_t l;
  YMDHMS ymdhms;
  int i;
  string temp;
  string working;
  long tempL;
  double tempD;
  string BlankString("                                        ");
  BlankString.append("                                        ");

  // Read epoch "time-tag" record.

  if (!getline(inputStream, inputRec, '\n')) {
    return 0;  // EOF encountered
  }
  initEpochInfo(epoch);
  makeRecordLength80(inputRec);
  incrementNumberLinesRead(1);

  temp = inputRec.substr(0, 2);
  if (temp == "AR")
    epoch.setClockDataType(AR);
  else if (temp == "AS")
    epoch.setClockDataType(AS);
  else if (temp == "CR")
    epoch.setClockDataType(CR);
  else if (temp == "DR")
    epoch.setClockDataType(DR);
  else if (temp == "MS")
    epoch.setClockDataType(MS);
  else  // problem reading  clock data type for this epoch
  {
    tempStream << "Warning! Cannot read Clock Data Type on line:" << endl
               << inputRec << endl;
    appendToWarningMessages(tempStream.str());
    epoch.setClockDataType(NOCLK);
  }

  temp = inputRec.substr(3, 4);
  epoch.setRecvrSatName(temp);

  temp = inputRec.substr(8, 4);
  if (getLong(temp, tempL))
    ymdhms.year = tempL;

  temp = inputRec.substr(13, 2);
  if (getLong(temp, tempL))
    ymdhms.month = tempL;

  temp = inputRec.substr(16, 2);
  if (getLong(temp, tempL))
    ymdhms.day = tempL;

  temp = inputRec.substr(19, 2);
  if (getLong(temp, tempL))
    ymdhms.hour = tempL;

  temp = inputRec.substr(22, 2);
  if (getLong(temp, tempL))
    ymdhms.min = tempL;

  temp = inputRec.substr(24, 10);
  if (getDouble(temp, tempD))
    ymdhms.sec = tempD;

  if (!validYMDHMS(
          ymdhms.year, ymdhms.month, ymdhms.day, ymdhms.hour, ymdhms.min,
          ymdhms.sec, warningString)) {
    tempStream << "Problems reading Date and Time in CLK epoch: " << endl
               << warningString << inputRec << endl;
    appendToWarningMessages(tempStream.str());
  }

  DateTime tempDateTime(ymdhms);
  epoch.setEpochTime(tempDateTime);

  temp = inputRec.substr(34, 3);
  if (getLong(temp, tempL))
    epoch.setNumberDataValues(static_cast<unsigned short>(tempL));
  if (epoch.getNumberDataValues() > 6) {
    tempStream << "Warning ! Number Data Values at epoch > 6 : " << endl
               << inputRec << endl;
    appendToWarningMessages(tempStream.str());
    epoch.setNumberDataValues(6);
  }

  if (epoch.getNumberDataValues() >= 1) {
    temp = inputRec.substr(40, 19);
    if (getDouble(temp, tempD))
      epoch.setClockBias(tempD);
  }
  if (epoch.getNumberDataValues() >= 2) {
    temp = inputRec.substr(60, 19);
    if (getDouble(temp, tempD))
      epoch.setClockBiasSigma(tempD);
  }

  if (epoch.getNumberDataValues() >= 3)  // read a second line of clock data
  {
    if (!getline(inputStream, inputRec, '\n')) {
      tempStream << "Unexpected EOF reading 2nd line of clock data:" << endl
                 << inputRec << endl;
      appendToWarningMessages(tempStream.str());
      return 0;  // EOF encountered
    }
    incrementNumberLinesRead(1);
    makeRecordLength80(inputRec);

    if (epoch.getNumberDataValues() >= 3) {
      temp = inputRec.substr(0, 19);
      if (getDouble(temp, tempD))
        epoch.setClockRate(tempD);
    }
    if (epoch.getNumberDataValues() >= 4) {
      temp = inputRec.substr(20, 19);
      if (getDouble(temp, tempD))
        epoch.setClockRateSigma(tempD);
    }
    if (epoch.getNumberDataValues() >= 5) {
      temp = inputRec.substr(40, 19);
      if (getDouble(temp, tempD))
        epoch.setClockAcceleration(tempD);
    }
    if (epoch.getNumberDataValues() >= 6) {
      temp = inputRec.substr(60, 19);
      if (getDouble(temp, tempD))
        epoch.setClockAccelSigma(tempD);
    }

  }  // end of if-stmt for reading second line of clock data

  return 1;
}

// Selectors
unsigned short ClockDataFile::getLeapSec() {
  return leapSec;
}
unsigned short ClockDataFile::getNumberClkTypes() {
  return numberClkTypes;
}
enum CLKTYPE ClockDataFile::getClkTypeListElement(int i) {
  return clkTypeList[i];
}
string ClockDataFile::getStationName() {
  return stationName;
}
string ClockDataFile::getStationNumber() {
  return stationNumber;
}
string ClockDataFile::getStationClkRef() {
  return stationClkRef;
}
string ClockDataFile::getACDesignator() {
  return ACDesignator;
}
string ClockDataFile::getAnalysisCenterName() {
  return analysisCenterName;
}

unsigned short ClockDataFile::getNumberAnalysisClockRef() {
  return numberAnalysisClkRef;
}
DateTime ClockDataFile::getAnalysisStartEpoch() {
  return analysisStartEpoch;
}
DateTime ClockDataFile::getAnalysisStopEpoch() {
  return analysisStopEpoch;
}
unsigned short ClockDataFile::getNumberSolnSta() {
  return numberSolnSta;
}
string ClockDataFile::getTerrRefFrameOrSinex() {
  return terrRefFrameOrSinex;
}
unsigned short ClockDataFile::getNumberSolnSatellites() {
  return numberSolnSatellites;
}

AnalysisClkRefData ClockDataFile::getClkRefListElement(int i) {
  return clkRefList[i];
}
SolnStaNameData ClockDataFile::getSolnStaListElement(int i) {
  return solnStaList[i];
}
string ClockDataFile::getPrnListElement(int i) {
  return prnList[i];
}

void ClockDataFile::writeHeaderImage(ofstream& outputStream) {
  rinexHeaderImage.writeHeaderImage(outputStream);
}

void ClockDataFile::writeEpoch(ofstream& outputStream, ClkEpoch& outputEpoch) {
  string theClockDataType;
  unsigned short i;
  YMDHMS ymdhms;
  DateTime tempDateTime;

  // add 0.001 seconds to allow for round off error in ymdhms.sec
  tempDateTime = outputEpoch.getEpochTime() + (0.001 / 86400.0);
  outputEpoch.setEpochTime(tempDateTime);

  ymdhms = outputEpoch.getEpochTime().GetYMDHMS();

  if (outputEpoch.getClockDataType() == AR)
    theClockDataType = "AR";
  else if (outputEpoch.getClockDataType() == AS)
    theClockDataType = "AS";
  else if (outputEpoch.getClockDataType() == CR)
    theClockDataType = "CR";
  else if (outputEpoch.getClockDataType() == DR)
    theClockDataType = "DR";
  else if (outputEpoch.getClockDataType() == MS)
    theClockDataType = "MS";

  outputStream.setf(ios::fixed, ios::floatfield);
  outputStream.setf(ios::showpoint);
  outputStream << setw(2) << theClockDataType << " " << setw(4)
               << outputEpoch.getRecvrSatName() << " " << setw(4) << ymdhms.year
               << " " << setw(2) << setfill(' ') << ymdhms.month << " "
               << setw(2) << ymdhms.day << " " << setw(2) << ymdhms.hour << " "
               << setw(2) << ymdhms.min << setw(10) << setprecision(6)
               << ymdhms.sec << setw(3) << outputEpoch.getNumberDataValues()
               << "   ";

  outputStream.setf(ios::scientific, ios::floatfield);
  if (outputEpoch.getNumberDataValues() >= 1)
    outputStream << setw(19) << setprecision(12) << outputEpoch.getClockBias();
  if (outputEpoch.getNumberDataValues() >= 2)
    outputStream << " " << setw(19) << setprecision(12)
                 << outputEpoch.getClockBiasSigma();
  outputStream << endl;  // end of first data line

  if (outputEpoch.getNumberDataValues() >= 3) {
    outputStream.setf(ios::scientific, ios::floatfield);
    if (outputEpoch.getNumberDataValues() >= 3)
      outputStream << setw(19) << setprecision(12)
                   << outputEpoch.getClockRate();
    if (outputEpoch.getNumberDataValues() >= 4)
      outputStream << " " << setw(19) << setprecision(12)
                   << outputEpoch.getClockRateSigma();
    if (outputEpoch.getNumberDataValues() >= 5)
      outputStream << " " << setw(19) << setprecision(12)
                   << outputEpoch.getClockAcceleration();
    if (outputEpoch.getNumberDataValues() >= 6)
      outputStream << " " << setw(19) << setprecision(12)
                   << outputEpoch.getClockAccelSigma();

    outputStream << endl;  // end of second data line

  }  // do this only if a second line of data values is needed
}

void ClockDataFile::incrementNumberClkEpochs(unsigned int n) {
  numberClkEpochs = numberClkEpochs + n;  // n is usually one
}

unsigned short ClockDataFile::getNumberClkEpochs() {
  return (numberClkEpochs);
}

unsigned short ClockDataFile::getClkFilesCount() {
  return (numberClkFiles);
}

//=================== RequiredRecordMissingException ==========================

RequiredRecordMissingException::RequiredRecordMissingException(
    const string& errMsg)
    : ErrorMessage(errMsg) {}

string RequiredRecordMissingException::getMessage() {
  return (ErrorMessage);
}

//========================== RinexFileException ===============================

RinexFileException::RinexFileException(const string& errMsg)
    : ErrorMessage(errMsg) {}

string RinexFileException::getMessage() {
  return (ErrorMessage);
}
//========================== RinexReadingException
//===============================

RinexReadingException::RinexReadingException(const string& errMsg)
    : ErrorMessage(errMsg) {}

string RinexReadingException::getMessage() {
  return (ErrorMessage);
}

}  // namespace NGSrinex
