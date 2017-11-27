// rinex.h

// class definitions for RINEX file objects and prototypes for
// RINEX file interaction methods.

// ver. 200108.17

#if !defined(__RINEX__)
#define __RINEX__

#if !defined(FSTREAM_)
#include <fstream>
#define FSTREAM_
#endif

#if !defined(IOSTREAM_)
#include <iostream>
#define IOSTREAM_
#endif

#if !defined(SSTREAM_)
#include <sstream>
#define SSTREAM_
#endif

#if !defined(STRING_)
#include <string>
#define STRING_
#endif

#if !defined(LIST_)
#include <list>
#define LIST_
#endif

#if !defined(DATETIME_H_)
#include "datetime.h"
#define DATETIME_H_
#endif

#include <climits>

namespace NGSrinex {

using namespace std;
using namespace NGSdatetime;

//======================== constants =====================================

const unsigned short MAXOBSHEADERRECTYPES = 20;
const unsigned short MAXNAVHEADERRECTYPES = 8;
const unsigned short MAXGLONAVHEADERRECTYPES = 6;
const unsigned short MAXGEONAVHEADERRECTYPES = 6;
const unsigned short MAXMETHEADERRECTYPES = 9;
const unsigned short MAXCLKHEADERRECTYPES = 15;

const unsigned short NUMREQROBSHEADERREC = 12;
const unsigned short NUMREQRNAVHEADERREC = 3;
const unsigned short NUMREQRMETHEADERREC = 7;
const unsigned short NUMREQRCLKHEADERREC = 4;

const unsigned short MAXPRNID = 36;
const unsigned short MAXGEOSTATIONARYID = 99;
const unsigned short MAXSATPEREPOCH = 24;
const unsigned short RINEXRECSIZE = 83;  // 80 cols plus \r \n etc.
const unsigned short MAXOBSTYPES = 11;
const unsigned short MAXMETTYPES = 6;
const unsigned short MAXCLKTYPES = 5;

enum OBSTYPE {
  NOOBS = 0,
  L1 = 1,
  L2 = 2,
  C1 = 3,
  P1 = 4,
  P2 = 5,
  D1 = 6,
  D2 = 7,
  T1 = 8,
  T2 = 9,
  S1 = 10,
  S2 = 11
};

enum METTYPE { NOMET = 0, PR = 1, TD = 2, HR = 3, ZW = 4, ZD = 5, ZT = 6 };

enum CLKTYPE { NOCLK = 0, AR = 1, AS = 2, CR = 3, DR = 4, MS = 5 };

//=================== Classes with public data only ==========================
class RecStruct {
 public:
  RecStruct() {
    numberPresent = 0;
    required = false;
    recID = "";
  };
  unsigned short numberPresent;  // number of this record found in file.
  bool required;                 // True or False.
  string recID;                  // label for this type of Header record.
};

class ObsSet {
 public:
  ObsSet() {
    obsPresent = false;
    observation = 0.0;
    obsType = NOOBS;
    LLI = 0;
    sigStrength = 0;
  };
  bool obsPresent;             // These types of obs present? True/False.
  double observation;          // Observed value for this PRN and type.
  enum OBSTYPE obsType;        // Type of RINEX OBS observation.
  unsigned short LLI;          // Loss of Lock Indicator:  0, 1, or 2.
  unsigned short sigStrength;  // Signal Strength: 0 - 9.
};

class MetSet {
 public:
  MetSet() {
    obsPresent = false;
    observation = 0.0;
    metType = NOMET;
  };
  bool obsPresent;     // These types of obs present? True/False.
  double observation;  // Observed value for this PRN and type.
  METTYPE metType;     // Type of RINEX MET observation.
};

class SensorInfo {
 public:
  SensorInfo() {
    model = "";
    type = "";
    accuracy = 0.0;
    metObsType = "";
  };
  string model;
  string type;
  double accuracy;
  string metObsType;
};

class SensorPosition {
 public:
  SensorPosition() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    h = 0.0;
    metObsType = "";
  };
  double x;
  double y;
  double z;
  double h;
  string metObsType;
};

class SatObsAtEpoch {
 public:
  SatObsAtEpoch() {
    satCode = ' ';
    satNum = 9999;
    for (int i = 0; i < MAXOBSTYPES; i++) {
      obsList[i].obsPresent = false;
      obsList[i].observation = 0.0;
      obsList[i].obsType = NOOBS;
      obsList[i].LLI = 0;
      obsList[i].sigStrength = 0;
    }
  };
  char satCode;           // G=GPS,R=GLONASS,S=GEOSTATIONARY,T=NNSS.
  unsigned short satNum;  // Satellite number.
  ObsSet obsList[MAXOBSTYPES];
};

class OneWaveLenRec {
 public:
  OneWaveLenRec() {
    L1Factor = 0;
    L2Factor = 0;
    numSatInRecord = 0;
    for (int i = 0; i < 7; i++)
      satsInRecord[i] = "";
  };
  unsigned short L1Factor;
  unsigned short L2Factor;
  unsigned short numSatInRecord;
  string satsInRecord[7];
};

class ObsCountForPRN {
 public:
  ObsCountForPRN() {
    satCode = ' ';
    satNum = 9999;
    for (int i = 0; i < MAXOBSTYPES; i++)
      PRNObsCount[i] = 0;
  };
  char satCode;                            // G=GPS,R=GLONASS,T=NNSS,
                                           // S=Geostationary
  unsigned short satNum;                   // Satellite number
  unsigned long PRNObsCount[MAXOBSTYPES];  // # obs for this obs.type
};

class AnalysisClkRefData {
 public:
  AnalysisClkRefData() {
    rcvrSatName = "";
    refClockID = "";
    aprioriClkConstraint = 0.0;
  };
  string rcvrSatName;           // receiver or satellite name.
  string refClockID;            // unique ID for reference clock.
  double aprioriClkConstraint;  // optional apriori clock constraint.
};

class SolnStaNameData {
 public:
  SolnStaNameData() {
    staRcvrName = "";
    staRcvrID = "";
    staX = 0.0;
    staY = 0.0;
    staZ = 0.0;
  };
  string staRcvrName;  // 4-char sta/recvr name.
  string staRcvrID;    // unique ID for sta/recvr (DOMES number).
  double staX;         // geocentric X-coordinate for analysis clk.
  double staY;         // geocentric Y-coordinate for analysis clk.
  double staZ;         // geocentric Z-coordinate for analysis clk.
};

//======================== ObsEpoch Class =============================

class ObsEpoch {
 public:
  ObsEpoch();   // default Constructor
  ~ObsEpoch();  // Destructor

  // Initializers
  bool setEpochTime(DateTime input);
  bool setEpochFlag(unsigned short input);
  bool setNumSat(unsigned short input);
  bool setSatListElement(
      SatObsAtEpoch input, unsigned short numObsTypes, int i);
  bool setRecClockOffset(double input);
  bool appendToEpochHeaderRecords(string input);
  bool initializeData();

  // Selectors
  DateTime getEpochTime();
  unsigned short getEpochFlag();
  unsigned short getNumSat();
  SatObsAtEpoch getSatListElement(int i);
  double getRecClockOffset();
  string getEpochHeaderRecords();

 private:
  DateTime epochTime;
  unsigned short epochFlag;  // 0=OK,1=power failure,>1=event flag.
  unsigned short numSat;     // if more than 12 satellites,
                             // then continue on the next line.

  SatObsAtEpoch satList[MAXSATPEREPOCH];
  double recClockOffset;  // receiver clock offset in seconds
  string epochHeaderRecords;
};

//======================== MetEpoch Class =============================

class MetEpoch {
 public:
  MetEpoch();   // Default Constructor
  ~MetEpoch();  // Destructor

  // Initializers
  bool setEpochTime(DateTime input);
  bool setMetListElement(MetSet input, int i);

  // Selectors
  DateTime getEpochTime();
  MetSet getMetListElement(int i);

 private:
  DateTime epochTime;
  MetSet metList[MAXMETTYPES];
};

//======================== ClkEpoch Class =============================

class ClkEpoch {
 public:
  ClkEpoch();   // Default Constructor
  ~ClkEpoch();  // Destructor

  // Initializers
  bool setClockDataType(CLKTYPE input);
  bool setRecvrSatName(string input);
  bool setEpochTime(DateTime input);
  bool setNumberDataValues(unsigned short input);
  bool setClockBias(double input);
  bool setClockBiasSigma(double input);
  bool setClockRate(double input);
  bool setClockRateSigma(double input);
  bool setClockAcceleration(double input);
  bool setClockAccelSigma(double input);

  // Selectors
  CLKTYPE getClockDataType();
  string getRecvrSatName();
  DateTime getEpochTime();
  unsigned short getNumberDataValues();
  double getClockBias();
  double getClockBiasSigma();
  double getClockRate();
  double getClockRateSigma();
  double getClockAcceleration();
  double getClockAccelSigma();

 private:
  CLKTYPE clockDataType;
  string recvrSatName;
  DateTime epochTime;
  unsigned short numberDataValues;
  double clockBias;
  double clockBiasSigma;
  double clockRate;
  double clockRateSigma;
  double clockAcceleration;
  double clockAccelSigma;
};

//======================== PRNBlock Class =============================

class PRNBlock {
 public:
  PRNBlock();   // Default Constructor
  ~PRNBlock();  // Destructor

  // Initializers
  bool setSatellitePRN(unsigned short input);
  bool setTocYear(unsigned short input);
  bool setTocMonth(unsigned short input);
  bool setTocDay(unsigned short input);
  bool setTocHour(unsigned short input);
  bool setTocMin(unsigned short input);
  bool setTocSec(double input);
  bool setClockBias(double input);
  bool setClockDrift(double input);
  bool setClockDriftRate(double input);

  bool setIode(double input);
  bool setCrs(double input);
  bool setDeltan(double input);
  bool setMo(double input);

  bool setCuc(double input);
  bool setEccen(double input);
  bool setCus(double input);
  bool setSqrtA(double input);

  bool setToe(double input);
  bool setCic(double input);
  bool setBigOmega(double input);
  bool setCis(double input);

  bool setIo(double input);
  bool setCrc(double input);
  bool setLilOmega(double input);
  bool setBigOmegaDot(double input);

  bool setIdot(double input);
  bool setCodesOnL2(double input);
  bool setToeGPSWeek(double input);
  bool setPDataFlagL2(double input);

  bool setSvAccur(double input);
  bool setSvHealth(double input);
  bool setTgd(double input);
  bool setIodc(double input);

  bool setTransmTime(double input);
  bool setFitInterval(double input);
  bool setSpare1(double input);
  bool setSpare2(double input);

  // Selectors
  unsigned short getSatellitePRN();
  unsigned short getTocYear();
  unsigned short getTocMonth();
  unsigned short getTocDay();
  unsigned short getTocHour();
  unsigned short getTocMin();
  double getTocSec();
  double getClockBias();
  double getClockDrift();
  double getClockDriftRate();

  double getIode();
  double getCrs();
  double getDeltan();
  double getMo();

  double getCuc();
  double getEccen();
  double getCus();
  double getSqrtA();

  double getToe();
  double getCic();
  double getBigOmega();
  double getCis();

  double getIo();
  double getCrc();
  double getLilOmega();
  double getBigOmegaDot();

  double getIdot();
  double getCodesOnL2();
  double getToeGPSWeek();
  double getPDataFlagL2();

  double getSvAccur();
  double getSvHealth();
  double getTgd();
  double getIodc();

  double getTransmTime();
  double getFitInterval();
  double getSpare1();
  double getSpare2();

 private:
  unsigned short satellitePRN;
  unsigned short tocYear;
  unsigned short tocMonth;
  unsigned short tocDay;
  unsigned short tocHour;
  unsigned short tocMin;
  double tocSec;
  double clockBias;
  double clockDrift;
  double clockDriftRate;
  double iode;
  double crs;
  double deltan;
  double mo;
  double cuc;
  double eEccen;
  double cus;
  double sqrtA;
  double toe;
  double cic;
  double bigOmega;
  double cis;
  double io;
  double crc;
  double lilOmega;
  double bigOmegaDot;
  double idot;
  double codesOnL2;
  double toeGPSWeek;
  double pDataFlagL2;
  double svAccur;
  double svHealth;
  double tgd;
  double iodc;
  double transmTime;
  double fitInterval;
  double spare1;
  double spare2;
};

//===================== GlonassEphemEpoch Class ======================

class GlonassEphemEpoch {
 public:
  GlonassEphemEpoch();   // Default Constructor
  ~GlonassEphemEpoch();  // Destructor

  // Initializers
  bool setSatelliteAlmanacNumber(unsigned short input);
  bool setEpochYear(unsigned short input);
  bool setEpochMonth(unsigned short input);
  bool setEpochDay(unsigned short input);
  bool setEpochHour(unsigned short input);
  bool setEpochMin(unsigned short input);
  bool setEpochSec(double input);
  bool setSvClockBias(double input);
  bool setSvRelFreqBias(double input);
  bool setMessageFrameTime(double input);
  bool setPosX(double input);
  bool setVelX(double input);
  bool setAccX(double input);
  bool setSvHealth(double input);
  bool setPosY(double input);
  bool setVelY(double input);
  bool setAccY(double input);
  bool setFreqNumber(double input);
  bool setPosZ(double input);
  bool setVelZ(double input);
  bool setAccZ(double input);
  bool setAgeOfOperation(double input);

  // Selectors
  unsigned short getSatelliteAlmanacNumber();
  unsigned short getEpochYear();
  unsigned short getEpochMonth();
  unsigned short getEpochDay();
  unsigned short getEpochHour();
  unsigned short getEpochMin();
  double getEpochSec();

  double getSvClockBias();
  double getSvRelFreqBias();
  double getMessageFrameTime();
  double getPosX();
  double getVelX();
  double getAccX();
  double getSvHealth();
  double getPosY();
  double getVelY();
  double getAccY();
  double getFreqNumber();
  double getPosZ();
  double getVelZ();
  double getAccZ();
  double getAgeOfOperation();

 private:
  unsigned short satelliteAlmanacNumber;
  unsigned short epochYear;
  unsigned short epochMonth;
  unsigned short epochDay;
  unsigned short epochHour;
  unsigned short epochMin;
  double epochSec;
  double svClockBias;
  double svRelFreqBias;
  double messageFrameTime;
  double posX;
  double velX;
  double accX;
  double svHealth;
  double posY;
  double velY;
  double accY;
  double freqNumber;
  double posZ;
  double velZ;
  double accZ;
  double ageOfOperation;
};

//===================== GeostationaryEphemEpoch Class ======================

class GeostationaryEphemEpoch {
 public:
  GeostationaryEphemEpoch();   // Default Constructor
  ~GeostationaryEphemEpoch();  // Destructor

  // Initializers
  bool setSatelliteNumber(unsigned short input);
  bool setEpochYear(unsigned short input);
  bool setEpochMonth(unsigned short input);
  bool setEpochDay(unsigned short input);
  bool setEpochHour(unsigned short input);
  bool setEpochMin(unsigned short input);
  bool setEpochSec(double input);
  bool setSvClockBias(double input);
  bool setSvRelFreqBias(double input);
  bool setMessageFrameTime(double input);
  bool setPosX(double input);
  bool setVelX(double input);
  bool setAccX(double input);
  bool setSvHealth(double input);
  bool setPosY(double input);
  bool setVelY(double input);
  bool setAccY(double input);
  bool setAccurCode(double input);
  bool setPosZ(double input);
  bool setVelZ(double input);
  bool setAccZ(double input);
  bool setSpare(double input);

  // Selectors
  unsigned short getSatelliteNumber();
  unsigned short getEpochYear();
  unsigned short getEpochMonth();
  unsigned short getEpochDay();
  unsigned short getEpochHour();
  unsigned short getEpochMin();
  double getEpochSec();
  double getSvClockBias();
  double getSvRelFreqBias();
  double getMessageFrameTime();
  double getPosX();
  double getVelX();
  double getAccX();
  double getSvHealth();
  double getPosY();
  double getVelY();
  double getAccY();
  double getAccurCode();
  double getPosZ();
  double getVelZ();
  double getAccZ();
  double getSpare();

 private:
  unsigned short satelliteNumber;
  unsigned short epochYear;
  unsigned short epochMonth;
  unsigned short epochDay;
  unsigned short epochHour;
  unsigned short epochMin;
  double epochSec;
  double svClockBias;
  double svRelFreqBias;
  double messageFrameTime;
  double posX;
  double velX;
  double accX;
  double svHealth;
  double posY;
  double velY;
  double accY;
  double accurCode;
  double posZ;
  double velZ;
  double accZ;
  double spare;
};

//======================== HeaderRecord Class =============================

class HeaderRecord {
  friend ostream& operator<<(ostream& os, const HeaderRecord& input);
  friend istream& operator>>(istream& is, HeaderRecord& output);

 public:
  // Constructors
  HeaderRecord();
  HeaderRecord(string input);

  // Destructor
  ~HeaderRecord();

  // Initializers
  void SetHeaderRecord(string input);
  void SetFirst60(string input);
  void SetLabel(string input);

  // Selectors
  string GetFirst60();
  string GetLabel();

  // Operators
  HeaderRecord& operator=(const HeaderRecord& input);
  HeaderRecord* operator&(HeaderRecord input);

 private:
  string first60;
  string label;
};

//======================== RinexHeader Class =============================

class RinexHeader {
 public:
  RinexHeader();
  RinexHeader(list<HeaderRecord> inputImage);
  ~RinexHeader();

  void appendHeaderRecord(HeaderRecord addedRec);
  void insertHeaderRecBeforeLabel(string label, HeaderRecord addedRec);
  void insertHeaderRecAfterLabel(string label, HeaderRecord addedRec);
  void overwriteHeaderRecord(string label, string newFirst60);
  void deleteHeaderRecord(string label);
  void setHeaderImage(list<HeaderRecord> inputImage);

  HeaderRecord getHeaderRecord(string label);
  void writeHeaderImage(ofstream& outputStream);

 private:
  list<HeaderRecord> headerImage;
};

//======================== RinexFile Class ================================

class RinexFile  // this is a base class
{
 public:
  ofstream outputStream;
  ifstream inputStream;

  // Constructors
  RinexFile();
  RinexFile(string pathFilename, ios::openmode mode);

  // Destructor
  virtual ~RinexFile();

  // Initializers
  void setPathFilenameMode(string pathFilename, ios::openmode mode);
  bool setRinexHeaderImage(list<HeaderRecord> input);
  bool setFormatVersion(float input);
  bool setRinexFileType(string input);
  bool setSatSystem(string input);
  bool setRinexProgram(string input);
  bool setCreatedByAgency(string input);
  bool setDateFileCreated(string input);
  bool incrementNumberErrors(unsigned long n);
  bool incrementNumberWarnings(unsigned long n);
  bool incrementNumberLinesRead(unsigned long n);
  bool setCurrentEpoch(DateTime input);
  void appendToErrorMessages(string errMessage);
  void appendToWarningMessages(string warnMessage);
  void readFileTypeAndProgramName();

  // Selectors
  string getPathFilename();
  ios::openmode getFileMode();
  RinexHeader getRinexHeaderImage();
  float getFormatVersion();
  char getRinexFileType();
  char getSatSystem();
  string getRinexProgram();
  string getCreatedByAgency();
  string getDateFileCreated();
  unsigned long getNumberErrors();
  unsigned long getNumberWarnings();
  unsigned long getNumberLinesRead();
  DateTime getCurrentEpoch();
  string getErrorMessages();
  string getWarningMessages();

 protected:
  string pathFilename;
  ios::openmode fileMode;
  RinexHeader rinexHeaderImage;
  float formatVersion;
  char rinexFileType;
  char satSystem;
  string rinexProgram;
  string createdByAgency;
  string dateFileCreated;
  unsigned long numberErrors;
  unsigned long numberWarnings;
  unsigned long numberLinesRead;
  DateTime currentEpoch;

  ostringstream tempStream;
  ostringstream errorMessages;
  ostringstream warningMessages;

  bool validFirstLine(string& recordReadIn);
  bool blankString(string inputStr);
  bool alphasInString(string inputStr);
  void makeRecordLength80(string& inputRec);
  void truncateHeaderRec(string& inputRec);
  bool getDouble(string input, double& output);
  bool getLong(string input, long& output);
  bool validYMDHMS(
      long year, long month, long day, long hour, long minute, double second,
      string& warningString);
};

//======================== RinexObsFile Class ===============================

class RinexObsFile : public RinexFile {
 public:
  // Constructors
  RinexObsFile();
  RinexObsFile(string pathFilename, ios::openmode mode);

  // Destructor
  ~RinexObsFile();

  // Initializers
  bool setMarkerName(string input);
  bool setMarkerNumber(string input);
  bool setObserverName(string input);
  bool setObserverAgency(string input);
  bool setReceiverNumber(string input);
  bool setReceiverType(string input);
  bool setReceiverFirmwareVersion(string input);
  bool setAntennaNumber(string input);
  bool setAntennaType(string input);

  bool setApproxX(double input);
  bool setApproxY(double input);
  bool setApproxZ(double input);
  bool setAntennaDeltaH(double input);
  bool setAntennaDeltaE(double input);
  bool setAntennaDeltaN(double input);

  bool setDefWaveLenFactorL1(unsigned short input);
  bool setDefWaveLenFactorL2(unsigned short input);
  bool setNumWaveLenPRN(unsigned short input);
  bool setNumWaveLenRecords(unsigned short input);

  bool setAllWaveLenRecordsElement(OneWaveLenRec input, int i);
  bool setNumObsTypes(unsigned short input);
  bool setObsTypeListElement(enum OBSTYPE input, int i);
  bool setObsInterval(float input);
  bool setFirstObs(YMDHMS input);
  bool setFirstObsTimeSystem(string input);
  bool setLastObs(YMDHMS input);
  bool setLastObsTimeSystem(string input);
  bool setNumberLeapSec(unsigned short input);
  bool setRcvrClockApplied(unsigned short input);
  bool setNumberOfSat(unsigned short input);
  bool setSatObsTypeListElement(ObsCountForPRN input, int i);
  bool setNextSat(unsigned short input);

  void incrementNumberObsEpochs(unsigned int n);
  unsigned short readHeader();
  unsigned short readEpoch(ObsEpoch& epoch);

  // Selectors
  string getMarkerName();
  string getMarkerNumber();
  string getObserverName();
  string getObserverAgency();
  string getReceiverNumber();
  string getReceiverType();
  string getReceiverFirmwareVersion();
  string getAntennaNumber();
  string getAntennaType();

  double getApproxX();
  double getApproxY();
  double getApproxZ();
  double getAntennaDeltaH();
  double getAntennaDeltaE();
  double getAntennaDeltaN();

  unsigned short getDefWaveLenFactorL1();
  unsigned short getDefWaveLenFactorL2();
  unsigned short getNumWaveLenPRN();
  unsigned short getNumWaveLenRecords();

  OneWaveLenRec getAllWaveLenRecordsElement(int i);
  unsigned short getNumObsTypes();
  enum OBSTYPE getObsTypeListElement(int i);
  float getObsInterval();
  YMDHMS getFirstObs();
  string getFirstObsTimeSystem();
  YMDHMS getLastObs();
  string getLastObsTimeSystem();
  unsigned short getNumberLeapSec();
  unsigned short getRcvrClockApplied();
  unsigned short getNumberOfSat();
  ObsCountForPRN getSatObsTypeListElement(int i);
  unsigned short getNextSat();

  void writeHeaderImage(ofstream& outputStream);
  void writeEpoch(ofstream& outputOBS, ObsEpoch& outputEpoch);

  unsigned int getNumberObsEpochs();
  static unsigned int getObsFilesCount();

 private:
  RecStruct headerRecs[MAXOBSHEADERRECTYPES];

  string markerName;                  // Name of antenna marker.
                                      //   - MARKER NAME
  string markerNumber;                // Number of antenna marker.
                                      //   - MARKER NUMBER
  string observerName;                // Name of observer.
                                      //   - OBSERVER / AGENCY
  string observerAgency;              // Name of observing agency.
                                      //   - OBSERVER / AGENCY
  string receiverNumber;              // Receiver identification number.
                                      //   - REC # / TYPE / VERS
  string receiverType;                // Receiver type code
                                      //   - REC # / TYPE / VERS
  string receiverFirmwareVersion;     // Receiver firmware version.
                                      //   - REC # / TYPE / VERS
  string antennaNumber;               // Antenna number.
                                      //   - ANT # / TYPE
  string antennaType;                 // Antenna type.
                                      //   - ANT # / TYPE
  double approxX;                     // Approximate position, X coord.
                                      //   - APPROX POSITION XYZ
  double approxY;                     // Approximate position, Y coord.
                                      //   - APPROX POSITION XYZ
  double approxZ;                     // Approximate position, Z coord.
                                      //   - APPROX POSITION XYZ
  double antennaDeltaH;               // Height of antenna above marker
                                      //   - ANTENNA: DELTA H/E/N
  double antennaDeltaE;               // East Eccen. of ant w.r.t. mark,
                                      //   - ANTENNA: DELTA H/E/N
  double antennaDeltaN;               // North Eccen. of ant w.r.t. mark,
                                      //   - ANTENNA: DELTA H/E/N
  unsigned short defWaveLenFactorL1;  // Default Wavelength factor of L1
                                      //   - WAVELENGTH FACT L1/2
  unsigned short defWaveLenFactorL2;  // Default Wavelength factor of L2
                                      //   - WAVELENGTH FACT L1/2
  unsigned short numWaveLenPRN;       // # PRNs in all Wavelength records
                                      //   - WAVELENGTH FACT L1/2
  unsigned short numWaveLenRecords;   // # Wavelength factor records
                                      //   - WAVELENGTH FACT L1/2

  // Wavelength factor records.
  OneWaveLenRec allWaveLenRecords[MAXPRNID];

  unsigned short numObsTypes;             // # observation types
                                          // - # / TYPES  OF OBSERV
  enum OBSTYPE obsTypeList[MAXOBSTYPES];  // List of obs types
                                          // - # / TYPES  OF OBSERV
  float obsInterval;                      // Obs time interval sec.
                                          //  - INTERVAL
  YMDHMS firstObs;                        // Date & time of first obs
                                          //  - TIME OF FIRST OBS
  string firstObsTimeSystem;              // system for mixed GPS+GLO
  YMDHMS lastObs;                         // Date & time of last obs
                                          //  - TIME OF LAST OBS
  string lastObsTimeSystem;               // system for mixed GPS+GLO

  unsigned short numberLeapSec;
  unsigned short rcvrClockApplied;

  unsigned short numberOfSat;  // # satellites in data.
                               //  - # OF SATELLITES

  ObsCountForPRN satObsTypeList[MAXPRNID];

  unsigned short nextSat;  // index for satObsTypeList

  unsigned int numberObsEpochs;
  static unsigned int numberObsFiles;  // # Obs Files instantiated

  void initializeData();
  bool validHeaderRecord(string inputRec);
  bool validEventFlagRecord(string inputRec);
  bool validObservationsRecord(string inputRec);
};

//======================== RinexNavFile Class =============================

class RinexNavFile : public RinexFile {
 public:
  // Constructors
  RinexNavFile();
  RinexNavFile(string pathFilename, ios::openmode mode);

  // Destructor
  ~RinexNavFile();

  // Initializers
  bool setA0(double input);
  bool setA1(double input);
  bool setA2(double input);
  bool setA3(double input);
  bool setB0(double input);
  bool setB1(double input);
  bool setB2(double input);
  bool setB3(double input);
  bool setUtcA0(double input);
  bool setUtcA1(double input);
  bool setUtcRefTime(long input);
  bool setUtcRefWeek(long input);
  bool setLeapSec(unsigned short input);

  void incrementNumberPRNBlocks(unsigned int n);
  unsigned short readHeader();
  unsigned short readPRNBlock(PRNBlock& prnBlock);

  // Selectors
  double getA0();
  double getA1();
  double getA2();
  double getA3();
  double getB0();
  double getB1();
  double getB2();
  double getB3();
  double getUtcA0();
  double getUtcA1();
  long getUtcRefTime();
  long getUtcRefWeek();
  unsigned short getLeapSec();
  unsigned int getNumberPRNBlocks();
  static unsigned int getNavFilesCount();
  void writeHeaderImage(ofstream& outputStream);
  void writePRNBlock(ofstream& outputStream, PRNBlock& outputPRNBlock);

 private:
  RecStruct headerRecs[MAXNAVHEADERRECTYPES];
  double a0;  // Ionosphere parameters:
  double a1;  // A0 to A3 of almanac
  double a2;  // (page 18 of subframe 4)
  double a3;
  double b0;  // Ionosphere parameters:
  double b1;  // B0 to B3 of almanac
  double b2;
  double b3;
  double utcA0;            // parameters to compute UTC time:
  double utcA1;            // A0,A1 terms of polynomial
  long utcRefTime;         // T : ref.time for UTC data
  long utcRefWeek;         // W : UTC ref. week number
  unsigned short leapSec;  // Delta time due to leap seconds
  unsigned int numberPRNBlocks;
  static unsigned int numberNavFiles;  // # Nav Files instantiated

  void initializeData();
  bool validHeaderRecord(string inputRec);
};

//======================== GlonassNavFile Class =============================

class GlonassNavFile : public RinexFile {
 public:
  // Constructors
  GlonassNavFile();
  GlonassNavFile(string pathFilename, ios::openmode mode);

  // Destructor
  ~GlonassNavFile();

  // Initializers
  bool setRefTimeYear(unsigned short input);
  bool setRefTimeMonth(unsigned short input);
  bool setRefTimeDay(unsigned short input);
  bool setTimeScaleCorr(double input);
  bool setLeapSec(unsigned short input);
  void incrementNumberEphemEpochs(unsigned int n);
  unsigned short readHeader();
  unsigned short readEphemEpoch(GlonassEphemEpoch& navEpoch);

  // Selectors
  unsigned short getRefTimeYear();
  unsigned short getRefTimeMonth();
  unsigned short getRefTimeDay();
  double getTimeScaleCorr();
  unsigned short getLeapSec();
  unsigned int getNumberEpochs();
  static unsigned int getFilesCount();
  void writeHeaderImage(ofstream& outputStream);
  void writeEphemEpoch(ofstream& outputStream, GlonassEphemEpoch& navEpoch);

 private:
  RecStruct headerRecs[MAXNAVHEADERRECTYPES];
  unsigned short refTimeYear;       // Time of reference for system
  unsigned short refTimeMonth;      // time correction
  unsigned short refTimeDay;        // (year, month, day).
  double timeScaleCorr;             // Correct GLONASS system time to
                                    // UTC(SU)  (-TauC).
  unsigned short leapSec;           // Leap seconds since 6-Jan-1980
  unsigned int numberEpochs;        // # epochs in the Nav File
  static unsigned int numberFiles;  // # GLONASS Nav Files instantiated

  void initializeData();
  bool validHeaderRecord(string inputRec);
};

//===================== GeostationaryNavFile Class ===========================

class GeostationaryNavFile : public RinexFile {
 public:
  // Constructors
  GeostationaryNavFile();
  GeostationaryNavFile(string pathFilename, ios::openmode mode);

  // Destructor
  ~GeostationaryNavFile();

  // Initializers
  bool setRefTimeYear(unsigned short input);
  bool setRefTimeMonth(unsigned short input);
  bool setRefTimeDay(unsigned short input);
  bool setCorrToUTC(double input);
  bool setLeapSec(unsigned short input);
  void incrementNumberEphemEpochs(unsigned int n);
  unsigned short readHeader();
  unsigned short readEphemEpoch(GeostationaryEphemEpoch& navEpoch);

  // Selectors
  unsigned short getRefTimeYear();
  unsigned short getRefTimeMonth();
  unsigned short getRefTimeDay();
  double getCorrToUTC();
  unsigned short getLeapSec();
  unsigned int getNumberEpochs();
  static unsigned int getFilesCount();
  void writeHeaderImage(ofstream& outputStream);
  void writeEphemEpoch(
      ofstream& outputStream, GeostationaryEphemEpoch& navEpoch);

 private:
  RecStruct headerRecs[MAXNAVHEADERRECTYPES];
  unsigned short refTimeYear;       // Time of reference for system
  unsigned short refTimeMonth;      // time correction
  unsigned short refTimeDay;        // (year, month, day).
  double corrToUTC;                 // Correct GEO system time to UTC
  unsigned short leapSec;           // Leap seconds since 6-Jan-1980
  unsigned int numberEpochs;        // # epochs in the Nav File
  static unsigned int numberFiles;  // # GLONASS Nav Files instantiated

  void initializeData();
  bool validHeaderRecord(string inputRec);
};

//======================== RinexMetFile Class =============================

class RinexMetFile : public RinexFile {
 public:
  // Constructors
  RinexMetFile();
  RinexMetFile(string pathFilename, ios::openmode mode);

  // Destructor
  ~RinexMetFile();

  // Initializers
  bool setMarkerName(string input);
  bool setMarkerNumber(string input);
  bool setNumMetTypes(unsigned short input);
  bool setObsTypeListElement(enum METTYPE input, int i);
  bool setSensorModAccurElement(SensorInfo input, int i);
  bool setSensorXYZhElement(SensorPosition input, int i);

  void incrementNumberMetEpochs(unsigned int n);
  unsigned short readHeader();
  void initEpochInfo(MetEpoch& epoch);
  unsigned short readEpoch(MetEpoch& epoch);

  // Selectors
  string getMarkerName();
  string getMarkerNumber();
  unsigned short getNumMetTypes();
  enum METTYPE getObsTypeListElement(int i);
  SensorInfo getSensorModAccurElement(int i);
  SensorPosition getSensorXYZhElement(int i);
  unsigned int getNumberMetEpochs();
  static unsigned int getMetFilesCount();
  void writeHeaderImage(ofstream& outputStream);
  void writeEpoch(ofstream& outputStream, MetEpoch& outputEpoch);

 private:
  RecStruct headerRecs[MAXMETHEADERRECTYPES];

  string markerName;    // Name of antenna marker.
                        //   - MARKER NAME
  string markerNumber;  // Number of antenna marker.
                        //   - MARKER NUMBER

  unsigned short numMetTypes;             // # observation types
                                          //   - # / TYPES  OF OBSERV
  enum METTYPE obsTypeList[MAXMETTYPES];  // List of obs types
                                          //   - # / TYPES  OF OBSERV

  SensorInfo sensorModAccur[MAXMETTYPES];
  SensorPosition sensorXYZh[MAXMETTYPES];

  unsigned int numberMetEpochs;
  static unsigned int numberMetFiles;  // # Met Files instantiated

  void initializeData();
  bool validHeaderRecord(string inputRec);
};

//======================== ClockDataFile Class =============================

class ClockDataFile : public RinexFile {
 public:
  // Constructors
  ClockDataFile();
  ClockDataFile(string pathFilename, ios::openmode mode);

  // Destructor
  ~ClockDataFile();

  // Initializers
  bool setLeapSec(unsigned short input);
  bool setNumberClkTypes(unsigned short input);
  bool setClkTypeListElement(enum CLKTYPE input, int i);
  bool setStationName(string input);
  bool setStationNumber(string input);
  bool setStationClkRef(string input);
  bool setACDesignator(string input);
  bool setAnalysisCenterName(string input);

  bool setNumberAnalysisClockRef(unsigned short input);
  bool setAnalysisStartEpoch(DateTime input);
  bool setAnalysisStopEpoch(DateTime input);
  bool setNumberSolnSta(unsigned short input);
  bool setTerrRefFrameOrSinex(string input);
  bool setNumberSolnSatellites(unsigned short input);

  bool setClkRefListElement(AnalysisClkRefData input, int i);
  bool setSolnStaListElement(SolnStaNameData input, int i);
  bool setPrnListElement(string input, int i);

  void incrementNumberClkEpochs(unsigned int n);
  unsigned short readHeader();
  void initEpochInfo(ClkEpoch& epoch);
  unsigned short readEpoch(ClkEpoch& epoch);

  // Selectors
  unsigned short getLeapSec();
  unsigned short getNumberClkTypes();
  enum CLKTYPE getClkTypeListElement(int i);
  string getStationName();
  string getStationNumber();
  string getStationClkRef();
  string getACDesignator();
  string getAnalysisCenterName();

  unsigned short getNumberAnalysisClockRef();
  DateTime getAnalysisStartEpoch();
  DateTime getAnalysisStopEpoch();
  unsigned short getNumberSolnSta();
  string getTerrRefFrameOrSinex();
  unsigned short getNumberSolnSatellites();

  AnalysisClkRefData getClkRefListElement(int i);
  SolnStaNameData getSolnStaListElement(int i);
  string getPrnListElement(int i);

  unsigned short getNumberClkEpochs();
  static unsigned short getClkFilesCount();
  void writeHeaderImage(ofstream& outputStream);
  void writeEpoch(ofstream& outputStream, ClkEpoch& outputEpoch);

 private:
  RecStruct headerRecs[MAXCLKHEADERRECTYPES];

  unsigned short leapSec;
  unsigned short numberClkTypes;          // # clock data types
  enum CLKTYPE clkTypeList[MAXCLKTYPES];  // List of clock data types
  string stationName;
  string stationNumber;
  string stationClkRef;
  string ACDesignator;
  string analysisCenterName;
  unsigned short numberAnalysisClkRef;
  DateTime analysisStartEpoch;
  DateTime analysisStopEpoch;

  AnalysisClkRefData* clkRefList;  // allocated in readHeader()
  SolnStaNameData* solnStaList;    // allocated in readHeader()

  unsigned short numberSolnSta;
  string terrRefFrameOrSinex;
  unsigned short numberSolnSatellites;
  string* prnList;  // allocated in readHeader()

  unsigned short numberClkEpochs;
  static unsigned short numberClkFiles;
  void initializeData();
  bool validHeaderRecord(string inputRec);
};

//=================== RequiredRecordMissingException ========================

class RequiredRecordMissingException {
 public:
  RequiredRecordMissingException(const string& errMsg);
  string ErrorMessage;
  string getMessage();
};

//======================== RinexFileException Class =======================

class RinexFileException {
 public:
  RinexFileException(const string& errMsg);
  string ErrorMessage;
  string getMessage();
};

//======================== RinexReadingException Class =======================

class RinexReadingException {
 public:
  RinexReadingException(const string& errMsg);
  string ErrorMessage;
  string getMessage();
};

}  // namespace NGSrinex

#endif
