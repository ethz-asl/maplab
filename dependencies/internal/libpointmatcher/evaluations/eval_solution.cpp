// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"
#include "pointmatcher/Timer.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <map>
#include <time.h>

#include <ncurses.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>


using namespace std;
using namespace PointMatcherSupport;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace pt = boost::posix_time;

typedef PointMatcher<float> PM;
typedef PointMatcherIO<float> PMIO;
typedef PM::TransformationParameters TP;

struct DataSetInfo
{
	string name;
	bool downloaded;
	string path;
	DataSetInfo(){};
	DataSetInfo(string name, bool downloaded):
		name(name),
		downloaded(downloaded)
	{}
};

struct Config
{
	// Default values
	string path_config;
	string path_download;
	string path_result;
	string path_server_validation;
	string path_server_protocols;
	map<string, DataSetInfo> dataSetStatus;
	
	Config()
	{
		path_config = string(getenv("HOME")) + "/.lpm/eval_solution.conf";
		path_download = string(getenv("HOME")) + "/.lpm/download/";
		path_result = "./";
		path_server_validation = "robotics.ethz.ch/~asl-datasets/evaluations/validation";
		path_server_protocols = "robotics.ethz.ch/~asl-datasets/evaluations/protocols";
		DataSetInfo info;
		info = DataSetInfo("apartment", false);
		info.path = "robotics.ethz.ch/~asl-datasets/apartment_03-Dec-2011-18_13_33/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("eth", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/ETH_hauptgebaude_23-Aug-2011-18_43_49/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("plain", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/plain_01-Sep-2011-16_39_18/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("stairs", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/stairs_26-Aug-2011-14_26_14/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("gazebo", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/gazebo_winter_18-Jan-2012-16_10_04/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
		info = DataSetInfo("wood", false);
		info.path = "http://robotics.ethz.ch/~asl-datasets/wood_summer_25-Aug-2011-13_00_30/csv_local/local_frame.zip";
		dataSetStatus[info.name] = info;
	}

};

class EvaluationModule
{
public:
	EvaluationModule();
	int coreId;
	string tmp_file_name;
	double result_time;
	void evaluateSolution(const string &tmp_file_name, const string &yaml_config, const int &coreId, PMIO::FileInfoVector::const_iterator it_eval, PMIO::FileInfoVector::const_iterator it_end);
	
};

po::options_description setupOptions(const string & name);
string outputStatus(map<string, DataSetInfo> dataSetStatus);
string enterValidPath(string message);
void setConfig(Config& config);
void saveConfig(Config& config);
void loadConfig(Config& config);
void downloadDataSets(Config& config, po::variables_map &vm);
void validateFileInfo(const PMIO::FileInfo &fileInfo);
void displayLoadingBar(const int &coreId, const int &i, const int &total, const int &nbFailures, const double sec, const double total_time);

int main(int argc, char *argv[])
{
	srand ( time(NULL) );

	// Option parsing
	po::options_description desc = setupOptions(argv[0]);
	po::positional_options_description p;
	p.add("icp-config", -1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
	          options(desc).positional(p).run(), vm);
	po::notify(vm);

	// Evaluation configuration
	Config config;

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	// Check for config
	fs::path c_path(config.path_config);
	if(fs::exists(c_path) == false)
	{
		fs::create_directories(c_path.parent_path());
		cout << ">> no configuration found. Using default values." << endl;
		
	}
	else
	{
		loadConfig(config);
	}

	if (vm.count("config")) 
	{
		setConfig(config);
		saveConfig(config);
		return 0;
	}
	
	if (vm.count("download")) 
	{
		downloadDataSets(config, vm);
		saveConfig(config);
		return 0;
	}

	if (vm.count("icp-config") == false)
	{
		cerr << "You must provide a YAMl file to evaluate it." << endl;
		return 1;
	}
	
	const string yaml_config = vm["icp-config"].as<string>();
	{
		ifstream cfgIfs(yaml_config.c_str());
		if (!cfgIfs.good())
		{
			cerr << "Cannot open YAML file name: it must exist and be readable." << endl;
			return 2;
		}
	}

	initscr(); // ncurse screen

	// Starting evaluation
	for(BOOST_AUTO(it, config.dataSetStatus.begin()); it != config.dataSetStatus.end(); ++it)
	{
		if(vm.count("all") || vm.count(it->second.name))
		{
			if(it->second.downloaded == false)
			{
				endwin();			/* End curses mode		  */
				cerr << ">> Please download data set first." << endl
				     << ">> You can use the option --download -A to download them all." << endl;
					 return 1;
			}
			
			const string protocol_name = config.path_download + "protocols/" + it->second.name + "_protocol.csv";
			const string data_directory = config.path_download + it->second.name + "/";
			
			if(!fs::exists(fs::path(protocol_name)))
			{
				endwin();			/* End curses mode		  */
				cerr << ">> Missing protocol file: " << protocol_name << endl
				     << ">> You can use the option --download to download it back." << endl;
					 return 1;
			}

			const PMIO::FileInfoVector eval_list(protocol_name, data_directory, "");

			// Ensure that all required columns are there
			validateFileInfo(eval_list[0]);
			move(0,0);
			clrtoeol();
			mvprintw(0, 0, " <<< Evaluating %s >>>", it->second.name.c_str());
			//cout << endl << "<<< Evaluating " << it->second.name << " >>>" << endl;
		
			// Spawn threads
			const int maxNbCore = 16;
			int nbCore = 1;
			if(vm["threads"].as<int>() > 1 || vm["threads"].as<int>() < maxNbCore)
			{
				nbCore = vm["threads"].as<int>();
			}
			
			// List of thread
			boost::thread a_threads[maxNbCore];
			std::vector<EvaluationModule> v_evalModules;
			
			const int nbPerThread = eval_list.size()/nbCore;
			PMIO::FileInfoVector::const_iterator it_start = eval_list.begin();
			for (int j=0; j<nbCore; ++j)
			{
				v_evalModules.push_back(EvaluationModule());
				v_evalModules[j].coreId = j;
				stringstream name;
				name << ".tmp_core" << j << "_" << rand() << ".csv";
				v_evalModules[j].tmp_file_name = name.str();
				// Start evaluation for every line
				if(j == nbCore-1)
				{
					// last core receive the reste
					a_threads[j] = boost::thread(&EvaluationModule::evaluateSolution, &v_evalModules[j], name.str(), yaml_config, j, it_start, eval_list.end());
					//evalCore.evaluateSolution(yaml_config, j, it_start, eval_list.end());
				}
				else
				{
					a_threads[j] = boost::thread(&EvaluationModule::evaluateSolution, &v_evalModules[j], name.str(), yaml_config, j, it_start, it_start + nbPerThread);
					//evalCore.evaluateSolution(yaml_config, j, it_start, it_start + nbPerThread);
					it_start += nbPerThread;
				}
				cout << endl;
			}

			// Wait for the results
			for (int k=0; k<nbCore; ++k)
			{
				if(a_threads[k].joinable())
					a_threads[k].join();
			}

			// Write the results to a file
			stringstream ss_path_time;
			pt::time_facet *facet = new pt::time_facet("%d-%b-%Y-%H_%M_%S");
			ss_path_time.imbue(locale(ss_path_time.getloc(), facet));

			ss_path_time << config.path_result << it->second.name << "_";
			ss_path_time << pt::second_clock::local_time() << ".csv";
			
			move(nbCore+3,0);
			clrtoeol();
			mvprintw(nbCore+3, 0, "Last result written to: %s", ss_path_time.str().c_str());
			std::ofstream fout(ss_path_time.str().c_str());
			if (!fout.good())
			{
				cerr << "Warning, cannot open result file " << ss_path_time << ", results were not saved" << endl;
				continue;
			}

			// dump header
			fout << "time";
			for(int r=0; r<4;++r)
				for(int c=0; c<4;++c)
					fout << ", T" << r << c;
			
			fout << "\n";
			
			// dump results
			// for all threads
			for(unsigned i=0; i<v_evalModules.size(); ++i)
			{
				const string tmp_file_name = v_evalModules[i].tmp_file_name;
				ifstream tmp_file(tmp_file_name.c_str());
				if(tmp_file.is_open())
				{
					fout << tmp_file.rdbuf();
					tmp_file.close();
					fs::remove(fs::path(tmp_file_name));
				}
				else
				{
					endwin();/* End curses mode		  */
					cerr << "Cannot find tmp file named "<< tmp_file_name << endl;
				}
			}

			fout.close();
		}
	}
	endwin();			/* End curses mode		  */
	
	return 0;
}


po::options_description setupOptions(const string & name)
{
	po::options_description desc("Allowed options");
	desc.add_options()
	    ("help,h", "Print this message")
		("icp-config", po::value<string>(), "YAML configuration file")
		("config,C", "Interactive configuration")
		("download,D", "Download selected data sets from the web")
		("evaluate,E", "Evaluate a solution over selected data sets")
		("threads,j", po::value<int>()->default_value(1), "Number of threads to use. Max 16.")
		("apartment,a", "Apply action only on the data set Apartment")
		("eth,e", "Apply action only on the data set ETH Hauptgebaude")
		("plain,p", "Apply action only on the data set Mountain Plain")
		("stairs,s", "Apply action only on the data set Stairs")
		("gazebo,g", "Apply action only on the data set Gazebo Winter")
		("wood,w", "Apply action only on the data set Wood Summer")
		("all,A", "Apply action for all data sets")
		;

	return desc;
}

string outputStatus(map<string, DataSetInfo> dataSetStatus)
{
	stringstream ss;

	for(BOOST_AUTO(it, dataSetStatus.begin()); it != dataSetStatus.end(); ++it)
	{
		string paddedName = it->second.name + ":";
		while (paddedName.length() < 12) paddedName += " ";
		ss << "\t" << paddedName;
		if(it->second.downloaded)
		{
			ss << "downloaded.";
		}
		else
		{
			ss << "not on your system.";
		}
		
		ss << endl;
	}

	return ss.str();
}


string enterValidPath(string message)
{
	bool validPath = false;
	string path_name;
	while(!validPath)
	{
		cout << message;
		getline(cin, path_name);
		validPath = fs::is_directory(fs::path(path_name));
		if(validPath == false)
			cout << ">> Not a valid path" << endl;
	}

	return path_name;
}


void setConfig(Config& config)
{
	string answer = "0";
	cout << "Current configuration:" << endl
		 << "\t Download path: " << config.path_download << endl
		 << "\t Result path: " << config.path_result << endl;

	cout << "Data set status:" << endl;
	cout << outputStatus(config.dataSetStatus);

	while(!(answer == "" || answer == "y" ||answer == "Y" || answer == "n" || answer == "N")) 
	{
		cout << endl << "Do you want to change something? [y/N]: ";
		getline(cin, answer);

		//if(answer == "" || answer == "n" ||answer == "N" )
		//	return 1;
		if(answer == "y" ||answer == "Y" )
		{
			config.path_download = enterValidPath("Enter data set path or where they will be downloaded: ");
			cout << endl;
			config.path_result = enterValidPath("Enter the result path (where the result of the test will be saved): ");

		}
	}
}

void saveConfig(Config& config)
{
	YAML::Emitter emitter;
		
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "path_download";
		emitter << YAML::Value << config.path_download;
		emitter << YAML::Key << "path_result";
		emitter << YAML::Value << config.path_result;

		for(BOOST_AUTO(it, config.dataSetStatus.begin()); it != config.dataSetStatus.end(); ++it)
		{
			emitter << YAML::Key << it->second.name;
			emitter << YAML::Value;
			emitter << YAML::BeginMap;
				emitter << YAML::Key << "downloaded";
				emitter << YAML::Value << it->second.downloaded;
			emitter << YAML::EndMap;
		}
		emitter << YAML::EndMap;

		std::ofstream fout(config.path_config.c_str());
		if (!fout.good())
		{
			cerr << "Warning, cannot open config file " << config.path_config << ", content was not saved" << endl;
			return;
		}
		fout << emitter.c_str();
		fout.close();
}

void loadConfig(Config& config)
{
	ifstream f_config(config.path_config.c_str());
	if (!f_config.good())
	{
		cerr << "Warning, cannot open config file " << config.path_config << ", content was not loaded" << endl;
		return;
	}
	YAML::Parser parser(f_config);

	YAML::Node doc;
	while(parser.GetNextDocument(doc)) 
	{
		doc["path_download"] >> config.path_download;
		doc["path_result"] >> config.path_result;
		for(BOOST_AUTO(it, config.dataSetStatus.begin()); it != config.dataSetStatus.end(); ++it)
		{
			string dataSetName = it->second.name;
			const YAML::Node &node = doc[dataSetName];
			node["downloaded"] >> it->second.downloaded;
		}
	}
}

void downloadDataSets(Config& config, po::variables_map &vm)
{
	// Ensure that validation and protocol folders are there
	fs::path extra_path(config.path_download+"/validation/");
	if(!fs::is_directory(extra_path))
		fs::create_directories(extra_path);
	extra_path = fs::path(config.path_download+"/protocols/");
	if(!fs::is_directory(extra_path))
		fs::create_directories(extra_path);

	for(BOOST_AUTO(it, config.dataSetStatus.begin()); it != config.dataSetStatus.end(); ++it)
	{
		if(vm.count("all") || vm.count(it->second.name))
		{
			cout << ">> Fetching files for: " << it->second.name << "..." << endl << endl;
			fs::path d_path(config.path_download+it->second.name);
			if(!fs::is_directory(d_path))
				fs::create_directories(d_path);

			string cmd;
			int sysRes;
			#define CHECK_RES if (sysRes != 0) { cerr << "Warning, system command \"" << cmd << "\" failed with result code " << sysRes << endl; }
			
			// Dowload validation
			cout << ">> Downloading validation file ..." << endl;
			cmd = "wget -P " + config.path_download + "/validation/ " + config.path_server_validation + "/" + it->second.name + "_validation.csv";
			sysRes = system(cmd.c_str());
			CHECK_RES

			// Dowload protocol
			cout << ">> Downloading protocol file ..." << endl;
			cmd = "wget -P " + config.path_download + "/protocols/ " + config.path_server_protocols + "/" + it->second.name + "_protocol.csv";
			sysRes = system(cmd.c_str());
			CHECK_RES

			// Download data set
			cout << ">> Downloading data set files ..." << endl;
			cmd = "wget -P " + d_path.string() + " " + it->second.path;
			sysRes = system(cmd.c_str());
			CHECK_RES
			
			cout << ">> Unzipping dataset..." << endl;
			cmd = "unzip -q " + d_path.string() + "/local_frame.zip -d " + d_path.string() + "/";
			sysRes = system(cmd.c_str());
			CHECK_RES
			
			cmd = "rm " + d_path.string() + "/local_frame.zip";
			sysRes = system(cmd.c_str());
			CHECK_RES

			it->second.downloaded = true;
		}
	}
}

void validateFileInfo(const PMIO::FileInfo &fileInfo)
{
	if(fileInfo.initialTransformation.rows() == 0)
	{
		cout << "Missing columns representing initial transformation \"iTxy\"" << endl;
		abort();
	}

	if(fileInfo.readingFileName == "")
	{
		cout << "Missing column named \"reading\"" << endl;
		abort();
	}

	if(fileInfo.referenceFileName == "")
	{
		cout << "Missing column named \"reference\"" << endl;
		abort();
	}
	
}

boost::mutex m_display;
void displayLoadingBar(const int &coreId, const int &i, const int &total, const int &nbFailures, const double sec, const double total_time)
{
	const double average_time = total_time/double(i+1);
	int time = average_time*double(total-i);
	const int h=time/3600;
	time=time%3600;
	const int m=time/60;
	time=time%60;
	const int s=time;


	m_display.lock();
	//ncurse output
	move(coreId+1,0);
	clrtoeol();
	mvprintw(coreId+1, 10, " Core %2d: %5d/%5d  failed: %2d last dur: %2.3f sec, avr: %2.3f sec, eta: %3dh%02dm%02d",coreId,i+1, total, nbFailures, sec, average_time, h, m, s);
	refresh();			/* Print it on to the real screen */
	m_display.unlock();

	//cout << "\r  Core " << coreId << ": " << i+1 << "/" << total << "     last dur: " <<  sec << " sec, avr: " << average_time << " sec, eta: " << h << "h" << m % 60 << "m" << eta % 60 << "s             "; 	
}





// ----------------------------------------------------
// EvaluationModule
// ----------------------------------------------------
EvaluationModule::EvaluationModule():
	coreId(0)
{
}

void EvaluationModule::evaluateSolution(const string &tmp_file_name, const string &yaml_config, const int &coreId, PMIO::FileInfoVector::const_iterator it_eval, PMIO::FileInfoVector::const_iterator it_end)
{
	PM::DataPoints refCloud, readCloud;
	string last_read_name = "";
	string last_ref_name = "";
	const int count = std::distance(it_eval, it_end);
	int current_line = 0;
	timer t_eval_list;

	std::ofstream fout(tmp_file_name.c_str());
	if (!fout.good())
	{
		cerr << "Warning, cannot open temporary result file " << tmp_file_name << ", evaluation was skipped" << endl;
		return;
	}

	for( ; it_eval < it_end; ++it_eval)
	{
		timer t_singleTest;

		// Load point clouds
		if(last_read_name != it_eval->readingFileName)
		{
			readCloud = PM::DataPoints::load(it_eval->readingFileName);
			last_read_name = it_eval->readingFileName;
		}

		if(last_ref_name != it_eval->referenceFileName)
		{
			refCloud = PM::DataPoints::load(it_eval->referenceFileName);
			last_ref_name = it_eval->referenceFileName;
		}

		// Build ICP based on config file
		PM::ICP icp;
		ifstream ifs(yaml_config.c_str());
		icp.loadFromYaml(ifs);

		const TP Tinit = it_eval->initialTransformation;

		timer t_icp;

		int nbFailures = 0;
		TP Tresult = TP::Identity(4,4);
		// Apply ICP
		try
		{
			Tresult = icp(readCloud, refCloud, Tinit);
		}
		catch (PM::ConvergenceError error)
		{
			nbFailures ++;
		}

		fout << t_icp.elapsed();

		for(int r=0; r<4;++r)
			for(int c=0; c<4;++c)
				fout << ", " << Tresult(r,c);
		
		fout << "\n";

		// Output eta to console
		displayLoadingBar(coreId, current_line, count, nbFailures, t_singleTest.elapsed(), t_eval_list.elapsed());
		current_line ++;
	}

	fout.close();
}
