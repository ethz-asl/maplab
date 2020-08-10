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
#include "pointmatcher/Bibliography.h"

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcherSupport::Parametrizable::ParametersDoc ParametersDoc;
typedef PointMatcher<float> PM;

void printBibliographyHeader(const CurrentBibliography::Mode mode)
{
	switch (mode)
	{
		case CurrentBibliography::NORMAL:
		cout << "* Bibliography *" << endl << endl;
		break;
		
		case CurrentBibliography::ROSWIKI:
		cout << "=== Bibliography ===" << endl << endl;
		break;
		
		default:
		break;
	}
}

void dumpWiki(const ParametersDoc& paramsDoc)
{
	cout << endl;
	if (!paramsDoc.empty())
		for (BOOST_AUTO(it, paramsDoc.begin()); it != paramsDoc.end(); ++it)
		{
			cout << "`" << it->name << "` (default: `" << it->defaultValue << "`";
			if (!it->minValue.empty())
				cout << ", min: `" << it->minValue << "`";
			if (!it->maxValue.empty())
				cout << ", max: `" << it->maxValue << "`";
			cout << ")" << endl;
			cout << endl;
			cout << " . " << it->doc << endl;
			cout << endl;
		}
	else
		cout << " . no parameters" << endl;
}

template<typename R>
void dumpRegistrar(const PM& pm, const R& registrar, const std::string& name, CurrentBibliography& bib)
{
	if (bib.mode == CurrentBibliography::ROSWIKI)
		cout << "=== " << name << " ===\n" << endl;
	else
		cout << "* " << name << " *\n" << endl;
	for (BOOST_AUTO(it, registrar.begin()); it != registrar.end(); ++it)
	{
		if (bib.mode == CurrentBibliography::ROSWIKI)
			cout << "==== " << it->first << " ====\n" << endl;
		else
			cout << it->first << endl;
		
		cout << getAndReplaceBibEntries(it->second->description(), bib) << endl;
		if (bib.mode == CurrentBibliography::ROSWIKI)
			dumpWiki(it->second->availableParameters());
		else
			cout << it->second->availableParameters();
		cout << endl;
	}
	cout << endl;
}


#define DUMP_REGISTRAR_CONTENT(pm, name, bib) \
	dumpRegistrar(pm, pm.REG(name), # name, bib);

void listModulesFull(const CurrentBibliography::Mode mode)
{
	CurrentBibliography bib(mode);
	
	DUMP_REGISTRAR_CONTENT(PM::get(), Transformation, bib)
	DUMP_REGISTRAR_CONTENT(PM::get(), DataPointsFilter, bib)
	DUMP_REGISTRAR_CONTENT(PM::get(), Matcher, bib)
	DUMP_REGISTRAR_CONTENT(PM::get(), OutlierFilter, bib)
	DUMP_REGISTRAR_CONTENT(PM::get(), ErrorMinimizer, bib)
	DUMP_REGISTRAR_CONTENT(PM::get(), TransformationChecker, bib)
	DUMP_REGISTRAR_CONTENT(PM::get(), Inspector, bib)
	DUMP_REGISTRAR_CONTENT(PM::get(), Logger, bib)
	
	printBibliographyHeader(mode);
	bib.dump(cout);
}


typedef vector<string> ModuleNameList;

template<typename R>
void dumpRegistrarSummary(const PM& pm, const R& registrar, const std::string& name, ModuleNameList& nameList)
{
	nameList.push_back(name);
	for (BOOST_AUTO(it, registrar.begin()); it != registrar.end(); ++it)
	{
		// TODO: remove title
		nameList.push_back(it->first);
	}
}

#define DUMP_REGISTRAR_SUMMARY(pm, name, nameList) \
	dumpRegistrarSummary(pm, pm.REG(name), # name, nameList);

void listModulesSummary(const CurrentBibliography::Mode mode)
{
	typedef vector<ModuleNameList> ModulesNames;
	
	ModulesNames modulesNames;
	modulesNames.resize(7);
	
	DUMP_REGISTRAR_SUMMARY(PM::get(), DataPointsFilter, modulesNames[0])
	DUMP_REGISTRAR_SUMMARY(PM::get(), Matcher, modulesNames[1])
	DUMP_REGISTRAR_SUMMARY(PM::get(), OutlierFilter, modulesNames[2])
	DUMP_REGISTRAR_SUMMARY(PM::get(), ErrorMinimizer, modulesNames[3])
	DUMP_REGISTRAR_SUMMARY(PM::get(), TransformationChecker, modulesNames[4])
	DUMP_REGISTRAR_SUMMARY(PM::get(), Inspector, modulesNames[5])
	DUMP_REGISTRAR_SUMMARY(PM::get(), Logger, modulesNames[6])
	
	// strip names and count
	ModulesNames strippedModulesNames;
	strippedModulesNames.resize(modulesNames.size());
	unsigned maxCount(0);
	for (size_t i(0); i < modulesNames.size(); ++i)
	{
		unsigned count(0);
		for (BOOST_AUTO(jt, modulesNames[i].begin()); jt != modulesNames[i].end(); ++jt)
		{
			const string& name(*jt);
			if (jt == modulesNames[i].begin())
			{
				strippedModulesNames[i].push_back(name);
				count++;
			}
			else
			{
				string strippedName(name.substr(0, name.length() - (modulesNames[i][0].length())));
				if ((strippedName != "Null") && (strippedName != "Identity"))
				{
					strippedModulesNames[i].push_back(strippedName);
					count++;
				}
			}
		}
		maxCount = max(count, maxCount);
	}
	
	// header
	cout << "\\begin{tabularx}{\\textwidth}{";
	for (size_t i(0); i<strippedModulesNames.size(); ++i)
		cout << "l";
	cout << "}\n\\toprule\n";
	for (unsigned row(0); row < maxCount; ++row)
	{
		for (BOOST_AUTO(it, strippedModulesNames.begin()); it != strippedModulesNames.end(); ++it)
		{
			if (row < it->size())
			{
				const string& name((*it)[row]);
				cout << name;
			}
			if (it+1 != strippedModulesNames.end())
				cout << " & ";
		}
		if (row == 0)
			cout << "\\\\\n\\midrule\n";
		else
			cout << "\\\\\n";
	}
	// footer
	cout << "\\bottomrule\n\\end{tabularx}\n";
}

int main(int argc, char *argv[])
{
	// choose bibliography mode
	CurrentBibliography::Mode mode(CurrentBibliography::NORMAL);
	if (argc == 2)
	{
		const string cmd(argv[1]);
		if (cmd == "--latexsummary")
		{
			mode = CurrentBibliography::BIBTEX;
			listModulesSummary(mode);
		}
		else if (cmd == "--roswiki")
		{
			mode = CurrentBibliography::ROSWIKI;
			listModulesFull(mode);
		}
		else if (cmd == "--bibtex")
		{
			mode = CurrentBibliography::BIBTEX;
			listModulesFull(mode);
		}
		else
		{
			cerr << "Invalid command " << cmd << endl;
			return 1;
		}
	}
	else
		listModulesFull(mode);
	
	return 0;
}