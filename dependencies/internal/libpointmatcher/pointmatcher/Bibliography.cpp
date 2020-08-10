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

#include "Bibliography.h"

#include <boost/typeof/typeof.hpp>
#include <boost/lexical_cast.hpp>
#define BOOST_ASSIGN_MAX_PARAMS 6
#include <iostream>
#include <cassert>
#include <stdexcept>

namespace PointMatcherSupport
{
	using namespace std;
	
	template<typename M>
	bool contains(const M& m, const typename M::key_type& k)
	{
		BOOST_AUTO(it,m.find(k));
		return (it!=m.end());
	}
	
	template<typename M>
	const typename M::mapped_type& get(const M& m, const typename M::key_type& k)
	{
		BOOST_AUTO(it,m.find(k));
		if (it!=m.end())
			return it->second;
		else
			throw std::runtime_error("unknown key");
	}
	
	static Bibliography bibliography()
	{
		return {
			{"Phillips2007VarTrimmed", {
				{"type", "inproceedings"},
				{"title", "Outlier robust ICP for minimizing fractional RMSD"},
				{"author", "Phillips, J.M. and Liu, R. and Tomasi, C."},
				{"booktitle", "3-D Digital Imaging and Modeling, 2007. 3DIM '07. Sixth International Conference on"},
				{"year", "2007"},
				{"pages", "427--434"},
				{"publisher", "IEEE Press"},
				{"doi", "10.1109/3DIM.2007.39"},
				{"fulltext", "http://x86.cs.duke.edu/~tomasi/papers/phillips/phillips3DIM07.pdf"}
			}},
			{"Chetverikov2002Trimmed", {
				{"type", "inproceedings"},
				{"title", "The Trimmed Iterative Closest Point Algorithm"},
				{"author", "Chetverikov, D. and Svirko, D. and Stepanov, D. and Krsek, P."},
				{"booktitle", "Pattern Recognition, 2002. Proceedings. 16th International Conference on"},
				{"year", "2002"},
				{"pages", "545--548"},
				{"publisher", "IEEE Press"},
				{"doi", "10.1109/ICPR.2002.1047997 "},
				{"fulltext", "http://hci.iwr.uni-heidelberg.de/publications/dip/2002/ICPR2002/DATA/10_1_03.PDF"}
			}},
			{"Besl1992Point2Point", {
				{"type", "inproceedings"},
				{"title", "A Method for Registration of 3-D Shapes"},
				{"author", "Besl, P.J. and McKay, H.D."},
				{"booktitle", "Pattern Analysis and Machine Intelligence, IEEE Transactions"},
				{"year", "1992"},
				{"pages", "239--256"},
				{"publisher", "IEEE Press"},
				{"doi", "10.1109/34.121791"},
				{"fulltext", "http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=121791"}
			}},
			{"Censi2007ICPCovariance", {
				{"type", "inproceedings" },
				{"title", "An Accurate Closed-Form Estimate of {ICP}'s Covariance" },
				{"author", "Censi, A." },
				{"booktitle", "Proceedings of the {IEEE} International Conference on Robotics and Automation ({ICRA})" },
				{"year", "2007" },
				{"pages", "3167--3172" },
				{"publisher", "IEEE Press" },
				{"doi", "10.1109/ROBOT.2007.363961" },
				{"fulltext", "http://purl.org/censi/research/2007-icra-icpcov.pdf"}
			}},
			{"Chen1991Point2Plane", {
				{"type", "inproceedings"},
				{"title", "Object modeling by registration of multiple range images"},
				{"author", "Chen, Y. and Medioni, G."},
				{"booktitle", "Robotics and Automation, 1991. Proceedings., 1991 IEEE International Conference on"},
				{"year", "1991"},
				{"pages", "2724--2729"},
				{"publisher", "IEEE Press"},
				{"doi", "10.1109/ROBOT.1991.132043"},
				{"fulltext", "http://ieeexplore.ieee.org/search/srchabstract.jsp?tp=&arnumber=132043"}
			}},
			{"Masuda1996Random", {
				{"type", "inproceedings"},
				{"title", "Registration and integration of multiple range images for 3-D model construction"},
				{"author", "Masuda, T. and Sakaue, K. and Yokoya, N."},
				{"booktitle", "Pattern Recognition, 1996., Proceedings of the 13th International Conference on"},
				{"year", "1996"},
				{"pages", "879--883"},
				{"publisher", "IEEE Press"},
				{"doi", "10.1109/ICPR.1996.546150"},
				{"fulltext", "http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=546150"}
			}},
			{"Diebel2004Median", {
				{"type", "inproceedings"},
				{"title", "Simultaneous Localization and Mapping with Active Stereo Vision"},
				{"author", "Diebel, J. and Reutersward, K. and Thrun, S. and Davis, J. and Gupta, R."},
				{"booktitle", "Intelligent Robots and Systems, 2004. (IROS 2004). Proceedings. 2004 IEEE/RSJ International Conference on"},
				{"year", "2004"},
				{"pages", "3436--3443"},
				{"publisher", "IEEE Press"},
				{"doi", "10.1109/IROS.2004.1389948"},
				{"fulltext", "http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1389948"}
			}},
			{"Pomerleau2012Noise", {
				{"type", "inproceedings"},
				{"title", "Noise Characterization of Depth Sensors for Surface Inspections"},
				{"author", "F. Pomerleau, A. Breitenmoser, M. Liu, F. Colas, R. Siegwart"},
				{"booktitle", " International Conference on Applied Robotics for the Power Industry, 2012. (CARPI 2012). Proceedings of the IEEE"},
				{"year", "2012"},
				{"pages", "1--8"},
				{"publisher", "IEEE Press"},
				{"doi", ""},
				{"fulltext", ""}
			}},
			{"RobustWeightFunctions", {
				{"type", "article"},
				{"title", "Robust regression using iteratively reweighted least-squares"},
				{"author", "Paul W Holland and Roy E Welsch"},
				{"booktitle", "Communications in Statistics - Theory and Methods"},
				{"year", "1977"},
				{"pages", "813-827"},
				{"publisher", ""},
				{"doi", "10.1080/03610927708827533"},
				{"fulltext", ""}
			}},
			{"Umeyama1991", {
				{"type", "article"},
				{"author", "Umeyama, Shinji"},
				{"title", "Least-Squares Estimation of Transformation Parameters Between Two Point Patterns"},
				{"journal", "IEEE Trans. Pattern Anal. Mach. Intell."},
				{"issue_date", "April 1991"},
				{"volume", "13"},
				{"number", "4"},
				{"month", "apr"},
				{"year", "1991"},
				{"issn", "0162-8828"},
				{"pages", "376--380"},
				{"numpages", "5"},
				{"url", "https://doi.org/10.1109/34.88573"},
				{"doi", "10.1109/34.88573"},
				{"acmid", "105525"},
				{"publisher", "IEEE Computer Society"},
				{"address", "Washington, DC, USA"}
			}},
			{"Rusinkiewicz2001", {
				{"type", "inproceedings"},
				{"author", "Rusinkiewicz, Szymon and Levoy, Marc"},
				{"title", "Efficient Variants of the ICP Algorithm"},
				{"journal", "Proceedings Third International Conference on 3-D Digital Imaging and Modeling"},
				{"year", "2001"},
				{"isbn", "0769509843"},
				{"pages", "145--152"},
				{"doi", "10.1109/IM.2001.924423"},
				{"publisher", "IEEE Computer Society"},
				{"address", "Quebec City, Quebec, Canada"}
			}},
			{"Gelfand2003", {
				{"type", "inproceedings"},
				{"author", "Gelfand, N. and Ikemoto, L. and Rusinkiewicz, Szymon and Levoy, M."},
				{"title", "Geometrically stable sampling for the ICP algorithm"},
				{"journal", "Fourth International Conference on 3-D Digital Imaging and Modeling, 2003. 3DIM 2003. Proceedings."},
				{"year", "2003"},
				{"isbn", "0-7695-1991-1"},
				{"pages", "260--267"},
				{"doi", "10.1109/IM.2003.1240258"},
				{"publisher", "IEEE Computer Society"}
			}}
		};
	}
	
	CurrentBibliography::CurrentBibliography(Mode mode):
		mode(mode)
	{}
	
	void CurrentBibliography::dump(std::ostream& os) const
	{
		switch (mode)
		{
			case NORMAL: dumpText(os); break;
			case ROSWIKI: dumpWiki(os); break;
			case BIBTEX: dumpBibtex(os); break;
			default: assert(false); break;
		};
	}
	
	void CurrentBibliography::dumpText(std::ostream& os) const
	{
		Bibliography biblio(bibliography());
		for (size_t i = 0; i < entries.size(); ++i)
		{
			const string& entryName(entries[i]);
			if (!contains(biblio, entryName))
				throw runtime_error(string("Broken bibliography, missing entry " + entryName));
			const StringMap& entry(get(biblio, entryName));
			
			os << "[" << i+1 << "]";
			if (contains(entry, "title"))
				os << " " << get(entry, "title") << ".";
			if (contains(entry, "author"))
				os << " " << get(entry, "author") << "";
			if (contains(entry, "booktitle"))
				os << " In " << get(entry, "booktitle") << ".";
			if (contains(entry, "journal"))
				os << " " << get(entry, "journal") << ".";
			if (contains(entry, "pages"))
				os << " " << get(entry, "pages") << ".";
			if (contains(entry, "year"))
				os << " " << get(entry, "year") << ".";
			os << endl << endl;
		}
	}

	void CurrentBibliography::dumpWiki(std::ostream& os) const
	{
		Bibliography biblio(bibliography());
		for (size_t i = 0; i < entries.size(); ++i)
		{
			const string& entryName(entries[i]);
			if (!contains(biblio, entryName))
				throw runtime_error(string("Broken bibliography, missing entry " + entryName));
			const StringMap& entry(get(biblio, entryName));
			
			os << " * " << "<<Anchor(" << entryName << ")>>[" << i+1 << "] -";
			if (contains(entry, "title"))
				os << " '''" << get(entry, "title") << ".'''";
			if (contains(entry, "author"))
				os << " " << get(entry, "author") << "";
			if (contains(entry, "booktitle"))
				os << " ''In " << get(entry, "booktitle") << ".''";
			if (contains(entry, "journal"))
				os << " " << get(entry, "journal") << ".";
			if (contains(entry, "pages"))
				os << " " << get(entry, "pages") << ".";
			if (contains(entry, "year"))
				os << " " << get(entry, "year") << ".";
			if (contains(entry, "doi"))
				os << " DOI: [[https://doi.org/" << get(entry, "doi") << "|" << get(entry, "doi") << "]].";
			if (contains(entry, "fulltext"))
				os << " [[" << get(entry, "fulltext") << "|full text]].";
			os << endl;
		}
	}

	void CurrentBibliography::dumpBibtex(std::ostream& os) const
	{
		Bibliography biblio(bibliography());
		for (size_t i = 0; i < entries.size(); ++i)
		{
			const string& entryName(entries[i]);
			if (!contains(biblio, entryName))
				throw runtime_error(string("Broken bibliography, missing entry " + entryName));
			const StringMap& entry(get(biblio, entryName));
			
			os << "@" << get(entry, "type") << "{" << entryName << endl;
			if (contains(entry, "title"))
				os << "\ttitle={" << get(entry, "title") << "}," << endl;
			if (contains(entry, "author"))
				os << "\tauthor={" << get(entry, "author") << "}," << endl;
			if (contains(entry, "booktitle"))
				os << "\tbooktitle={" << get(entry, "booktitle") << "}," << endl;
			if (contains(entry, "journal"))
				os << "\tjournal={" << get(entry, "journal") << "}," << endl;
			if (contains(entry, "pages"))
				os << "\tpages={" << get(entry, "pages") << "}," << endl;
			if (contains(entry, "year"))
				os << "\tyear={" << get(entry, "year") << "}," << endl;
			os << "}" << endl << endl;
		}
	}
	
	static StringVector splitString(const string& text, char delim)
	{
		StringVector res;
		size_t pos = 0;
		while(true)
		{
			const size_t nextPos = text.find(delim, pos);
			if (nextPos == text.npos)
			{
				res.push_back(text.substr(pos));
				break;
			}
			else
				res.push_back(text.substr(pos, nextPos - pos));
			pos = nextPos + 1;
		}
		return res;
	}
	
	std::string getAndReplaceBibEntries(const std::string& text, CurrentBibliography& curBib)
	{
		CurrentBibliography::Mode mode(curBib.mode);
		BibIndices& indices(curBib.indices);
		StringVector& entries(curBib.entries);
		
		string newText;
		const StringVector words(splitString(text, ' '));
		for (size_t i = 0; i < words.size(); ++i)
		{
			const string& word(words[i]);
			const size_t l(word.length());
			const size_t p(word.find('}'));
			if ((l > 7) && (word.substr(0, 6) == "\\cite{") && (p != string::npos))
			{
				if (mode == CurrentBibliography::ROSWIKI)
					newText += "&#91;";
				else
					newText += '[';
				const StringVector keys(splitString(word.substr(6, p-6), ','));
				for (size_t j = 0; j < keys.size(); ++j)
				{
					const string key(keys[j]);
					if (mode == CurrentBibliography::ROSWIKI)
						newText += "[[#" + key + "|";
					if (contains(indices, key))
					{
						newText += std::to_string(get(indices, key)+1);
					}
					else
					{
						size_t index(entries.size());
						entries.push_back(key);
						indices[key] = index;
						newText += std::to_string(index+1);
					}
					if (mode == CurrentBibliography::ROSWIKI)
						newText += "]]";
					if (j+1 != keys.size())
						newText += ',';
				}
				if (mode == CurrentBibliography::ROSWIKI)
					newText += "&#93;";
				else
					newText += ']';
				newText += word.substr(p+1);
			}
			else
				newText += word;
			if (i+1 != words.size())
				newText += ' ';
		}
		if (mode == CurrentBibliography::BIBTEX)
			return text;
		else
			return newText;
	}
}; // PointMatcherSupport
