#include "yaml-cpp-pm/yaml.h"
#include "yaml-cpp-pm/eventhandler.h"
#include <fstream>
#include <iostream>
#include <vector>

struct Params {
	bool hasFile;
	std::string fileName;
};

Params ParseArgs(int argc, char **argv) {
	Params p;

	std::vector<std::string> args(argv + 1, argv + argc);
	
	return p;
}

class NullEventHandler: public YAML_PM::EventHandler
{
public:
	virtual void OnDocumentStart(const YAML_PM::Mark&) {}
	virtual void OnDocumentEnd() {}
	
	virtual void OnNull(const YAML_PM::Mark&, YAML_PM::anchor_t) {}
	virtual void OnAlias(const YAML_PM::Mark&, YAML_PM::anchor_t) {}
	virtual void OnScalar(const YAML_PM::Mark&, const std::string&, YAML_PM::anchor_t, const std::string&) {}
	
	virtual void OnSequenceStart(const YAML_PM::Mark&, const std::string&, YAML_PM::anchor_t) {}
	virtual void OnSequenceEnd() {}
	
	virtual void OnMapStart(const YAML_PM::Mark&, const std::string&, YAML_PM::anchor_t) {}
	virtual void OnMapEnd() {}
};

void parse(std::istream& input)
{
	try {
		YAML_PM::Parser parser(input);
		YAML_PM::Node doc;
		while(parser.GetNextDocument(doc)) {
			YAML_PM::Emitter emitter;
			emitter << doc;
			std::cout << emitter.c_str() << "\n";
		}
	} catch(const YAML_PM::Exception& e) {
		std::cerr << e.what() << "\n";
	}
}

int main(int argc, char **argv)
{
	Params p = ParseArgs(argc, argv);

	if(argc > 1) {
		std::ifstream fin;
		fin.open(argv[1]);
		parse(fin);
	} else {
		parse(std::cin);
	}

	return 0;
}
