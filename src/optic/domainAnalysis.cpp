#include "domainAnalysis.h"

#ifdef STOCHASTICDURATIONS
#include "StochasticDurations.h"
#endif

#include <fstream>

using namespace TIM;
using namespace Inst;
using namespace VAL;
using namespace std;

using std::cerr;
using std::ostringstream;
using std::endl;
using std::ifstream;

domainAnalysis DomainAnalysis;

void domainAnalysis::readDomainActions()
{
	std:ifstream domainStream((this->domainRoute).c_str());

	string line;
	while (getline(domainStream, line))
	{
		cout << line << endl;
		if(line.find(":durative-action") != string::npos &&
				line.find(":requirements") == string::npos)
		{
			istringstream lineStream(line);
			string actionName;
			while(lineStream >> actionName)
			{
				if(actionName.find(":") == string::npos)
				{
					transform(actionName.begin(), actionName.end(), actionName.begin(), ::tolower);
					actionAnalysis* action = new actionAnalysis(actionName);
					this->actionList.push_back(action);
				}
			}
			while (getline(domainStream, line))
			{
				if(line.find(":parameters") != string::npos)
				{
					istringstream lineStream2(line);
					string word;
					string parameter = "", parameterType = "";
					bool storedLast = true;
					while(lineStream2 >> word)
					{
						if(word.find("?") != string::npos &&
								word.find(":") == string::npos)
						{
							if(!storedLast)
							{
								actionAnalysis* actIt = actionList.back();
								actIt->arguments.push_back(parameter);
								actIt->argumentType.push_back(parameterType);
								storedLast = true;
							}
							parameter = word.substr(word.find("?") + 1, word.length());
							if(parameter.find(")") != string::npos)
							{
								parameter = parameter.substr(0,parameter.find(")"));
							}
							transform(parameter.begin(), parameter.end(), parameter.begin(), ::tolower);
							storedLast = false;
						}else if(word.find("-") != string::npos &&
								word.find(":") == string::npos)
						{
							lineStream2 >> word;
							parameterType = word;
							if(parameterType.find(")") != string::npos)
							{
								parameterType = parameterType.substr(0,parameterType.find(")"));
							}
							transform(parameterType.begin(), parameterType.end(), parameterType.begin(), ::tolower);
							actionAnalysis* actIt = actionList.back();
							actIt->arguments.push_back(parameter);
							actIt->argumentType.push_back(parameterType);
							storedLast = true;
						}
					}

					break;
				}
			}
		}
	}
}

void domainAnalysis::readDomainPredicates()
{
	std:ifstream domainStream((this->domainRoute).c_str());

	string line;
	while (getline(domainStream, line))
	{
		cout << line << endl;
		if(line.find(":predicates") != string::npos)
		{
			while (getline(domainStream, line))
			{
				istringstream lineStream(line);
				string word;
				string parameter = "", parameterType = "";
				bool storedLast = true;

				lineStream >> word;
				if(word.find("(") != string::npos)
				{
					word = word.substr(word.find("(")+1, word.length());
				}
				else if(word == ")")
				{
					break;
				}
				transform(word.begin(), word.end(), word.begin(), ::tolower);
				predicateAnalysis* pred = new predicateAnalysis(word);
				this->predicateList.push_back(pred);

				while(lineStream >> word)
				{
					if(word.find("?") != string::npos &&
							word.find(":") == string::npos)
					{
						if(!storedLast)
						{
							predicateAnalysis* actIt = predicateList.back();
							actIt->arguments.push_back(parameter);
							actIt->argumentType.push_back(parameterType);
							storedLast = true;
						}
						parameter = word.substr(word.find("?") + 1, word.length());
						if(parameter.find(")") != string::npos)
						{
							parameter = parameter.substr(0,parameter.find(")"));
						}
						transform(parameter.begin(), parameter.end(), parameter.begin(), ::tolower);
						storedLast = false;
					}else if(word.find("-") != string::npos &&
							word.find(":") == string::npos)
					{
						lineStream >> word;
						parameterType = word;
						if(parameterType.find(")") != string::npos)
						{
							parameterType = parameterType.substr(0,parameterType.find(")"));
						}
						transform(parameterType.begin(), parameterType.end(), parameterType.begin(), ::tolower);
						predicateAnalysis* actIt = predicateList.back();
						actIt->arguments.push_back(parameter);
						actIt->argumentType.push_back(parameterType);
						storedLast = true;
					}
				}
			}
		}
	}
}

void domainAnalysis::readDomainFunctions()
{
	std:ifstream domainStream((this->domainRoute).c_str());

		string line;
		while (getline(domainStream, line))
		{
			cout << line << endl;
			if(line.find(":functions") != string::npos)
			{
				while (getline(domainStream, line))
				{
					istringstream lineStream(line);
					string word;
					string parameter = "", parameterType = "";
					bool storedLast = true;

					lineStream >> word;
					if(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(")+1, word.length());
					}
					else if(word == ")")
					{
						break;
					}
					transform(word.begin(), word.end(), word.begin(), ::tolower);
					functionAnalysis* pred = new functionAnalysis(word);
					this->functionList.push_back(pred);

					while(lineStream >> word)
					{
						if(word.find("?") != string::npos &&
								word.find(":") == string::npos)
						{
							if(!storedLast)
							{
								functionAnalysis* actIt = functionList.back();
								actIt->arguments.push_back(parameter);
								actIt->argumentType.push_back(parameterType);
								storedLast = true;
							}
							parameter = word.substr(word.find("?") + 1, word.length());
							if(parameter.find(")") != string::npos)
							{
								parameter = parameter.substr(0,parameter.find(")"));
							}
							transform(parameter.begin(), parameter.end(), parameter.begin(), ::tolower);
							storedLast = false;
						}else if(word.find("-") != string::npos &&
								word.find(":") == string::npos)
						{
							lineStream >> word;
							parameterType = word;
							if(parameterType.find(")") != string::npos)
							{
								parameterType = parameterType.substr(0,parameterType.find(")"));
							}
							transform(parameterType.begin(), parameterType.end(), parameterType.begin(), ::tolower);
							functionAnalysis* actIt = functionList.back();
							actIt->arguments.push_back(parameter);
							actIt->argumentType.push_back(parameterType);
							storedLast = true;
						}
					}
				}
			}
		}
}
