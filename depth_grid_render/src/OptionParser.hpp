#ifndef OPTION_PARSER_
#define OPTION_PARSER_

#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>

class OptionParser {
public:
    OptionParser(const int &argc, char **argv) : dirty(true) {
        for (int i=1; i<argc; i++) {
            tokens.emplace_back(argv[i]);
        }
    }
    bool cmdOptionExists(const std::string option) {
        return std::find(tokens.begin(), tokens.end(), "-" + option) != tokens.end();
    }
    std::string get(const std::string option) {
        std::vector<std::string>::const_iterator itr = std::find(tokens.begin(), tokens.end(), "-" + option);
        if (itr != tokens.end() && ++itr != tokens.end()) {
            return *itr;
        }
        return "";
    }
    std::string get(int index) {
        if (dirty) computeArgs();
        if (index < args.size()) {
            return args[index];
        }
        return "";
    }

  void addArg(std::string option, std::string default_value) {
    if (!cmdOptionExists(option)) {
      tokens.push_back("-" + option);
      tokens.push_back(default_value);
    }
    
  }

    int getNumArguments() {
        if (dirty) computeArgs();
        return args.size();
    }
private:
    void computeArgs() {
        args.clear();
        for (int i=0; i<tokens.size(); i++) {
            const std::string &tok = tokens[i];
            if (tok[0] == '-') {
                i++;
                continue;
            }
            args.push_back(tok);
        }
        dirty = false;
    }
    std::vector<std::string> tokens;
    std::vector<std::string> args;
    bool dirty;
};

#endif
