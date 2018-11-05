#ifndef XMLWRITER_H
#define XMLWRITER_H

#include <map>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <fstream>

class XMLElement {
public:
    XMLElement(std::string name) : _name(name) {}
    XMLElement(std::string name, std::string type) : _name(name) {
        AddProperty("type", type);
    }
    XMLElement(std::string type, std::string name, std::string value) : _name(type) {
        AddProperty("name", name);
        AddProperty("value", value);
    }
    void AddChild(const std::shared_ptr<XMLElement> &child) {
        children.push_back(child);
    }
    void AddProperty(std::string name, std::string value) {
        properties[name] = value;
    }
    void SaveXML(std::ofstream &of) const {
        SaveXML(of, 0);
    }
    void SaveXML(std::ofstream &of,int indent) const {
        for (int i=0; i<indent; i++) {
            of << " ";
        }
        of << "<" << _name;
        for (const auto &pair : properties) {
            of << " " << pair.first << "=" << "\"" << pair.second << "\"";
        }
        if (children.empty()) {
            of << "/";
        }
        of << ">" << std::endl;

        if (!children.empty()) {
            for (const auto &child : children) {
            child->SaveXML(of, indent + 1);
            }

            for (int i = 0; i < indent; i++) {
                of << " ";
            }
            of << "</" << _name << ">" << std::endl;
        }
    }
private:
    std::string _name;
    std::map<std::string, std::string> properties;
    std::vector<std::shared_ptr<XMLElement>> children;
};

#endif