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
    /**
     * construct a named element with no properties
     * @param name type of the XML tag
     */
    XMLElement(std::string name) : _name(name) {}
    /**
     * construct a a named element with property type="<type>", e.g. <sensor type="perspective">
     * @param name type of the XML tag
     * @param type value of the property "type"
     */
    XMLElement(std::string name, std::string type) : _name(name) {
        AddProperty("type", type);
    }
    /**
     * construct a named element with a name property and value property, e.g. <integer name="sampleCount" value="256">
     * @param type type of XML tag
     * @param name value of property "name"
     * @param value value of property "value"
     */
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
private:
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
    std::string _name;
    std::map<std::string, std::string> properties;
    std::vector<std::shared_ptr<XMLElement>> children;
};

#endif