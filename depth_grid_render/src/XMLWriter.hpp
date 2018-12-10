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
    /**
     * construct a rotation element around the axis (x, y, z)
     */
    template <class T>
    static std::shared_ptr<XMLElement> Rotation(T x, T y, T z, T angle) {
        auto rot = std::make_shared<XMLElement>("rotate");
        rot->AddProperty("x", std::to_string(x));
        rot->AddProperty("y", std::to_string(y));
        rot->AddProperty("z", std::to_string(z));
        rot->AddProperty("angle", std::to_string(angle));
        return rot;
    }

    /**
     * construct a rotation element around a single axis (0, 1, or 2)
     */
    template <class T>
    static std::shared_ptr<XMLElement> Rotation(int axis, T angle) {
        auto rot = std::make_shared<XMLElement>("rotate");
        switch (axis) {
            case 0:
                rot->AddProperty("x", "1");
                break;
            case 1:
                rot->AddProperty("y", "1");
                break;
            default:
                rot->AddProperty("z", "1");
                break;

        }
        rot->AddProperty("angle", std::to_string(angle));
        return rot;
    }

    /**
     * Construct a translation element
     */
    template <class T>
    static std::shared_ptr<XMLElement> Translation(T x, T y, T z) {
        auto trans = std::make_shared<XMLElement>("translate");
        if (x != 0)
            trans->AddProperty("x", std::to_string(x));
        if (y != 0)
            trans->AddProperty("y", std::to_string(y));
        if (z != 0)
            trans->AddProperty("z", std::to_string(z));
        return trans;
    }

    /**
     * Construct a scale element
     */
    template <class T>
    static std::shared_ptr<XMLElement> Scale(T x, T y, T z) {
        auto trans = std::make_shared<XMLElement>("scale");
        if (x != 1)
            trans->AddProperty("x", std::to_string(x));
        if (y != 1)
            trans->AddProperty("y", std::to_string(y));
        if (z != 1)
            trans->AddProperty("z", std::to_string(z));
        return trans;
    }

    static std::shared_ptr<XMLElement> Transform(std::string name) {
        auto transform = std::make_shared<XMLElement>("transform");
        transform->AddProperty("name", std::move(name));
        return transform;
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