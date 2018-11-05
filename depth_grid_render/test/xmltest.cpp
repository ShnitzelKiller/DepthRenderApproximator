//
// Created by James Noeckel on 10/30/18.
//

#include "../XMLWriter.hpp"
#include <iostream>
#include <fstream>

int main() {
    auto top = std::make_shared<XMLElement>("scene");
    top->AddProperty("version", "1.4");
    auto child = std::make_shared<XMLElement>("butt");
    child->AddProperty("fuck_factor", "30");
    top->AddChild(child);
    child->AddProperty("animals", "2.2");
    child->AddChild(std::make_shared<XMLElement>("forensics"));
    child->AddChild(std::make_shared<XMLElement>("integer", "butts", "30"));
    std::ofstream of("xmltest.xml");
    top->SaveXML(of);
}