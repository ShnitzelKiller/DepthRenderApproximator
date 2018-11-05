#ifndef UNDERSCORE_STR_H
#define UNDERSCORE_STR_H

#include <iostream>
#include <string>

class UnderscoreString : public std::string {

};

std::istream& operator>>(std::istream& is, UnderscoreString& output)
{
    std::getline(is, output, '_');
    return is;
}

#endif