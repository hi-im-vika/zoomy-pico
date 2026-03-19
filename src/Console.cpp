#include "Console.hpp"

Console::~Console() {
    free(_lines);
}

void Console::init(U8G2 *u) {
    _u8g2 = u;
    _num_lines = u->getDisplayHeight() / u->getMaxCharHeight();
    _width_line = u->getDisplayWidth() / u->getMaxCharWidth();
    _height_line = u->getMaxCharHeight();

    _lines = (char**) malloc(_num_lines * sizeof(char*));
    for (int i = 0; i < _num_lines; i++) {
        *(_lines + i) = (char*) malloc(_width_line * sizeof(char));
        memset(*(_lines + i), 0, _width_line * sizeof(char));
    }
}

void Console::add(char *str) {
    for (int i = _num_lines - 1; i--; i < 0) {
        strcpy(_lines[i+1], _lines[i]);
    }
    strcpy(_lines[0], str);
}

void Console::append(char *str) {
    uint8_t left = _width_line - strlen(_lines[0]);
    strncat(_lines[0], str, left);
}

void Console::display() {
    _u8g2->clearBuffer();

    for (int i = 0; i < _num_lines; i++) {
        _u8g2->setCursor(0,_height_line * i);
        _u8g2->print(_lines[i]);
    }
    _u8g2->sendBuffer();
}