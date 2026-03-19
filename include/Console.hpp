#include <U8g2lib.h>

class Console {
    public:
    Console() = default;
    ~Console();
    void init(U8G2 *u);
    void add(char *str);
    void append(char *str);
    void display();

    private:
    char **_lines;
    U8G2 *_u8g2;
    uint8_t _num_lines;
    uint8_t _width_line;
    uint8_t _height_line;
};