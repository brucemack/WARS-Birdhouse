#include "../WARS-Birdhouse/Utils.h"

#include <iostream>

using namespace std;

class TestStream : public Stream {
public:

    void print(const char* m) { cout << m; }
    void print(char m) { cout << m; }
    void println() { cout << endl; }
    void println(const char* m) { cout << m << endl; }
};

int main(int argc, const char** argv) {
    TestStream str;
    CallSign my("KC1FSZ");
    my.printTo(str);
    str.println();
    return 0;
}