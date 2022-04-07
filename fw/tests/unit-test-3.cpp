#include "../WARS-Birdhouse/Utils.h"
#include "../WARS-Birdhouse/ConfigurationImpl.h"

#include <EEPROM.h>
#include <iostream>

using namespace std;

class TestStream : public Stream {
public:

    void print(const char* m) { cout << m; }
    void print(char m) { cout << m; }
    void println() { cout << endl; }
    void println(const char* m) { cout << m << endl; }
};

void test_1() {

    CallSign my0("KC1FSZ");
    CallSign my1("KC1FSZ");
    assert(my0.equals(my1));
    assert(!my0.equals("XX1XXX"));

    char test[8];
    my0.writeTo(test);
    assert(test[0] == 'K');
    assert(test[7] == 0);

    CallSign my2("LLLLLLLLXXXXXXXXXX");
    my2.writeTo(test);
    assert(test[0] == 'L');
    assert(test[7] == 'L');

    const char test2[16] = { "XXXXXXXXYYYYYYY" };
    CallSign my3;
    my3.readFrom(test2);
    assert(my3.isEqual("XXXXXXXX"));
}

void test_2() {
    EEPROM.write(8,1);
    assert(1 == EEPROM.read(8));
}

void test_3() {

    TestStream stream;

    ConfigurationImpl config;
    config.setCall(CallSign("KC1FSZ"));
    config.setAddr(7);

    assert(my1.equals(CallSign("KC1FSZ")));
    assert(config.getAddr() == 7);
}

int main(int argc, const char** argv) {
    test_1();
    test_2();
    test_3();
    return 0;
}
