#pragma once
struct DummyEEPROM {template<typename T> void get(int,T&){}; template<typename T> void put(int,const T&){};};
extern DummyEEPROM EEPROM;
