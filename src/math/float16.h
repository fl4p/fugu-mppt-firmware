#pragma once
//
//    FILE: float16.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.7
// PURPOSE: Arduino library to implement float16 data type.
//          half-precision floating point format, 
//          used for efficient storage and transport.
//     URL: https://github.com/RobTillaart/float16
//


#include "Arduino.h"

#define FLOAT16_LIB_VERSION                 (F("0.1.7"))


class float16
{
  public:
    // Constructors
    float16(void)               { _value = 0x0000; };
    float16(double f);
    float16(const float16 &f)   { _value = f._value; };
    float16 & operator=(const float16 &f) = default;

    //constexpr float16 & operator=(const float16 &f) = constexpr default; //   { return float16(f); };

    // Conversion
    float   toFloat(void) const;
    inline double toDouble(void) const { return toFloat(); }
    //  access the 2 byte representation.
    uint16_t getBinary()           { return _value; };
    void     setBinary(uint16_t u) { _value = u; };
    
    // Printable
    // size_t   printTo(Print& p) const;
    //void     setDecimals(uint8_t d) { _decimals = d; };
    //uint8_t  getDecimals()          { return _decimals; };

    // equalities
    bool operator == (const float16& f);
    bool operator != (const float16& f);

    bool operator >  (const float16& f);
    bool operator >= (const float16& f);
    bool operator <  (const float16& f);
    bool operator <= (const float16& f);

    // negation
    float16 operator - ();

    // basic math
    float16 operator + (const float16& f);
    float16 operator - (const float16& f);
    float16 operator * (const float16& f);
    float16 operator / (const float16& f);

    float16& operator += (const float16& f);
    float16& operator -= (const float16& f);
    float16& operator *= (const float16& f);
    float16& operator /= (const float16& f);

    // math helper functions
    int   sign();       // 1 = positive   0 = zero   -1 = negative.
    bool  isZero() const;
//    bool isNaN();
    bool isInf();


    // CORE CONVERSION
    // should be private but for testing...
    float    f16tof32(uint16_t) const;
    uint16_t f32tof16(const float&) const;

    explicit operator float() const { return toFloat(); }


  private:
    constexpr static uint8_t  _decimals = 4;
    uint16_t _value;

};


// -- END OF FILE --

