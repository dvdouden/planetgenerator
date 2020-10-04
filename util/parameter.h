#pragma once

#include <vector>

#include <vlCore/vlnamespace.hpp>

template<typename T>
struct parameter {
    T value;
    bool dirty = true;

    parameter( T val ) :
            value(val ) { }

    parameter<T>& operator=( T newValue ) {
        if ( value != newValue ) {
            value = newValue;
            dirty = true;
        }
        return *this;
    }

    T operator()() {
        return value;
    }

    void clear() {
        dirty = false;
    }
};

template<typename T>
struct numericParameter : public parameter<T> {

    numericParameter( T val ) :
            parameter<T>( val ) {}

    numericParameter<T>& operator+=( T rhs ) {
        parameter<T>::value += rhs;
        parameter<T>::dirty = true;
        return *this;
    }

    numericParameter<T>& operator-=( T rhs ) {
        parameter<T>::value -= rhs;
        parameter<T>::dirty = true;
        return *this;
    }

    bool operator>( T rhs ) const {
        return parameter<T>::value > rhs;
    }

    bool operator<( T rhs ) const {
        return parameter<T>::value < rhs;
    }
};


struct parameterBindingBase {
    parameterBindingBase( vl::EKey incKey, vl::EKey decKey, bool generate) :
            incKey( incKey ),
            decKey( decKey ),
            generate( generate ){

    }
    virtual ~parameterBindingBase() = default;

    virtual void inc() = 0;
    virtual void dec() = 0;

    vl::EKey incKey;
    vl::EKey decKey;
    bool generate;
};

template<typename T>
struct parameterBinding : public parameterBindingBase {
    parameterBinding( numericParameter<T>& value, T min, T max, T increments, vl::EKey incKey, vl::EKey decKey, bool generate ) :
            parameterBindingBase( incKey, decKey, generate ),
            value( value ),
            min( min ),
            max( max ),
            increments( increments ) {

    }
    ~parameterBinding() override = default;

    void inc() override {
        value += increments;
        if ( value > max ) {
            value = max;
        }
    }
    void dec() override {
        value -= increments;
        if ( value < min ) {
            value = min;
        }
    }
    numericParameter<T>& value;
    T min;
    T max;
    T increments;
};


template<typename T>
struct constantParameterBinding : public parameterBindingBase {
    constantParameterBinding( numericParameter<T>& param, T value, vl::EKey key, bool generate ) :
            parameterBindingBase( key, key, generate ),
            param( param ),
            value( value ) {

    }
    ~constantParameterBinding() override = default;

    void inc() override {
        param = value;
    }
    void dec() override {

    }
    numericParameter<T>& param;
    T value;
};

struct boolParameterBinding : public parameterBindingBase {
    boolParameterBinding( parameter<bool>& param, vl::EKey key, bool generate ) :
            parameterBindingBase( key, key, generate ),
            param( param )
    {}

    ~boolParameterBinding() override = default;

    void inc() override {
        param = !param();
    }

    void dec() override {

    }

    parameter<bool>& param;
};

typedef std::shared_ptr<parameterBindingBase> param_ptr;

