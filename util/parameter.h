#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <vlCore/vlnamespace.hpp>
#include <vlCore/Say.hpp>

template<typename T, typename S>
struct parameter {
    T value;
    bool dirty = true;
    std::string name;
    std::string format;

    parameter( T val, const std::string& name, const std::string& format ) :
            value( val ),
            name( name ),
            format( format ) { }

    virtual parameter<T,S>& operator=( T newValue ) {
        if ( value != newValue ) {
            value = newValue;
            dirty = true;
        }
        return *this;
    }

    T operator()() const {
        return value;
    }

    virtual S say() const = 0;

    void clear() {
        dirty = false;
    }
};


template<typename T>
struct numericParameter : public parameter<T, T> {

    numericParameter( T val, const std::string& name ) :
            parameter<T, T>( val, name, "%n" ) {}

    numericParameter<T>& operator=( T rhs ) {
        parameter<T, T>::operator=( rhs );
        return *this;
    }

    bool operator>( T rhs ) const {
        return parameter<T, T>::value > rhs;
    }

    bool operator<( T rhs ) const {
        return parameter<T, T>::value < rhs;
    }

    T say() const override {
        return parameter<T, T>::value;
    }
};


struct boolParameter : public parameter<bool, std::string> {

    std::string strTrue;
    std::string strFalse;

    boolParameter( bool val, const std::string& name, const std::string& strTrue = "true", const std::string& strFalse = "false" ) :
            parameter<bool, std::string>( val, name, "%s" ),
                    strTrue( strTrue ),
                    strFalse( strFalse ) {}

    boolParameter& operator=( bool rhs ) override {
        parameter<bool, std::string>::operator=( rhs );
        return *this;
    }


    std::string say() const override {
        return parameter<bool, std::string>::value ? strTrue : strFalse;
    }
};



struct parameterBindingBase {
    parameterBindingBase( bool generate) :
            generate( generate ){

    }
    virtual ~parameterBindingBase() = default;

    virtual void pressed( vl::EKey key ) {};
    virtual void held( vl::EKey key ) {};
    virtual std::set<vl::EKey> getKeys() const = 0;

    virtual std::string name() const = 0;
    virtual std::string format() const = 0;
    virtual void say( vl::Say& say ) const = 0;

    bool generate;
};

template<typename T>
struct parameterBinding : public parameterBindingBase {
    parameterBinding( numericParameter<T>& param, T min, T max, T increments, vl::EKey incKey, vl::EKey decKey, bool generate ) :
            parameterBindingBase( generate ),
            param( param ),
            min( min ),
            max( max ),
            increments( increments ),
            incKey( incKey ),
            decKey( decKey ) {

    }

    ~parameterBinding() override = default;

    void pressed( vl::EKey key ) override {
        T newVal = param();
        if ( key == incKey ) {
            newVal += increments;
            if ( newVal > max ) {
                newVal = max;
            }
        } else if ( key == decKey ) {
            newVal -= increments;
            if ( newVal < min ) {
                newVal = min;
            }
        }
        param = newVal;
    }

    std::set<vl::EKey> getKeys() const override {
        std::set<vl::EKey> keys;
        keys.insert( incKey );
        keys.insert( decKey );
        return keys;
    }

    std::string name() const override {
        return param.name;
    }

    std::string format() const override {
        return param.format;
    }

    void say( vl::Say& say ) const override {
        say << param.say();
    }

    numericParameter<T>& param;
    T min;
    T max;
    T increments;
    vl::EKey incKey;
    vl::EKey decKey;
};


template<typename T>
struct multiplyingParameterBinding : public parameterBinding<T> {
    multiplyingParameterBinding( numericParameter<T>& param, T min, T max, T increments, vl::EKey incKey, vl::EKey decKey, bool generate ) :
            parameterBinding<T>( param, min, max, increments, incKey, decKey, generate ) {}

    void pressed( vl::EKey key ) override {
        T newVal = parameterBinding<T>::param();
        if ( key == parameterBinding<T>::incKey ) {
            newVal *= parameterBinding<T>::increments;
            if ( newVal > parameterBinding<T>::max ) {
                newVal = parameterBinding<T>::max;
            }
        } else if ( key == parameterBinding<T>::decKey ) {
            newVal /= parameterBinding<T>::increments;
            if ( newVal < parameterBinding<T>::min ) {
                newVal = parameterBinding<T>::min;
            }
        }
        parameterBinding<T>::param = newVal;
    }
};

template<typename T>
struct repeatingParameterBinding : public parameterBinding<T> {
    repeatingParameterBinding( numericParameter<T>& param, T min, T max, T increments, vl::EKey incKey, vl::EKey decKey, bool generate ) :
    parameterBinding<T>( param, min, max, increments, incKey, decKey, generate ) {}

    void pressed( vl::EKey key ) override {

    }
    void held( vl::EKey key ) override {
        parameterBinding<T>::pressed(key);
    }
};

template<typename T>
struct repeatingMultiplyingParameterBinding : public multiplyingParameterBinding<T> {
    repeatingMultiplyingParameterBinding( numericParameter<T>& param, T min, T max, T increments, vl::EKey incKey, vl::EKey decKey, bool generate ) :
            multiplyingParameterBinding<T>( param, min, max, increments, incKey, decKey, generate ) {}

    void pressed( vl::EKey key ) override {}

    void held( vl::EKey key ) override {
        multiplyingParameterBinding<T>::pressed( key );
    }

};

template<typename T>
struct constantParameterBinding : public parameterBindingBase {
    constantParameterBinding( numericParameter<T>& param, bool generate ) :
            parameterBindingBase(  generate ),
            param( param ) {
    }
    ~constantParameterBinding() override = default;

    void addKey( vl::EKey key, T value ) {
        values[key] = value;
    }

    void pressed( vl::EKey key ) override {
        param = values[key];
    }

    std::set<vl::EKey> getKeys() const override {
        std::set<vl::EKey> keys;
        std::transform(values.begin(), values.end(), std::inserter(keys, keys.end()),[](auto pair){ return pair.first; });
        return keys;
    }

    std::string name() const override {
        return param.name;
    }

    std::string format() const override {
        return param.format;
    }

    void say( vl::Say& say ) const override {
        say << param.say();
    }

    numericParameter<T>& param;
    std::map<vl::EKey, T> values;
};

struct boolParameterBinding : public parameterBindingBase {
    boolParameterBinding( boolParameter& param, vl::EKey key, bool generate ) :
            parameterBindingBase( generate ),
            param( param ),
            key( key )
    {}

    ~boolParameterBinding() override = default;

    void pressed( vl::EKey ) override {
        param = !param();
    }

    std::set<vl::EKey> getKeys() const override {
        std::set<vl::EKey> keys;
        keys.insert( key );
        return keys;
    }

    std::string name() const override {
        return param.name;
    }

    std::string format() const override {
        return param.format;
    }

    void say( vl::Say& say ) const override {
        say << param.say();
    }

    boolParameter& param;
    vl::EKey key;
};

typedef std::shared_ptr<parameterBindingBase> param_ptr;

