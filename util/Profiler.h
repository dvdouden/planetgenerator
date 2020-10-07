#pragma once


#include <string>
#include <utility>
#include <vector>
#include <chrono>

class Profiler {
public:
    struct time_t {
        std::string name;
        long long us;
        std::vector<time_t> details;


        time_t( std::string name, long long us ) :
        name( std::move(name) ),
        us( us ) {}

        time_t( std::string name, long long us, std::vector<time_t> details ) :
                name( std::move(name) ),
                us( us ),
                details( std::move(details ) ){}
    };
    typedef std::vector<time_t> resultset;
    explicit Profiler( const std::string& name )
    {
        this->operator()(name);
    }

    void operator()( const std::string& event ) {
        timestamps.emplace_back( event, std::chrono::high_resolution_clock::now(),resultset() );
    }

    void operator()( const std::string& event, resultset details ) {
        timestamps.emplace_back( event, std::chrono::high_resolution_clock::now(),details );
    }

    resultset results() {
        auto end = std::chrono::high_resolution_clock::now();
        resultset times;
        times.emplace_back( timestamps[0].name, std::chrono::duration_cast<std::chrono::microseconds>(end - timestamps[0].time).count());
        for ( int i = 1; i < timestamps.size(); ++i ) {
            times.emplace_back( timestamps[i].name, std::chrono::duration_cast<std::chrono::microseconds>(timestamps[i].time - timestamps[i-1].time).count(), timestamps[i].details);
        }

        return std::move(times);
    }

private:
    struct timestamp_t {
        std::string name;
        std::chrono::high_resolution_clock::time_point time;
        resultset details;

        timestamp_t( std::string name, std::chrono::high_resolution_clock::time_point time ) :
                name( std::move(name) ),
                time( time ) {}

        timestamp_t( std::string name, std::chrono::high_resolution_clock::time_point time, resultset details ) :
        name( std::move(name) ),
        time( time ),
        details( std::move(details) ) {}
    };

    std::vector<timestamp_t> timestamps;
};



