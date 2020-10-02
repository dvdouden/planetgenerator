#pragma once


#include <string>
#include <utility>
#include <vector>
#include <chrono>

class Profiler {
public:
    typedef std::vector<std::pair<std::string,long long>> resultset;
    explicit Profiler( const std::string& name )
    {
        this->operator()(name);
    }

    void operator()( const std::string& event ) {
        timestamps.emplace_back( event, std::chrono::high_resolution_clock::now() );
    }

    resultset results() {
        auto end = std::chrono::high_resolution_clock::now();
        resultset times;
        times.emplace_back( timestamps[0].first, std::chrono::duration_cast<std::chrono::microseconds>(end - timestamps[0].second).count());
        for ( int i = 1; i < timestamps.size(); ++i ) {
            times.emplace_back( timestamps[i].first, std::chrono::duration_cast<std::chrono::microseconds>(timestamps[i].second - timestamps[i-1].second).count());
        }

        return std::move(times);
    }

private:
    std::vector<std::pair<std::string, std::chrono::high_resolution_clock::time_point>> timestamps;
};



