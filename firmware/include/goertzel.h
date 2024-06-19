#ifndef GOERTZEL_H
#define GOERTZEL_H

#include <vector>
#include <cmath>

class Goertzel {
public:
    Goertzel(double targetFreq, double sampleRate, size_t windowSize);
    void addSample(double sample);
    double getMagnitude();

private:
    double targetFreq;
    double sampleRate;
    size_t windowSize;
    int k;
    double omega;
    double coeff;
    double s1;
    double s2;
    std::vector<double> samples;
    size_t currentIndex;
};

#endif // GOERTZEL_H
