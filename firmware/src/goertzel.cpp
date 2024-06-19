#include "goertzel.h"

Goertzel::Goertzel(double targetFreq, double sampleRate, size_t windowSize)
    : targetFreq(targetFreq), sampleRate(sampleRate), windowSize(windowSize),
      k(static_cast<int>(0.5 + (windowSize * targetFreq) / sampleRate)),
      omega((2.0 * M_PI * k) / windowSize),
      coeff(2.0 * cos(omega)), s1(0.0), s2(0.0), samples(windowSize, 0.0), currentIndex(0) {}

void Goertzel::addSample(double sample) {
    // Chatgpt suggested subtracting old sample here, voodoo magic
    // I do not understand why that works but it does
    double s = coeff * s1 - s2 + sample - samples[currentIndex];
    s2 = s1;
    s1 = s;
    samples[currentIndex] = sample;
    currentIndex = (currentIndex + 1) % windowSize;
}

double Goertzel::getMagnitude() {
    double real = s1 - s2 * cos(omega);
    double imag = s2 * sin(omega);
    return sqrt(real * real + imag * imag);
}
