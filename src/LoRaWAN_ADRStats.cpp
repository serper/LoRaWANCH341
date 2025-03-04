#include "LoRaWAN.hpp"
#include <deque>
class ADRStats {
private:
    std::deque<float> snrHistory;
    std::deque<int> rssiHistory;
    const size_t MAX_SAMPLES = 10;

public:
    void addSnrSample(float snr) {
        snrHistory.push_back(snr);
        if (snrHistory.size() > MAX_SAMPLES) snrHistory.pop_front();
    }
    
    void addRssiSample(int rssi) {
        rssiHistory.push_back(rssi);
        if (rssiHistory.size() > MAX_SAMPLES) rssiHistory.pop_front();
    }
    
    float getAverageSnr() const {
        if (snrHistory.empty()) return 0;
        float sum = 0;
        for (auto snr : snrHistory) sum += snr;
        return sum / snrHistory.size();
    }
    
    int getAverageRssi() const {
        if (rssiHistory.empty()) return -120;
        int sum = 0;
        for (auto rssi : rssiHistory) sum += rssi;
        return sum / rssiHistory.size();
    }
    
    void reset() {
        snrHistory.clear();
        rssiHistory.clear();
    }
    
    bool hasEnoughSamples() const {
        return snrHistory.size() >= 5; // Mínimo 5 muestras para estadísticas confiables
    }
};
