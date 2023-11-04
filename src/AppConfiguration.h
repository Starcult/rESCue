#ifndef __APP_CONFIGURATION_H__
#define __APP_CONFIGURATION_H__

#include <config.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "visit_struct.hh"
#include "visit_struct_intrusive.hh"

// visitable struct, see C++ Visitor-Pattern and https://github.com/garbageslam/visit_struct
struct Config {
  BEGIN_VISITABLES(Config);
    VISITABLE(String, deviceName);
    VISITABLE(boolean, otaUpdateActive);
    VISITABLE(boolean, isNotificationEnabled);
    VISITABLE(boolean, isBatteryNotificationEnabled);
    VISITABLE(boolean, isCurrentNotificationEnabled);
    VISITABLE(boolean, isErpmNotificationEnabled);
    VISITABLE(double, minBatteryVoltage);
    VISITABLE(double, lowBatteryVoltage);
    VISITABLE(double, maxBatteryVoltage);
    VISITABLE(double, maxAverageCurrent);
    VISITABLE(double, brakeLightMinAmp);
    VISITABLE(double, batteryDrift);
    VISITABLE(int, startSoundIndex);
    VISITABLE(int, startLightIndex);
    VISITABLE(int, batteryWarningSoundIndex);
    VISITABLE(int, batteryAlarmSoundIndex);
    VISITABLE(int, startLightDuration);
    VISITABLE(int, idleLightIndex);
    VISITABLE(int, lightFadingDuration);
    VISITABLE(int, lightMaxBrightness);
    VISITABLE(int, lightColorPrimary);
    VISITABLE(int, lightColorPrimaryRed);
    VISITABLE(int, lightColorPrimaryGreen);
    VISITABLE(int, lightColorPrimaryBlue);
    VISITABLE(int, lightColorSecondary);
    VISITABLE(int, lightColorSecondaryRed);
    VISITABLE(int, lightColorSecondaryGreen);
    VISITABLE(int, lightColorSecondaryBlue);
    VISITABLE(int, lightbarTurnOffErpm);
    VISITABLE(int, lightbarMaxBrightness);
    VISITABLE(boolean, brakeLightEnabled);
    VISITABLE(int, numberPixelLight);
    VISITABLE(int, numberPixelBatMon);
    VISITABLE(int, vescId);
    VISITABLE(boolean, saveConfig);
    VISITABLE(boolean, sendConfig);
    VISITABLE(String , ledType);
    VISITABLE(String , lightBarLedType);
    VISITABLE(String , ledFrequency);
    VISITABLE(String , lightBarLedFrequency);
    VISITABLE(boolean , isLightBarReversed);
    VISITABLE(boolean , isLightBarLedTypeDifferent);
    VISITABLE(int , idleLightTimeout);
    VISITABLE(bool , mallGrab);
    VISITABLE(int , mtuSize);
    VISITABLE(boolean , oddevenActive);
    VISITABLE(boolean, lightsSwitch);
    VISITABLE(boolean, sendConfigFinished);
  END_VISITABLES;
};

class AppConfiguration {
  public: 
    static AppConfiguration* getInstance();
    boolean readPreferences();
    boolean savePreferences();
    boolean readMelodies();
    boolean saveMelodies();
    Config config;

  private:
    AppConfiguration() = default;
    static AppConfiguration *instance; 
    Preferences preferences; 
};
#endif